// Package corridorfollower_test mirrors tests/unit/test_corridor_follower.py.
package corridorfollower_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/corridorfollower"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"
)

const (
	rays           = 360
	creepSpeedMPS  = 0.15
	farRangeM      = 3.0
	tolerance      = 1e-6
	turnEntryMargM = 0.05
)

// corridorScan builds a sweep of a straight corridor: leftM at +90 degrees,
// rightM at -90, and aheadM across the whole forward arc both the clearance
// reading and the way-through test look at.
//
// The forward window is deliberately wider than the 8 degree clearance arc,
// so a scan that reads as "wall ahead" also reads as "no way through" --
// otherwise the corner branch could never fire.
func corridorScan(leftM, rightM, aheadM float64) controllers.LidarScan {
	const forwardHalfWindowRad = 20 * math.Pi / 180

	angles := make([]float64, rays)
	ranges := make([]float64, rays)
	for i := range angles {
		angle := -math.Pi + 2*math.Pi*float64(i)/float64(rays-1)
		angles[i] = angle
		switch {
		case math.Abs(navutil.WrapAngle(angle)) <= forwardHalfWindowRad:
			ranges[i] = aheadM
		default:
			ranges[i] = farRangeM
		}
	}
	ranges[nearestIndex(angles, math.Pi/2)] = leftM
	ranges[nearestIndex(angles, -math.Pi/2)] = rightM
	return controllers.LidarScan{RangesM: ranges, AnglesRad: angles}
}

func nearestIndex(angles []float64, target float64) int {
	best, bestErr := 0, math.Inf(1)
	for i, a := range angles {
		if err := math.Abs(navutil.WrapAngle(a - target)); err < bestErr {
			best, bestErr = i, err
		}
	}
	return best
}

func maxCenteringNorm(cfg corridorfollower.Config) float64 {
	return cfg.MaxCenteringSteerDeg * math.Pi / 180 / cfg.MaxSteeringAngleRad
}

func maxCornerNorm(cfg corridorfollower.Config) float64 {
	return cfg.MaxCornerSteerDeg * math.Pi / 180 / cfg.MaxSteeringAngleRad
}

// TestCornerTurn_WallSpanningTheTrackCommits covers the branch whose absence
// was every closed-loop failure of this feature: stopping at a corner with no
// direction yet means sitting there until the round expires.
func TestCornerTurn_WallSpanningTheTrackCommits(t *testing.T) {
	t.Parallel()

	cfg := corridorfollower.DefaultConfig()
	scan := corridorScan(0.8, 0.2, cfg.TurnClearanceM-turnEntryMargM)

	got := corridorfollower.FollowCorridor(
		scan,
		corridorfollower.Params{SpeedMPS: creepSpeedMPS},
		cfg,
	)

	if got.SpeedMPS <= 0 {
		t.Fatalf("SpeedMPS = %v, want forward motion into the turn", got.SpeedMPS)
	}
	if math.Abs(got.SteeringNorm-maxCornerNorm(cfg)) > tolerance {
		t.Fatalf(
			"SteeringNorm = %v, want the corner angle %v",
			got.SteeringNorm,
			maxCornerNorm(cfg),
		)
	}
}

// TestCornerTurn_TurnsTowardTheRoomierSide checks the turn agrees with the
// observation the direction estimator settles on.
func TestCornerTurn_TurnsTowardTheRoomierSide(t *testing.T) {
	t.Parallel()

	cfg := corridorfollower.DefaultConfig()
	ahead := cfg.TurnClearanceM - turnEntryMargM

	leftRoomier := corridorScan(0.8, 0.2, ahead)
	if got := corridorfollower.FollowCorridor(
		leftRoomier, corridorfollower.Params{SpeedMPS: creepSpeedMPS}, cfg,
	); got.SteeringNorm <= 0 {
		t.Fatalf("SteeringNorm = %v, want a left turn toward the roomier side", got.SteeringNorm)
	}

	rightRoomier := corridorScan(0.2, 0.8, ahead)
	if got := corridorfollower.FollowCorridor(
		rightRoomier, corridorfollower.Params{SpeedMPS: creepSpeedMPS}, cfg,
	); got.SteeringNorm >= 0 {
		t.Fatalf("SteeringNorm = %v, want a right turn toward the roomier side", got.SteeringNorm)
	}
}

// TestCornerTurn_OpenCorridorDoesNotTurn covers the way-through test: a wall
// seen at an angle is not a corner, and must not trigger one.
func TestCornerTurn_OpenCorridorDoesNotTurn(t *testing.T) {
	t.Parallel()

	cfg := corridorfollower.DefaultConfig()
	// Close enough to trip the clearance test, but the arc is wide open.
	scan := corridorScan(0.5, 0.5, 2.5)

	got := corridorfollower.FollowCorridor(
		scan,
		corridorfollower.Params{SpeedMPS: creepSpeedMPS},
		cfg,
	)

	if math.Abs(got.SteeringNorm) > tolerance {
		t.Fatalf("SteeringNorm = %v, want no turn in an open corridor", got.SteeringNorm)
	}
	if math.Abs(got.SpeedMPS-creepSpeedMPS) > tolerance {
		t.Fatalf("SpeedMPS = %v, want the full creep speed", got.SpeedMPS)
	}
}

// TestCornerTurn_DroppedBeamCannotVetoACorner covers the dropout exclusion.
// The gateway substitutes max range for a no-return, and nothing on a 3 m mat
// can be further than its diagonal -- left in, one dropped beam reads as
// wide-open track and vetoes every corner on the round.
func TestCornerTurn_DroppedBeamCannotVetoACorner(t *testing.T) {
	t.Parallel()

	cfg := corridorfollower.DefaultConfig()
	scan := corridorScan(0.8, 0.2, cfg.TurnClearanceM-turnEntryMargM)
	scan.RangesM[nearestIndex(scan.AnglesRad, 0.0)] = 12.0 // a no-return, sanitized to max range

	got := corridorfollower.FollowCorridor(
		scan,
		corridorfollower.Params{SpeedMPS: creepSpeedMPS},
		cfg,
	)

	if math.Abs(got.SteeringNorm-maxCornerNorm(cfg)) > tolerance {
		t.Fatalf("SteeringNorm = %v, want the corner turn despite the dropout", got.SteeringNorm)
	}
}

// TestSafety_BacksOffWhenRearIsMeasurable covers the reversing branch. It can
// only be reached with a rear reading, which the shipped mount cannot produce
// -- the occlusion wedges have met at 180 degrees since 2026-08-22.
func TestSafety_BacksOffWhenRearIsMeasurable(t *testing.T) {
	t.Parallel()

	cfg := corridorfollower.DefaultConfig()
	scan := corridorScan(0.8, 0.2, 0.1)

	got := corridorfollower.FollowCorridor(scan, corridorfollower.Params{
		SpeedMPS:      creepSpeedMPS,
		RearClearance: func() (float64, bool) { return 1.0, true },
	}, cfg)

	if got.SpeedMPS >= 0 {
		t.Fatalf("SpeedMPS = %v, want reverse", got.SpeedMPS)
	}
	// Reversing swings the nose away from the steer direction, so the sign is
	// inverted against the corner branch's.
	if got.SteeringNorm >= 0 {
		t.Fatalf("SteeringNorm = %v, want the mirrored steer while reversing", got.SteeringNorm)
	}
}

// TestSafety_RefusesToBackOffWhenRearUnmeasurable is the important half: an
// unreadable rear is NOT permission to reverse. This was once a single raw
// ray straight back, which read 12 m of open road through an occlusion wedge
// and backed into whatever was behind.
func TestSafety_RefusesToBackOffWhenRearUnmeasurable(t *testing.T) {
	t.Parallel()

	cfg := corridorfollower.DefaultConfig()
	scan := corridorScan(0.8, 0.2, 0.1)

	for name, rear := range map[string]func() (float64, bool){
		"no rear sensor at all": nil,
		"rear unmeasurable":     func() (float64, bool) { return 0, false },
	} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()

			got := corridorfollower.FollowCorridor(scan, corridorfollower.Params{
				SpeedMPS:      creepSpeedMPS,
				RearClearance: rear,
			}, cfg)

			if got.SpeedMPS < 0 {
				t.Fatalf("SpeedMPS = %v, reversed on an unmeasurable rear", got.SpeedMPS)
			}
		})
	}
}

// TestSafety_PivotsOutOfTheBayInsteadOfHolding covers the 2026-08-27 fix. A
// robot started INSIDE the parking bay -- a legal start -- sat at exactly
// 0.00 m for a whole run, 8/8 scenarios, with the front against a fin and
// BOTH sides reading 12.0 m. There was an open corridor either side and the
// robot could see it.
func TestSafety_PivotsOutOfTheBayInsteadOfHolding(t *testing.T) {
	t.Parallel()

	cfg := corridorfollower.DefaultConfig()
	scan := corridorScan(12.0, 12.0, 0.1)

	got := corridorfollower.FollowCorridor(
		scan,
		corridorfollower.Params{SpeedMPS: creepSpeedMPS},
		cfg,
	)

	if got.SpeedMPS <= 0 {
		t.Fatalf("SpeedMPS = %v, want a forward pivot toward the open side", got.SpeedMPS)
	}
	if math.Abs(got.SteeringNorm) < tolerance {
		t.Fatal("SteeringNorm = 0, want full lock to swing the nose out")
	}
}

// TestSafety_HoldsWhenBoxedAtBothEnds checks the pivot is gated: a true dead
// end, boxed on three sides with nowhere to pivot to, still holds.
func TestSafety_HoldsWhenBoxedAtBothEnds(t *testing.T) {
	t.Parallel()

	cfg := corridorfollower.DefaultConfig()
	scan := corridorScan(0.2, 0.2, 0.1)

	got := corridorfollower.FollowCorridor(
		scan,
		corridorfollower.Params{SpeedMPS: creepSpeedMPS},
		cfg,
	)

	if math.Abs(got.SpeedMPS) > tolerance {
		t.Fatalf("SpeedMPS = %v, want a hold when boxed at both ends", got.SpeedMPS)
	}
}

// TestForcedTurnSide covers the WRO pass-side override: red outward, green
// inward has nothing to do with which side looks more open, and this function
// only ever sees robot-frame LIDAR so it cannot resolve that rule itself.
func TestForcedTurnSide(t *testing.T) {
	t.Parallel()

	cfg := corridorfollower.DefaultConfig()

	t.Run("corner branch honors the forced side", func(t *testing.T) {
		t.Parallel()

		// Left is roomier, so clearance alone would turn left.
		scan := corridorScan(0.8, 0.2, cfg.TurnClearanceM-turnEntryMargM)
		got := corridorfollower.FollowCorridor(scan, corridorfollower.Params{
			SpeedMPS:       creepSpeedMPS,
			ForcedTurnSide: corridorfollower.TurnSideRight,
		}, cfg)

		if got.SteeringNorm >= 0 {
			t.Fatalf(
				"SteeringNorm = %v, want the forced right turn over the roomier left",
				got.SteeringNorm,
			)
		}
	})

	t.Run("back-off branch honors the forced side", func(t *testing.T) {
		t.Parallel()

		scan := corridorScan(12.0, 12.0, 0.1)
		got := corridorfollower.FollowCorridor(scan, corridorfollower.Params{
			SpeedMPS:       creepSpeedMPS,
			ForcedTurnSide: corridorfollower.TurnSideRight,
		}, cfg)

		if got.SteeringNorm >= 0 {
			t.Fatalf("SteeringNorm = %v, want the forced right side", got.SteeringNorm)
		}
	})
}

// TestTurnSideNoneIsTheZeroValue guards a bug the behavioral tests above
// cannot see: if TurnSideNone stops being 0, a caller that never sets
// ForcedTurnSide still gets clearance-based steering (an unmatched value
// falls through the switch), so every other test keeps passing while
// TurnSideLeft silently means "none".
func TestTurnSideNoneIsTheZeroValue(t *testing.T) {
	t.Parallel()

	var unset corridorfollower.TurnSide
	if unset != corridorfollower.TurnSideNone {
		t.Fatalf("zero TurnSide = %v, want TurnSideNone", unset)
	}
	if corridorfollower.TurnSideLeft == corridorfollower.TurnSideNone ||
		corridorfollower.TurnSideRight == corridorfollower.TurnSideNone {
		t.Fatal("TurnSideLeft/Right collide with TurnSideNone")
	}
}

// TestCentring_ShippedCreepHoldsItsLane pins the zero gain as INTENTIONAL.
// Chasing the lateral offset is what swung the heading past the direction
// estimator's alignment gate, costing 11 of 32 wide outer-band starts their
// direction entirely.
func TestCentring_ShippedCreepHoldsItsLane(t *testing.T) {
	t.Parallel()

	cfg := corridorfollower.DefaultConfig()
	// Hard against one wall but square to the corridor.
	scan := corridorScan(0.8, 0.2, 2.5)

	got := corridorfollower.FollowCorridor(
		scan,
		corridorfollower.Params{SpeedMPS: creepSpeedMPS},
		cfg,
	)

	if math.Abs(got.SteeringNorm) > tolerance {
		t.Fatalf(
			"SteeringNorm = %v, want 0 -- the shipped creep must hold its lane",
			got.SteeringNorm,
		)
	}
	if math.Abs(got.SpeedMPS-creepSpeedMPS) > tolerance {
		t.Fatalf("SpeedMPS = %v, want the full creep speed", got.SpeedMPS)
	}
}

// TestCentring_SteersWhenTheGainIsRestored keeps the disabled branch from
// rotting unnoticed -- the same inert-configuration trap that has already
// cost this project several wrong conclusions. 44.0 is the value shipped
// until 2026-08-22.
func TestCentring_SteersWhenTheGainIsRestored(t *testing.T) {
	t.Parallel()

	cfg := corridorfollower.DefaultConfig()
	cfg.CenteringGainDegPerM = 44.0
	scan := corridorScan(0.8, 0.2, 2.5)

	got := corridorfollower.FollowCorridor(
		scan,
		corridorfollower.Params{SpeedMPS: creepSpeedMPS},
		cfg,
	)

	if got.SteeringNorm <= 0 {
		t.Fatalf("SteeringNorm = %v, want a left steer toward the roomier side", got.SteeringNorm)
	}
}

// TestHeadingDamping_ObliqueChassisSteersBackToAxis is the case offset-only
// centering cannot see: dead center, pointing wrong. Measured on hardware
// 2026-08-07 as a 3.2 s limit cycle with the heading 30 degrees off axis at
// the median, which starves the direction gate.
func TestHeadingDamping_ObliqueChassisSteersBackToAxis(t *testing.T) {
	t.Parallel()

	cfg := corridorfollower.DefaultConfig()
	scan := corridorScan(0.5, 0.5, 2.5)
	oblique := 20 * math.Pi / 180

	undamped := corridorfollower.FollowCorridor(
		scan, corridorfollower.Params{SpeedMPS: creepSpeedMPS}, cfg,
	)
	if math.Abs(undamped.SteeringNorm) > tolerance {
		t.Fatalf("undamped SteeringNorm = %v, want 0", undamped.SteeringNorm)
	}

	damped := corridorfollower.FollowCorridor(
		scan, corridorfollower.Params{SpeedMPS: creepSpeedMPS, Yaw: &oblique}, cfg,
	)
	if damped.SteeringNorm >= 0 {
		t.Fatalf(
			"damped SteeringNorm = %v, want a right steer for a nose left of axis",
			damped.SteeringNorm,
		)
	}
}

func TestHeadingDamping_CorrectionIsSignedByNoseDirection(t *testing.T) {
	t.Parallel()

	cfg := corridorfollower.DefaultConfig()
	scan := corridorScan(0.5, 0.5, 2.5)
	leftOfAxis, rightOfAxis := 20*math.Pi/180, -20*math.Pi/180

	left := corridorfollower.FollowCorridor(
		scan, corridorfollower.Params{SpeedMPS: creepSpeedMPS, Yaw: &leftOfAxis}, cfg,
	)
	right := corridorfollower.FollowCorridor(
		scan, corridorfollower.Params{SpeedMPS: creepSpeedMPS, Yaw: &rightOfAxis}, cfg,
	)

	if math.Abs(left.SteeringNorm+right.SteeringNorm) > tolerance {
		t.Fatalf(
			"left %v and right %v are not mirror images",
			left.SteeringNorm,
			right.SteeringNorm,
		)
	}
}

// TestHeadingDamping_SquareToAnyAxisNeedsNoCorrection covers the Manhattan
// world: every 90 degrees is "square".
func TestHeadingDamping_SquareToAnyAxisNeedsNoCorrection(t *testing.T) {
	t.Parallel()

	cfg := corridorfollower.DefaultConfig()
	scan := corridorScan(0.5, 0.5, 2.5)

	for quarter := range 4 {
		yaw := float64(quarter) * math.Pi / 2
		got := corridorfollower.FollowCorridor(
			scan, corridorfollower.Params{SpeedMPS: creepSpeedMPS, Yaw: &yaw}, cfg,
		)
		if math.Abs(got.SteeringNorm) > tolerance {
			t.Fatalf("axis %d asked for steering %v", quarter, got.SteeringNorm)
		}
	}
}

// TestHeadingDamping_CapStillBinds checks damping adds to the demand without
// widening the steering envelope.
func TestHeadingDamping_CapStillBinds(t *testing.T) {
	t.Parallel()

	cfg := corridorfollower.DefaultConfig()
	cfg.CenteringGainDegPerM = 44.0
	scan := corridorScan(0.9, 0.1, 2.5)
	yaw := -40 * math.Pi / 180

	got := corridorfollower.FollowCorridor(
		scan, corridorfollower.Params{SpeedMPS: creepSpeedMPS, Yaw: &yaw}, cfg,
	)

	if math.Abs(got.SteeringNorm) > maxCenteringNorm(cfg)+1e-9 {
		t.Fatalf(
			"SteeringNorm = %v exceeds the centering cap %v",
			got.SteeringNorm,
			maxCenteringNorm(cfg),
		)
	}
}

// TestNarrowCorridorTurnsLater covers the believed-width override: the
// wide-corridor threshold leaves no direction-settling window in a narrow one.
func TestNarrowCorridorTurnsLater(t *testing.T) {
	t.Parallel()

	cfg := corridorfollower.DefaultConfig()
	// Between the narrow and wide thresholds: a wide corridor would commit to
	// the turn here, a narrow one should still be creeping.
	ahead := (cfg.NarrowTurnClearanceM + cfg.TurnClearanceM) / 2
	scan := corridorScan(0.3, 0.3, ahead)

	wide := corridorfollower.FollowCorridor(
		scan,
		corridorfollower.Params{SpeedMPS: creepSpeedMPS},
		cfg,
	)
	if math.Abs(wide.SteeringNorm) < tolerance {
		t.Fatal("with no believed width, expected the wide threshold to commit to the turn")
	}

	narrow := 0.6
	got := corridorfollower.FollowCorridor(scan, corridorfollower.Params{
		SpeedMPS:       creepSpeedMPS,
		BelievedWidthM: &narrow,
	}, cfg)
	if math.Abs(got.SteeringNorm) > tolerance {
		t.Fatalf("SteeringNorm = %v, want a narrow corridor to keep creeping", got.SteeringNorm)
	}
}
