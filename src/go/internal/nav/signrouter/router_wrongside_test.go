package signrouter_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// TestWrongSideViolations_CorrectSideIsNotAViolation and its sibling match
// TestWrongSidePassDetection: a sign retired on the forbidden side must
// register as a violation (the simulator stops the run on this exactly as
// it does on a forbidden wall contact); a pass on the permitted side must
// not.
func TestWrongSideViolations_CorrectSideIsNotAViolation(t *testing.T) {
	t.Parallel()

	cfg := routerTestConfig(t)
	table := signrouter.RoutingTable()
	for _, geo := range deformationSections {
		for _, color := range []signrouter.SignColor{signrouter.SignColorRed, signrouter.SignColorGreen} {
			entry := table[signrouter.RoutingKey{Corridor: geo.section, Direction: trackmodel.Counterclockwise}]
			permitted := entry.RedMult
			if color == signrouter.SignColorGreen {
				permitted = entry.GreenMult
			}

			sign := signrouter.SignSpec{X: geo.sx, Y: geo.sy, Color: color}
			router := newTestRouter(t, []signrouter.SignSpec{sign}, cfg)

			// Engage the sign, then retire it while the robot sits on the
			// PERMITTED side.
			engagedGap := cfg.ActivationDistM / 4
			passedGap := cfg.PassedDistM + 0.2
			var engagePos, passPos trackmodel.Waypoint
			if geo.axisY {
				engagePos = trackmodel.Waypoint{X: geo.sx - engagedGap, Y: geo.sy}
				passPos = trackmodel.Waypoint{X: geo.sx, Y: geo.sy + float64(permitted)*passedGap}
			} else {
				engagePos = trackmodel.Waypoint{X: geo.sx, Y: geo.sy - engagedGap}
				passPos = trackmodel.Waypoint{X: geo.sx + float64(permitted)*passedGap, Y: geo.sy}
			}
			router.DeformWaypoint(
				trackmodel.Waypoint{X: geo.sx, Y: geo.sy},
				engagePos,
				0.0,
				geo.section,
				nil,
			)
			router.DeformWaypoint(
				trackmodel.Waypoint{X: geo.sx, Y: geo.sy},
				passPos,
				0.0,
				geo.section,
				nil,
			)

			if got := router.WrongSideViolations(); len(got) != 0 {
				t.Errorf(
					"%v/%v: WrongSideViolations() = %v, want empty (permitted-side pass)",
					geo.section,
					color,
					got,
				)
			}
		}
	}
}

func TestWrongSideViolations_WrongSideIsAViolation(t *testing.T) {
	t.Parallel()

	cfg := routerTestConfig(t)
	table := signrouter.RoutingTable()
	for _, geo := range deformationSections {
		for _, color := range []signrouter.SignColor{signrouter.SignColorRed, signrouter.SignColorGreen} {
			entry := table[signrouter.RoutingKey{Corridor: geo.section, Direction: trackmodel.Counterclockwise}]
			permitted := entry.RedMult
			if color == signrouter.SignColorGreen {
				permitted = entry.GreenMult
			}
			forbidden := -permitted

			sign := signrouter.SignSpec{X: geo.sx, Y: geo.sy, Color: color}
			router := newTestRouter(t, []signrouter.SignSpec{sign}, cfg)

			engagedGap := cfg.ActivationDistM / 4
			passedGap := cfg.PassedDistM + 0.2
			var engagePos, passPos trackmodel.Waypoint
			if geo.axisY {
				engagePos = trackmodel.Waypoint{X: geo.sx - engagedGap, Y: geo.sy}
				passPos = trackmodel.Waypoint{X: geo.sx, Y: geo.sy + float64(forbidden)*passedGap}
			} else {
				engagePos = trackmodel.Waypoint{X: geo.sx, Y: geo.sy - engagedGap}
				passPos = trackmodel.Waypoint{X: geo.sx + float64(forbidden)*passedGap, Y: geo.sy}
			}
			router.DeformWaypoint(
				trackmodel.Waypoint{X: geo.sx, Y: geo.sy},
				engagePos,
				0.0,
				geo.section,
				nil,
			)
			router.DeformWaypoint(
				trackmodel.Waypoint{X: geo.sx, Y: geo.sy},
				passPos,
				0.0,
				geo.section,
				nil,
			)

			got := router.WrongSideViolations()
			if _, violated := got[0]; !violated || len(got) != 1 {
				t.Errorf(
					"%v/%v: WrongSideViolations() = %v, want {0} (forbidden-side pass)",
					geo.section,
					color,
					got,
				)
			}
		}
	}
}

// TestResetForNewLap_ClearsWrongSideViolations matches
// TestWrongSidePassDetection.test_reset_for_new_lap_clears_violations.
func TestResetForNewLap_ClearsWrongSideViolations(t *testing.T) {
	t.Parallel()

	cfg := routerTestConfig(t)
	// Red in SOUTH is permitted outward (-Y); pass it on the inner (+Y) side.
	sign := signrouter.SignSpec{X: 1.5, Y: 0.4, Color: signrouter.SignColorRed}
	router := newTestRouter(t, []signrouter.SignSpec{sign}, cfg)

	engagedGap := cfg.ActivationDistM / 4
	passedGap := cfg.PassedDistM + 0.2
	router.DeformWaypoint(
		trackmodel.Waypoint{X: 1.5, Y: 0.4},
		trackmodel.Waypoint{X: 1.5 - engagedGap, Y: 0.4},
		0.0,
		trackmodel.South,
		nil,
	)
	router.DeformWaypoint(
		trackmodel.Waypoint{X: 1.5, Y: 0.4},
		trackmodel.Waypoint{X: 1.5, Y: 0.4 + passedGap},
		0.0,
		trackmodel.South,
		nil,
	)

	if got := router.WrongSideViolations(); len(got) != 1 {
		t.Fatalf("WrongSideViolations() = %v, want {0} before reset", got)
	}
	router.ResetForNewLap()
	if got := router.WrongSideViolations(); len(got) != 0 {
		t.Errorf("WrongSideViolations() = %v, want empty after ResetForNewLap", got)
	}
}

// TestDeformWaypoint_CameraDetectionOverridesGroundTruth matches
// TestCameraDetectionOverridesGroundTruth.test_camera_color_flips_avoidance_side:
// a confident camera detection can override the sign's own metadata color --
// this is the one part of the navigation stack where a live sensor reading
// beats known-good ground truth. Proves the override actually changes which
// side the robot passes on.
func TestDeformWaypoint_CameraDetectionOverridesGroundTruth(t *testing.T) {
	t.Parallel()

	cfg := routerTestConfig(t)
	geo := deformationSections[0] // South
	sign := signrouter.SignSpec{X: geo.sx, Y: geo.sy, Color: signrouter.SignColorRed}
	router := newTestRouter(t, []signrouter.SignSpec{sign}, cfg)

	robotPos := trackmodel.Waypoint{X: geo.sx - 0.3, Y: geo.sy}
	obs := []signrouter.TrafficSignObservation{
		{
			WorldXM:    robotPos.X + 0.3,
			WorldYM:    robotPos.Y,
			Color:      signrouter.SignColorGreen,
			Confidence: 0.9,
		},
	}

	result := router.DeformWaypoint(
		trackmodel.Waypoint{X: geo.sx, Y: geo.sy},
		robotPos,
		0.0,
		trackmodel.South,
		obs,
	)

	expectedIfGreen := signrouter.ApplyDeformation(
		trackmodel.Waypoint{
			X: geo.sx,
			Y: geo.sy,
		},
		sign,
		signrouter.SignColorGreen,
		trackmodel.South,
		trackmodel.Counterclockwise,
		cfg.LateralOffsetM,
		signrouter.PinContext{},
		cfg,
	)
	expectedIfRed := signrouter.ApplyDeformation(
		trackmodel.Waypoint{X: geo.sx, Y: geo.sy}, sign, signrouter.SignColorRed, trackmodel.South,
		trackmodel.Counterclockwise, cfg.LateralOffsetM, signrouter.PinContext{}, cfg,
	)

	if math.Abs(result.Y-expectedIfGreen.Y) > tolerance {
		t.Errorf(
			"DeformWaypoint() with a confident GREEN observation = %+v, want %+v (as if the sign were green)",
			result,
			expectedIfGreen,
		)
	}
	if math.Abs(result.Y-expectedIfRed.Y) < tolerance {
		t.Errorf(
			"DeformWaypoint() = %+v, matches the RED ground truth %+v -- the camera override had no effect",
			result,
			expectedIfRed,
		)
	}
}

// TestNewSignRouter_RejectsInvalidConfig matches the construction-time half
// of TestActivationPassedOrdering: SignRouter must not be buildable with an
// activation_dist/passed_dist ordering that silently disables the whole
// sign-avoidance path.
func TestNewSignRouter_RejectsInvalidConfig(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	cfg.ActivationDistM = 1.30
	cfg.PassedDistM = 1.20

	if _, err := signrouter.NewSignRouter(nil, cfg, trackmodel.Counterclockwise); err == nil {
		t.Error("NewSignRouter() error = nil, want an error for activation_dist >= passed_dist")
	}
}

// TestPreferCommitted_HoldsTheCommittedSignOverANearerNewcomer pins
// preferCommitted's documented property (see router.go): where two signs
// are both in play, the winner of a pure nearest-wins race can flip while
// the chassis is already committed to one, jumping the commanded lateral
// line with no runway left to track it. A sign the robot is already
// committed to holds its claim as long as it stays an applicable candidate,
// even once a nearer sign becomes available. No Python oracle test exists
// for this (see router.py's _prefer_committed, untested there too); this is
// hand-derived from the documented invariant, matching this port's
// established precedent for such cases (see geometry_internal_test.go).
func TestPreferCommitted_HoldsTheCommittedSignOverANearerNewcomer(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	cfg.SettleTicks = 0
	cfg.CommitHysteresis = true
	// Two RED signs in the same SOUTH corridor, spaced far apart in Y so
	// their deformed lateral targets land clearly apart even after
	// clamping (A's is pinned to the outer-wall clamp floor; B's is not).
	signA := signrouter.SignSpec{X: 1.6, Y: 0.30, Color: signrouter.SignColorRed}
	signB := signrouter.SignSpec{X: 1.5, Y: 0.70, Color: signrouter.SignColorRed}
	router := newTestRouter(t, []signrouter.SignSpec{signA, signB}, cfg)

	waypoint := trackmodel.Waypoint{X: 1.55, Y: 0.5}
	// Tick 1: the robot sits right next to B, far from A -- commits to B.
	tick1 := router.DeformWaypoint(
		waypoint,
		trackmodel.Waypoint{X: 1.5, Y: 0.75},
		0.0,
		trackmodel.South,
		nil,
	)
	const bDrivenThreshold = 0.35 // strictly above A's clamp floor (~0.22), below B's unclamped target (~0.46)
	if tick1.Y < bDrivenThreshold {
		t.Fatalf(
			"tick 1 Y = %v, want > %v (committed to the nearer sign B)",
			tick1.Y,
			bDrivenThreshold,
		)
	}

	// Tick 2: the robot has moved next to A, which is now the NEARER sign by
	// raw distance -- but the commitment to B must hold, since B is still
	// an applicable candidate (within activation distance, same corridor).
	// Without hysteresis this tick would flip to A and read near the clamp
	// floor instead.
	tick2 := router.DeformWaypoint(
		waypoint,
		trackmodel.Waypoint{X: 1.6, Y: 0.25},
		0.0,
		trackmodel.South,
		nil,
	)
	if tick2.Y < bDrivenThreshold {
		t.Errorf(
			"tick 2 Y = %v, want > %v (commit hysteresis should hold on B even though A is now nearer)",
			tick2.Y,
			bDrivenThreshold,
		)
	}
}

// TestPreferCommitted_DisabledFollowsPureNearestEachTick is the control for
// the test above: with CommitHysteresis off, the same approach flips to
// whichever sign is nearest on each tick.
func TestPreferCommitted_DisabledFollowsPureNearestEachTick(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	cfg.SettleTicks = 0
	cfg.CommitHysteresis = false
	signA := signrouter.SignSpec{X: 1.6, Y: 0.30, Color: signrouter.SignColorRed}
	signB := signrouter.SignSpec{X: 1.5, Y: 0.70, Color: signrouter.SignColorRed}
	router := newTestRouter(t, []signrouter.SignSpec{signA, signB}, cfg)

	waypoint := trackmodel.Waypoint{X: 1.55, Y: 0.5}
	const bDrivenThreshold = 0.35
	router.DeformWaypoint(
		waypoint,
		trackmodel.Waypoint{X: 1.5, Y: 0.75},
		0.0,
		trackmodel.South,
		nil,
	) // near B

	tick2 := router.DeformWaypoint(
		waypoint,
		trackmodel.Waypoint{X: 1.6, Y: 0.25},
		0.0,
		trackmodel.South,
		nil,
	) // near A
	if tick2.Y >= bDrivenThreshold {
		t.Errorf(
			"tick 2 Y = %v, want < %v (without hysteresis, nearest-wins should flip to A)",
			tick2.Y,
			bDrivenThreshold,
		)
	}
}
