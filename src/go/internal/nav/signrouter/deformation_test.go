package signrouter_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/racetracker"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// sectionGeometry is one representative in-corridor sign position per
// section, matching test_sign_router.py's _SECTION_GEOMETRY: axis is which
// coordinate the routing table deforms, redMult is ROUTING_TABLE's
// red_mult for that section (the same for CW and CCW).
type sectionGeometry struct {
	section trackmodel.Section
	sx, sy  float64
	axisY   bool
	redMult int
	lowSide bool
}

// pinDepthScenario bundles the geometry shared by every corner-buffer /
// heading-drift case below: a SOUTH-reading lateral, and a robot/sign/
// waypoint depth triple straddling the buffered corner window.
type pinDepthScenario struct {
	cfg                                            signrouter.Config
	lateralY, robotDepth, signDepth, waypointDepth float64
}

const tolerance = 1e-6

var deformationSections = []sectionGeometry{
	{trackmodel.South, 1.5, 0.4, true, -1, true},
	{trackmodel.North, 1.5, 2.6, true, +1, false},
	{trackmodel.East, 2.6, 1.5, false, +1, false},
	{trackmodel.West, 0.4, 1.5, false, -1, true},
}

// expectedLateral independently reproduces ClampLateral's formula (rather
// than calling it) so TestApplyDeformation_OffsetDirectionPerSectionColorDirection
// isn't tautological, matching test_sign_router.py's own
// “_expected_lateral“ helper.
func expectedLateral(value float64, lowSide bool, cfg signrouter.Config) float64 {
	wallClearance := cfg.ChassisHalfDiagonalM + cfg.WallClearanceMarginM
	if lowSide {
		v := math.Min(value, cfg.TrackCornerMinM-wallClearance)
		return math.Max(v, cfg.TrackMinCoordM+wallClearance)
	}
	v := math.Max(value, cfg.TrackCornerMaxM+wallClearance)
	return math.Min(v, cfg.TrackMaxCoordM-wallClearance)
}

// newPinDepthScenario builds a pinDepthScenario whose robot/sign/waypoint
// depths sit robotOffset/signOffset/waypointOffset past the buffered corner
// window's far edge (DefaultConfig's TrackCornerMaxM + DeformDepthBufferM).
func newPinDepthScenario(robotOffset, signOffset, waypointOffset float64) pinDepthScenario {
	cfg := signrouter.DefaultConfig()
	depthMax := cfg.TrackCornerMaxM + cfg.DeformDepthBufferM
	return pinDepthScenario{
		cfg:           cfg,
		lateralY:      cfg.TrackCornerMinM - 0.05,
		robotDepth:    depthMax + robotOffset,
		signDepth:     depthMax + signOffset,
		waypointDepth: depthMax + waypointOffset,
	}
}

// apply runs ApplyDeformation for this scenario's SOUTH/CLOCKWISE/RED sign
// and returns the resulting depth coordinate (X, since SOUTH deforms Y).
func (s pinDepthScenario) apply(pin signrouter.PinContext) float64 {
	sign := signrouter.SignSpec{X: s.signDepth, Y: s.lateralY, Color: signrouter.SignColorRed}
	result := signrouter.ApplyDeformation(
		trackmodel.Waypoint{X: s.waypointDepth, Y: s.lateralY}, sign, signrouter.SignColorRed,
		trackmodel.South, trackmodel.Clockwise, s.cfg.LateralOffsetM, pin, s.cfg,
	)
	return result.X
}

// TestApplyDeformation_OffsetDirectionPerSectionColorDirection matches
// TestDeformationDirections.test_offset_side: 16 cases (4 sections x
// {red, green} x {CCW, CW}). geo.redMult is the section's OUTWARD
// multiplier (CCW), so colorSign says whether this (direction, color) pair
// should deform outward (+1) or inward (-1). The rule is TRAVEL-RELATIVE --
// red passes on the vehicle's right -- and the vehicle's right is the
// outer wall driving COUNTERCLOCKWISE but the inner square driving
// CLOCKWISE, so the two directions take OPPOSITE signs. They were
// identical here until the 2026-09-03 fix, which is what let the absolute
// misreading survive.
func TestApplyDeformation_OffsetDirectionPerSectionColorDirection(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	cases := []struct {
		direction trackmodel.Direction
		color     signrouter.SignColor
		colorSign int
	}{
		// CCW: the chassis's right hand points OUTWARD.
		{trackmodel.Counterclockwise, signrouter.SignColorRed, +1},
		{trackmodel.Counterclockwise, signrouter.SignColorGreen, -1},
		// CW: it points INWARD, so the same rule inverts in track terms.
		{trackmodel.Clockwise, signrouter.SignColorRed, -1},
		{trackmodel.Clockwise, signrouter.SignColorGreen, +1},
	}

	for _, geo := range deformationSections {
		for _, c := range cases {
			direction := c.direction
			sign := signrouter.SignSpec{X: geo.sx, Y: geo.sy, Color: c.color}
			result := signrouter.ApplyDeformation(
				trackmodel.Waypoint{
					X: geo.sx,
					Y: geo.sy,
				},
				sign,
				c.color,
				geo.section,
				direction,
				cfg.LateralOffsetM,
				signrouter.PinContext{},
				cfg,
			)
			offset := float64(geo.redMult*c.colorSign) * cfg.LateralOffsetM

			if geo.axisY {
				want := expectedLateral(geo.sy+offset, geo.lowSide, cfg)
				if math.Abs(result.Y-want) > tolerance {
					t.Errorf(
						"%v/%v/%v: Y = %v, want %v",
						geo.section,
						direction,
						c.color,
						result.Y,
						want,
					)
				}
				if math.Abs(result.X-geo.sx) > tolerance {
					t.Errorf(
						"%v/%v/%v: X = %v, want unchanged %v",
						geo.section,
						direction,
						c.color,
						result.X,
						geo.sx,
					)
				}
			} else {
				want := expectedLateral(geo.sx+offset, geo.lowSide, cfg)
				if math.Abs(result.X-want) > tolerance {
					t.Errorf("%v/%v/%v: X = %v, want %v", geo.section, direction, c.color, result.X, want)
				}
				if math.Abs(result.Y-geo.sy) > tolerance {
					t.Errorf(
						"%v/%v/%v: Y = %v, want unchanged %v",
						geo.section,
						direction,
						c.color,
						result.Y,
						geo.sy,
					)
				}
			}
		}
	}
}

// TestApplyDeformation_SignKeptOnCorrectSide matches
// TestPassSideRule.test_sign_kept_on_correct_side: red is passed on the
// vehicle's RIGHT, green on its LEFT (rules 2026 9.19), for every
// corridor, in the direction the round is actually driven. Asserted
// against the chassis's own heading (racetracker.TravelNormalFor, the
// production table) rather than a track-frame outward vector, because the
// two agree counterclockwise and are OPPOSITE clockwise -- which is
// exactly the bug this replaces (was
// TestApplyDeformation_PassSideIsAbsoluteAcrossDirections, asserting the
// absolute form for BOTH directions, so it passed while every clockwise
// round routed backwards).
func TestApplyDeformation_SignKeptOnCorrectSide(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	directions := []trackmodel.Direction{trackmodel.Clockwise, trackmodel.Counterclockwise}

	for _, geo := range deformationSections {
		for _, direction := range directions {
			for _, color := range []signrouter.SignColor{signrouter.SignColorRed, signrouter.SignColorGreen} {
				sign := signrouter.SignSpec{X: geo.sx, Y: geo.sy, Color: color}
				result := signrouter.ApplyDeformation(
					trackmodel.Waypoint{X: geo.sx, Y: geo.sy}, sign, color, geo.section, direction,
					cfg.LateralOffsetM, signrouter.PinContext{}, cfg,
				)
				// The chassis's own right-hand direction for the heading it
				// drives here: rotating the travel vector by -90 degrees
				// gives (hy, -hx).
				heading, ok := racetracker.TravelNormalFor(geo.section, direction)
				if !ok {
					t.Fatalf("TravelNormalFor(%v, %v) ok = false", geo.section, direction)
				}
				rightX, rightY := heading.NY, -heading.NX
				rightComponent := rightX*(result.X-geo.sx) + rightY*(result.Y-geo.sy)
				if color == signrouter.SignColorRed && rightComponent <= 0 {
					t.Errorf(
						"%v/%v/red: right component = %v, want > 0 (passed on the vehicle's right)",
						geo.section,
						direction,
						rightComponent,
					)
				}
				if color == signrouter.SignColorGreen && rightComponent >= 0 {
					t.Errorf(
						"%v/%v/green: right component = %v, want < 0 (passed on the vehicle's left)",
						geo.section,
						direction,
						rightComponent,
					)
				}
			}
		}
	}
}

// TestApplyDeformation_ClampsAtInnerSquareEdge matches
// TestDeformationClamping.test_sign_at_inner_edge_does_not_enter_inner_square:
// unclamped, a SOUTH sign at the inner-square boundary would deform past it;
// ClampLateral must hold the result below TrackCornerMinM.
func TestApplyDeformation_ClampsAtInnerSquareEdge(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	sign := signrouter.SignSpec{X: 1.5, Y: cfg.TrackCornerMinM, Color: signrouter.SignColorRed}
	result := signrouter.ApplyDeformation(
		trackmodel.Waypoint{X: 1.5, Y: cfg.TrackCornerMinM},
		sign,
		signrouter.SignColorRed,
		trackmodel.South,
		trackmodel.Counterclockwise,
		cfg.LateralOffsetM,
		signrouter.PinContext{},
		cfg,
	)
	if math.Abs(result.X-1.5) > tolerance {
		t.Errorf("X = %v, want unchanged 1.5", result.X)
	}
	if result.Y >= cfg.TrackCornerMinM {
		t.Errorf("Y = %v, want it held below TrackCornerMinM = %v", result.Y, cfg.TrackCornerMinM)
	}
}

// TestApplyDeformation_ClampsAtOuterWallEdge matches
// TestDeformationClamping.test_sign_at_outer_edge_does_not_cross_wall.
func TestApplyDeformation_ClampsAtOuterWallEdge(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	sign := signrouter.SignSpec{X: 1.5, Y: cfg.TrackMinCoordM, Color: signrouter.SignColorGreen}
	result := signrouter.ApplyDeformation(
		trackmodel.Waypoint{X: 1.5, Y: cfg.TrackMinCoordM},
		sign,
		signrouter.SignColorGreen,
		trackmodel.South,
		trackmodel.Counterclockwise,
		cfg.LateralOffsetM,
		signrouter.PinContext{},
		cfg,
	)
	if math.Abs(result.X-1.5) > tolerance {
		t.Errorf("X = %v, want unchanged 1.5", result.X)
	}
	if result.Y < cfg.TrackMinCoordM {
		t.Errorf("Y = %v, want it held at/above TrackMinCoordM = %v", result.Y, cfg.TrackMinCoordM)
	}
}

// TestApplyDeformation_ClampsAtInnerSquareEdgeEastCorridor matches
// TestDeformationClamping.test_sign_at_inner_edge_east_corridor: EAST
// deforms x, and EAST/CCW's red_mult=+1 with the sign already sitting at
// the inner-square boundary would deform it inward without the clamp.
func TestApplyDeformation_ClampsAtInnerSquareEdgeEastCorridor(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	sign := signrouter.SignSpec{X: cfg.TrackCornerMaxM, Y: 1.5, Color: signrouter.SignColorRed}
	result := signrouter.ApplyDeformation(
		trackmodel.Waypoint{X: cfg.TrackCornerMaxM, Y: 1.5},
		sign,
		signrouter.SignColorRed,
		trackmodel.East,
		trackmodel.Counterclockwise,
		cfg.LateralOffsetM,
		signrouter.PinContext{},
		cfg,
	)
	if math.Abs(result.Y-1.5) > tolerance {
		t.Errorf("Y = %v, want unchanged 1.5", result.Y)
	}
	if result.X <= cfg.TrackCornerMaxM {
		t.Errorf("X = %v, want it held above TrackCornerMaxM = %v", result.X, cfg.TrackCornerMaxM)
	}
}

// TestPinDepth_CornerGuardBlocksOnceRobotIsPastTheBufferedCorner matches
// TestDepthPinCornerGuard.test_pin_does_not_fire_once_the_robot_is_past_the_corner_buffer:
// the pin must not fire once the ROBOT (not the receding waypoint) has
// curved out of the straight-corridor assumption it depends on. Regression
// guard for an 11-collision wall-hit regression.
func TestPinDepth_CornerGuardBlocksOnceRobotIsPastTheBufferedCorner(t *testing.T) {
	t.Parallel()

	scenario := newPinDepthScenario(+0.05, -0.05, -0.15) // robot past buffer
	robotPos := trackmodel.Waypoint{X: scenario.robotDepth, Y: scenario.lateralY}
	got := scenario.apply(signrouter.PinContext{RobotPos: &robotPos})

	if math.Abs(got-scenario.waypointDepth) > tolerance {
		t.Errorf(
			"result depth = %v, want unchanged waypoint depth %v (pin should not fire)",
			got,
			scenario.waypointDepth,
		)
	}
}

// TestPinDepth_StillFiresWhenRobotIsSquarelyInCorridor matches
// TestDepthPinCornerGuard.test_pin_still_fires_when_the_robot_is_squarely_in_corridor:
// regression guard for the fix itself -- the guard must not also kill
// legitimate pinning when the robot genuinely is square to the corridor.
func TestPinDepth_StillFiresWhenRobotIsSquarelyInCorridor(t *testing.T) {
	t.Parallel()

	scenario := newPinDepthScenario(-0.20, -0.10, -0.05) // robot inside the buffer
	robotPos := trackmodel.Waypoint{X: scenario.robotDepth, Y: scenario.lateralY}
	got := scenario.apply(signrouter.PinContext{RobotPos: &robotPos})

	if math.Abs(got-scenario.signDepth) > tolerance {
		t.Errorf("result depth = %v, want pinned to sign depth %v", got, scenario.signDepth)
	}
}

// TestPinDepth_CornerGuardOffRestoresThePin matches
// TestDepthPinCornerGuard.test_guard_off_restores_the_pin_that_cost_11_wall_collisions:
// with PinCornerGuard disabled, the same geometry that blocked the pin above
// must let it fire again -- otherwise the sweep that measured the guard's
// effect would be comparing a knob that does not move the geometry.
func TestPinDepth_CornerGuardOffRestoresThePin(t *testing.T) {
	t.Parallel()

	scenario := newPinDepthScenario(+0.05, -0.05, -0.15)
	scenario.cfg.PinCornerGuard = false
	robotPos := trackmodel.Waypoint{X: scenario.robotDepth, Y: scenario.lateralY}
	got := scenario.apply(signrouter.PinContext{RobotPos: &robotPos})

	if math.Abs(got-scenario.signDepth) > tolerance {
		t.Errorf(
			"result depth = %v, want pinned to sign depth %v with the guard off",
			got,
			scenario.signDepth,
		)
	}
}

// TestPinDepth_HeadingGuardReleasesPastTheThreshold matches
// TestDepthPinHeadingGuard.test_pin_releases_once_yaw_has_drifted_past_the_threshold:
// traced on go_obstacles_0049 -- the pin held a commanded point frozen for
// 46 ticks while the robot's yaw rotated 67deg mid-corner, because
// PinCornerGuard's position-only check never tripped.
func TestPinDepth_HeadingGuardReleasesPastTheThreshold(t *testing.T) {
	t.Parallel()

	scenario := newPinDepthScenario(-0.20, -0.10, -0.05)
	robotPos := trackmodel.Waypoint{X: scenario.robotDepth, Y: scenario.lateralY}
	drift := 40.0 * math.Pi / 180
	got := scenario.apply(signrouter.PinContext{RobotPos: &robotPos, YawDriftRad: &drift})

	if math.Abs(got-scenario.waypointDepth) > tolerance {
		t.Errorf(
			"result depth = %v, want unchanged waypoint depth %v (40deg drift exceeds the 35deg guard)",
			got,
			scenario.waypointDepth,
		)
	}
}

// TestPinDepth_StillFiresUnderTheYawDriftThreshold matches
// TestDepthPinHeadingGuard.test_pin_still_fires_under_the_yaw_drift_threshold.
func TestPinDepth_StillFiresUnderTheYawDriftThreshold(t *testing.T) {
	t.Parallel()

	scenario := newPinDepthScenario(-0.20, -0.10, -0.05)
	robotPos := trackmodel.Waypoint{X: scenario.robotDepth, Y: scenario.lateralY}
	drift := 10.0 * math.Pi / 180
	got := scenario.apply(signrouter.PinContext{RobotPos: &robotPos, YawDriftRad: &drift})

	if math.Abs(got-scenario.signDepth) > tolerance {
		t.Errorf(
			"result depth = %v, want pinned to sign depth %v (10deg drift is under the 35deg guard)",
			got,
			scenario.signDepth,
		)
	}
}

// TestPinDepth_HeadingGuardOffIgnoresDrift matches
// TestDepthPinHeadingGuard.test_guard_off_ignores_yaw_drift: a large
// yaw_drift must not suppress the pin unless PinHeadingGuard is on.
func TestPinDepth_HeadingGuardOffIgnoresDrift(t *testing.T) {
	t.Parallel()

	scenario := newPinDepthScenario(-0.20, -0.10, -0.05)
	scenario.cfg.PinHeadingGuard = false
	robotPos := trackmodel.Waypoint{X: scenario.robotDepth, Y: scenario.lateralY}
	drift := 90.0 * math.Pi / 180
	got := scenario.apply(signrouter.PinContext{RobotPos: &robotPos, YawDriftRad: &drift})

	if math.Abs(got-scenario.signDepth) > tolerance {
		t.Errorf(
			"result depth = %v, want pinned to sign depth %v with the heading guard off",
			got,
			scenario.signDepth,
		)
	}
}

// TestMatchDetectionToSign_LowConfidenceRejected matches
// TestMatchDetectionToSign.test_low_confidence_observation_rejected.
func TestMatchDetectionToSign_LowConfidenceRejected(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	obs := []signrouter.TrafficSignObservation{
		{WorldXM: 0.5, WorldYM: 0.0, Color: signrouter.SignColorRed, Confidence: 0.1},
	}
	_, ok := signrouter.MatchDetectionToSign(obs, trackmodel.Waypoint{X: 0.5, Y: 0.0}, cfg)
	if ok {
		t.Error("MatchDetectionToSign() ok = true for a below-threshold confidence, want false")
	}
}

// TestMatchDetectionToSign_FarMatchRejected matches
// TestMatchDetectionToSign.test_far_match_rejected.
func TestMatchDetectionToSign_FarMatchRejected(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	obs := []signrouter.TrafficSignObservation{
		{WorldXM: 2.0, WorldYM: 0.0, Color: signrouter.SignColorRed, Confidence: 0.9},
	}
	_, ok := signrouter.MatchDetectionToSign(obs, trackmodel.Waypoint{X: 0.0, Y: 0.0}, cfg)
	if ok {
		t.Error(
			"MatchDetectionToSign() ok = true for an observation 2m from the expected position, want false",
		)
	}
}

// TestMatchDetectionToSign_NearestCandidateWinsRegardlessOfOrder matches
// TestMatchDetectionToSign.test_nearest_candidate_wins_regardless_of_order.
func TestMatchDetectionToSign_NearestCandidateWinsRegardlessOfOrder(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	expected := trackmodel.Waypoint{X: 0.5, Y: 0.0}
	near := signrouter.TrafficSignObservation{
		WorldXM:    0.5,
		WorldYM:    0.0,
		Color:      signrouter.SignColorGreen,
		Confidence: 0.9,
	}
	far := signrouter.TrafficSignObservation{
		WorldXM:    0.65,
		WorldYM:    0.0,
		Color:      signrouter.SignColorRed,
		Confidence: 0.9,
	}

	for _, order := range [][]signrouter.TrafficSignObservation{{near, far}, {far, near}} {
		color, ok := signrouter.MatchDetectionToSign(order, expected, cfg)
		if !ok {
			t.Fatal("MatchDetectionToSign() ok = false, want true")
		}
		if color != signrouter.SignColorGreen {
			t.Errorf(
				"MatchDetectionToSign() = %v, want SignColorGreen (the nearer observation)",
				color,
			)
		}
	}
}
