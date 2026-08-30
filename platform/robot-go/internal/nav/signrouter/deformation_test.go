package signrouter_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
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
// {red, green} x {CCW, CW}). Red always moves the deformed waypoint OUTWARD
// (away from the inner square), green always INWARD -- identically for CW
// and CCW, since this is an absolute property of the track, not the travel
// direction.
func TestApplyDeformation_OffsetDirectionPerSectionColorDirection(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	directions := []trackmodel.Direction{trackmodel.Counterclockwise, trackmodel.Clockwise}
	colors := []struct {
		color     signrouter.SignColor
		colorSign int
	}{
		{signrouter.SignColorRed, +1},
		{signrouter.SignColorGreen, -1},
	}

	for _, geo := range deformationSections {
		for _, direction := range directions {
			for _, c := range colors {
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
}

// TestApplyDeformation_PassSideIsAbsoluteAcrossDirections matches
// TestPassSideRule.test_sign_kept_on_correct_side: red is avoided outward,
// green inward, for every corridor, in BOTH directions -- pinned as an
// ABSOLUTE track-relative invariant (not "red on the robot's right", a
// travel-relative rule that would flip outward/inward between CW and CCW).
func TestApplyDeformation_PassSideIsAbsoluteAcrossDirections(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	outwardDir := map[trackmodel.Section][2]float64{
		trackmodel.South: {0, -1},
		trackmodel.North: {0, 1},
		trackmodel.East:  {1, 0},
		trackmodel.West:  {-1, 0},
	}
	directions := []trackmodel.Direction{trackmodel.Clockwise, trackmodel.Counterclockwise}

	for _, geo := range deformationSections {
		for _, direction := range directions {
			for _, color := range []signrouter.SignColor{signrouter.SignColorRed, signrouter.SignColorGreen} {
				sign := signrouter.SignSpec{X: geo.sx, Y: geo.sy, Color: color}
				result := signrouter.ApplyDeformation(
					trackmodel.Waypoint{X: geo.sx, Y: geo.sy}, sign, color, geo.section, direction,
					cfg.LateralOffsetM, signrouter.PinContext{}, cfg,
				)
				dir := outwardDir[geo.section]
				outwardComponent := dir[0]*(result.X-geo.sx) + dir[1]*(result.Y-geo.sy)
				if color == signrouter.SignColorRed && outwardComponent <= 0 {
					t.Errorf(
						"%v/%v/red: outward component = %v, want > 0",
						geo.section,
						direction,
						outwardComponent,
					)
				}
				if color == signrouter.SignColorGreen && outwardComponent >= 0 {
					t.Errorf(
						"%v/%v/green: outward component = %v, want < 0",
						geo.section,
						direction,
						outwardComponent,
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
