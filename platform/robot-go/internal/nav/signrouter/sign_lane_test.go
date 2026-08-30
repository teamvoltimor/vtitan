package signrouter_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

const laneSouthBaseY = 0.5

// laneTestParams matches test_sign_lane.py's module-level _PARAMS: a fixed
// (not chassis-derived) offset since this file's tests pin the LANE
// GEOMETRY's own transitions, not the offset's derivation (config_test.go
// covers that).
var laneTestParams = signrouter.SignLaneParams{LateralOffsetM: 0.28, RampM: 0.70, HoldM: 0.25}

// southStraight builds evenly spaced centerline waypoints spanning the
// SOUTH corridor's straight, matching test_sign_lane.py's _south_straight.
func southStraight(t *testing.T, count int) []trackmodel.Waypoint {
	t.Helper()
	cfg := signrouter.DefaultConfig()
	lo, hi := cfg.TrackCornerMinM, cfg.TrackCornerMaxM
	out := make([]trackmodel.Waypoint, count)
	for i := range count {
		out[i] = trackmodel.Waypoint{X: lo + (hi-lo)*float64(i)/float64(count-1), Y: laneSouthBaseY}
	}
	return out
}

// lateralAt returns the Y of the waypoint nearest depth along X, matching
// test_sign_lane.py's _lateral_at.
func lateralAt(waypoints []trackmodel.Waypoint, depth float64) float64 {
	best := waypoints[0]
	bestDist := math.Abs(best.X - depth)
	for _, wp := range waypoints[1:] {
		if d := math.Abs(wp.X - depth); d < bestDist {
			best, bestDist = wp, d
		}
	}
	return best.Y
}

// TestApplySignLanes_EmptySignListReturnsInputPath matches
// TestNoSigns.test_empty_sign_list_returns_input_path.
func TestApplySignLanes_EmptySignListReturnsInputPath(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	path := southStraight(t, 21)
	got := signrouter.ApplySignLanes(path, nil, laneTestParams, cfg)

	if len(got) != len(path) {
		t.Fatalf("len(ApplySignLanes()) = %v, want %v", len(got), len(path))
	}
	for i := range path {
		if got[i] != path[i] {
			t.Errorf("waypoint %d = %+v, want unchanged %+v", i, got[i], path[i])
		}
	}
}

// TestApplySignLanes_EmptyPathIsHandled matches
// TestNoSigns.test_empty_path_is_handled.
func TestApplySignLanes_EmptyPathIsHandled(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	signs := []signrouter.LaneSpec{
		{
			Spec:     signrouter.SignSpec{X: 1.5, Y: 0.5, Color: signrouter.SignColorRed},
			Corridor: trackmodel.South,
		},
	}
	got := signrouter.ApplySignLanes(nil, signs, laneTestParams, cfg)
	if len(got) != 0 {
		t.Errorf("ApplySignLanes(nil path) = %v, want empty", got)
	}
}

// TestApplySignLanes_LaneOffsetsToTheRuledSide matches
// TestPassSide.test_lane_offsets_to_the_ruled_side: the lane must land on
// the same side the pass-side rule already demands.
func TestApplySignLanes_LaneOffsetsToTheRuledSide(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	cases := []struct {
		color        signrouter.SignColor
		expectedSign float64
	}{
		{signrouter.SignColorRed, -1},
		{signrouter.SignColorGreen, +1},
	}
	for _, tc := range cases {
		sign := signrouter.SignSpec{X: 1.5, Y: 0.5, Color: tc.color}
		laned := signrouter.ApplySignLanes(
			southStraight(
				t,
				21,
			),
			[]signrouter.LaneSpec{{Spec: sign, Corridor: trackmodel.South}},
			laneTestParams,
			cfg,
		)
		got := lateralAt(laned, 1.5)
		want := sign.Y + tc.expectedSign*laneTestParams.LateralOffsetM
		if math.Abs(got-want) > 0.02 {
			t.Errorf("color=%v: lateral at depth 1.5 = %v, want approx %v", tc.color, got, want)
		}
	}
}

// TestApplySignLanes_LaneMeetsTheCornerArcOnTheCentreline matches
// TestPassSide.test_lane_meets_the_corner_arc_on_the_centerline: both ends
// of the straight must still be centered, however the ramp falls. The sign
// sits mid-straight, so the nominal ramp start (1.5 - 0.25 - 0.70 = 0.55)
// lands well outside [CORNER_MIN, CORNER_MAX] -- this only passes if the
// ramp COMPRESSES rather than truncates.
func TestApplySignLanes_LaneMeetsTheCornerArcOnTheCentreline(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	sign := signrouter.SignSpec{X: 1.5, Y: 0.5, Color: signrouter.SignColorRed}
	laned := signrouter.ApplySignLanes(
		southStraight(
			t,
			21,
		),
		[]signrouter.LaneSpec{{Spec: sign, Corridor: trackmodel.South}},
		laneTestParams,
		cfg,
	)
	if got := lateralAt(laned, cfg.TrackCornerMinM); math.Abs(got-laneSouthBaseY) > tolerance {
		t.Errorf("lateral at TrackCornerMinM = %v, want %v", got, laneSouthBaseY)
	}
	if got := lateralAt(laned, cfg.TrackCornerMaxM); math.Abs(got-laneSouthBaseY) > tolerance {
		t.Errorf("lateral at TrackCornerMaxM = %v, want %v", got, laneSouthBaseY)
	}
}

// TestApplySignLanes_TransitionIsGradualNotAStep matches
// TestPassSide.test_transition_is_gradual_not_a_step: the whole point of
// the lane is that the offset arrives over meters, not one waypoint -- a
// carrot-level deformation jumps to full offset the tick a sign enters
// activation range, and the lane exists to close exactly that shortfall.
func TestApplySignLanes_TransitionIsGradualNotAStep(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	sign := signrouter.SignSpec{X: 1.5, Y: 0.5, Color: signrouter.SignColorRed}
	laned := signrouter.ApplySignLanes(
		southStraight(
			t,
			21,
		),
		[]signrouter.LaneSpec{{Spec: sign, Corridor: trackmodel.South}},
		laneTestParams,
		cfg,
	)
	maxStep := 0.0
	for i := 1; i < len(laned); i++ {
		if step := math.Abs(laned[i].Y - laned[i-1].Y); step > maxStep {
			maxStep = step
		}
	}
	if maxStep >= laneTestParams.LateralOffsetM/2 {
		t.Errorf(
			"max single-waypoint step = %v, want < %v (offset/2)",
			maxStep,
			laneTestParams.LateralOffsetM/2,
		)
	}
}

// TestApplySignLanes_TransformIsOneToOneAndOrdered matches
// TestPathInvariants.test_transform_is_one_to_one_and_ordered: properties
// CoreNavigator's waypoint-index bookkeeping depends on.
func TestApplySignLanes_TransformIsOneToOneAndOrdered(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	path := southStraight(t, 21)
	sign := signrouter.SignSpec{X: 1.5, Y: 0.5, Color: signrouter.SignColorRed}
	laned := signrouter.ApplySignLanes(
		path,
		[]signrouter.LaneSpec{{Spec: sign, Corridor: trackmodel.South}},
		laneTestParams,
		cfg,
	)

	if len(laned) != len(path) {
		t.Fatalf("len(laned) = %v, want %v", len(laned), len(path))
	}
	for i := range path {
		if laned[i].X != path[i].X {
			t.Errorf(
				"waypoint %d: X = %v, want unchanged %v (depth orders the path)",
				i,
				laned[i].X,
				path[i].X,
			)
		}
	}
}

// TestApplySignLanes_CornerArcWaypointsAreUntouched matches
// TestPathInvariants.test_corner_arc_waypoints_are_untouched: arc points
// sit outside [CORNER_MIN, CORNER_MAX] in depth and must not move.
func TestApplySignLanes_CornerArcWaypointsAreUntouched(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	arcStart := trackmodel.Waypoint{X: 0.8, Y: 0.42}
	arcEnd := trackmodel.Waypoint{X: 2.2, Y: 0.42}
	path := append([]trackmodel.Waypoint{arcStart}, southStraight(t, 21)...)
	path = append(path, arcEnd)

	sign := signrouter.SignSpec{X: 1.1, Y: 0.5, Color: signrouter.SignColorRed}
	laned := signrouter.ApplySignLanes(
		path,
		[]signrouter.LaneSpec{{Spec: sign, Corridor: trackmodel.South}},
		laneTestParams,
		cfg,
	)

	if laned[0] != arcStart {
		t.Errorf("laned[0] = %+v, want unchanged arc point %+v", laned[0], arcStart)
	}
	if laned[len(laned)-1] != arcEnd {
		t.Errorf("laned[last] = %+v, want unchanged arc point %+v", laned[len(laned)-1], arcEnd)
	}
}

// TestApplySignLanes_LaneStaysClearOfTheInnerSquare matches
// TestPathInvariants.test_lane_stays_clear_of_the_inner_square: a green
// sign hard against the inner square must not command a lane inside it.
func TestApplySignLanes_LaneStaysClearOfTheInnerSquare(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	sign := signrouter.SignSpec{
		X:     1.5,
		Y:     cfg.TrackCornerMinM - 0.05,
		Color: signrouter.SignColorGreen,
	}
	laned := signrouter.ApplySignLanes(
		southStraight(
			t,
			21,
		),
		[]signrouter.LaneSpec{{Spec: sign, Corridor: trackmodel.South}},
		laneTestParams,
		cfg,
	)
	maxY := math.Inf(-1)
	for _, wp := range laned {
		maxY = math.Max(maxY, wp.Y)
	}
	if maxY >= cfg.TrackCornerMinM {
		t.Errorf(
			"max Y across the lane = %v, want < TrackCornerMinM = %v",
			maxY,
			cfg.TrackCornerMinM,
		)
	}
}

// signLaneCornerEntryBoundary matches TestCornerEntry._BOUNDARY: a boundary
// sign, the overwhelmingly common case (94% of real signs sit at a section
// boundary depth per the Python docstring's corpus fact).
func signLaneCornerEntryBoundary(cfg signrouter.Config) signrouter.SignSpec {
	return signrouter.SignSpec{X: cfg.TrackCornerMinM, Y: 0.5, Color: signrouter.SignColorRed}
}

// withArc matches TestCornerEntry._with_arc: a straight preceded by arc
// points that have begun turning off-center.
func withArc(t *testing.T) []trackmodel.Waypoint {
	t.Helper()
	straight := southStraight(t, 21)
	out := make([]trackmodel.Waypoint, 0, 3+len(straight))
	out = append(
		out,
		trackmodel.Waypoint{X: 0.70, Y: 0.86},
		trackmodel.Waypoint{X: 0.80, Y: 0.72},
		trackmodel.Waypoint{X: 0.90, Y: 0.60},
	)
	return append(out, straight...)
}

func maxStepIn(waypoints []trackmodel.Waypoint) float64 {
	maxStep := 0.0
	for i := 1; i < len(waypoints); i++ {
		if step := math.Abs(waypoints[i].Y - waypoints[i-1].Y); step > maxStep {
			maxStep = step
		}
	}
	return maxStep
}

// TestApplySignLanes_WithoutRunwayABoundarySignStepsOffTheArc matches
// TestCornerEntry.test_without_runway_a_boundary_sign_steps_off_the_arc:
// the defect the borrowed-runway feature exists to fix, pinned so it stays
// fixed. Confined to the straight, the lane has nowhere to ramp on the near
// side of a boundary sign, so its first straight waypoint is already at
// full offset while the arc point immediately before it has not moved.
func TestApplySignLanes_WithoutRunwayABoundarySignStepsOffTheArc(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	path := withArc(t)
	sign := signLaneCornerEntryBoundary(cfg)
	laned := signrouter.ApplySignLanes(
		path,
		[]signrouter.LaneSpec{{Spec: sign, Corridor: trackmodel.South}},
		laneTestParams,
		cfg,
	)

	stepAtArcJoin := math.Abs(laned[3].Y - laned[2].Y)
	if stepAtArcJoin <= 0.15 {
		t.Errorf("step at arc/straight join = %v, want > 0.15 (the un-fixed defect)", stepAtArcJoin)
	}
}

// TestApplySignLanes_BorrowedRunwayShrinksTheWorstStep matches
// TestCornerEntry.test_borrowed_runway_shrinks_the_worst_step: relative to
// the no-runway case, not against an absolute threshold -- spreading the
// same lateral travel over more path makes the worst single step smaller,
// which is what the chassis actually has to track.
func TestApplySignLanes_BorrowedRunwayShrinksTheWorstStep(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	path := withArc(t)
	sign := signLaneCornerEntryBoundary(cfg)
	confined := signrouter.ApplySignLanes(
		path,
		[]signrouter.LaneSpec{{Spec: sign, Corridor: trackmodel.South}},
		laneTestParams,
		cfg,
	)
	borrowed := signrouter.ApplySignLanes(
		path,
		[]signrouter.LaneSpec{{Spec: sign, Corridor: trackmodel.South}},
		signrouter.SignLaneParams{
			LateralOffsetM: 0.28,
			RampM:          0.70,
			HoldM:          0.25,
			CornerEntryM:   0.45,
		},
		cfg,
	)

	worstConfined := maxStepIn(confined)
	worstBorrowed := maxStepIn(borrowed)
	if worstBorrowed >= worstConfined*0.75 {
		t.Errorf(
			"worst step: borrowed = %v, confined = %v; want borrowed < 0.75x confined",
			worstBorrowed,
			worstConfined,
		)
	}
}

// TestApplySignLanes_BorrowingTranslatesTheArcRatherThanFlatteningIt matches
// TestCornerEntry.test_borrowing_translates_the_arc_rather_than_flattening_it:
// the arc must stay a turn; only its position may move. Applying the
// profile as an absolute lateral instead of a shift would collapse every
// borrowed arc point onto one line.
func TestApplySignLanes_BorrowingTranslatesTheArcRatherThanFlatteningIt(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	path := withArc(t)
	sign := signLaneCornerEntryBoundary(cfg)
	params := signrouter.SignLaneParams{
		LateralOffsetM: 0.28,
		RampM:          0.70,
		HoldM:          0.25,
		CornerEntryM:   0.45,
	}
	laned := signrouter.ApplySignLanes(
		path,
		[]signrouter.LaneSpec{{Spec: sign, Corridor: trackmodel.South}},
		params,
		cfg,
	)

	arcBefore := []float64{path[0].Y, path[1].Y, path[2].Y}
	arcAfter := []float64{laned[0].Y, laned[1].Y, laned[2].Y}

	seen := map[float64]bool{}
	for _, y := range arcAfter {
		seen[y] = true
	}
	if len(seen) != len(arcBefore) {
		t.Errorf(
			"arc points collapsed onto %d distinct laterals (from %v), want %d distinct",
			len(seen),
			arcAfter,
			len(arcBefore),
		)
	}
	for i := 1; i < len(arcAfter); i++ {
		if !(arcAfter[i] < arcAfter[i-1]) {
			t.Errorf("arc after: %v is not monotonically descending at index %d", arcAfter, i)
		}
	}
}

// TestApplySignLanes_BorrowingNeverCrossesIntoTheNeighbouringCorridor
// matches
// TestCornerEntry.test_borrowing_never_moves_a_point_into_the_neighboring_corridor:
// the lateral test does not widen with the depth test, so the borrow
// self-limits.
func TestApplySignLanes_BorrowingNeverCrossesIntoTheNeighbouringCorridor(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	westPoint := trackmodel.Waypoint{X: 0.4, Y: 1.4} // a WEST-corridor point
	path := append([]trackmodel.Waypoint{westPoint}, withArc(t)...)
	sign := signLaneCornerEntryBoundary(cfg)
	params := signrouter.SignLaneParams{
		LateralOffsetM: 0.28,
		RampM:          0.70,
		HoldM:          0.25,
		CornerEntryM:   0.90,
	}

	laned := signrouter.ApplySignLanes(
		path,
		[]signrouter.LaneSpec{{Spec: sign, Corridor: trackmodel.South}},
		params,
		cfg,
	)
	if laned[0] != westPoint {
		t.Errorf("laned[0] = %+v, want unchanged WEST-corridor point %+v", laned[0], westPoint)
	}
}

// TestApplySignLanes_OpposingSignsEachGetTheirOwnSide matches
// TestTwoSigns.test_opposing_signs_each_get_their_own_side: opposite-side
// signs in one corridor must produce an S-bend, not an average.
func TestApplySignLanes_OpposingSignsEachGetTheirOwnSide(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	red := signrouter.SignSpec{X: 1.3, Y: 0.5, Color: signrouter.SignColorRed}
	green := signrouter.SignSpec{X: 2.0, Y: 0.5, Color: signrouter.SignColorGreen}
	laned := signrouter.ApplySignLanes(
		southStraight(t, 41),
		[]signrouter.LaneSpec{
			{Spec: red, Corridor: trackmodel.South},
			{Spec: green, Corridor: trackmodel.South},
		},
		laneTestParams, cfg,
	)

	if got, want := lateralAt(laned, 1.3), red.Y-laneTestParams.LateralOffsetM; math.Abs(
		got-want,
	) > 0.03 {
		t.Errorf("lateral at the red sign's depth = %v, want approx %v", got, want)
	}
	if got, want := lateralAt(laned, 2.0), green.Y+laneTestParams.LateralOffsetM; math.Abs(
		got-want,
	) > 0.03 {
		t.Errorf("lateral at the green sign's depth = %v, want approx %v", got, want)
	}
}
