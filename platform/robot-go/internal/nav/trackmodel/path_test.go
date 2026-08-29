package trackmodel_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

func TestProjectOntoPath_MidSegment(t *testing.T) {
	t.Parallel()

	waypoints := []trackmodel.Waypoint{{X: 0, Y: 0}, {X: 2, Y: 0}, {X: 2, Y: 2}}

	got := trackmodel.ProjectOntoPath(waypoints, 1, 0.5)

	if math.Abs(got.X-1) > tolerance || math.Abs(got.Y-0) > tolerance {
		t.Errorf("closest point = (%v, %v), want (1, 0)", got.X, got.Y)
	}
	if math.Abs(got.DistanceM-0.5) > tolerance {
		t.Errorf("DistanceM = %v, want 0.5", got.DistanceM)
	}
	if math.Abs(got.SignedOffsetM-0.5) > tolerance {
		t.Errorf("SignedOffsetM = %v, want 0.5 (point is left of the +x-heading segment)", got.SignedOffsetM)
	}
	if got.SegmentIndex != 0 {
		t.Errorf("SegmentIndex = %d, want 0", got.SegmentIndex)
	}
	if math.Abs(got.TangentRad-0) > tolerance {
		t.Errorf("TangentRad = %v, want 0", got.TangentRad)
	}
}

func TestProjectOntoPath_RightOfSegmentIsNegativeOffset(t *testing.T) {
	t.Parallel()

	waypoints := []trackmodel.Waypoint{{X: 0, Y: 0}, {X: 2, Y: 0}}
	got := trackmodel.ProjectOntoPath(waypoints, 1, -0.5)

	if got.SignedOffsetM >= 0 {
		t.Errorf("SignedOffsetM = %v, want negative (point is right of the +x-heading segment)", got.SignedOffsetM)
	}
}

func TestProjectOntoPath_ClampsAtSegmentEnds(t *testing.T) {
	t.Parallel()

	waypoints := []trackmodel.Waypoint{{X: 0, Y: 0}, {X: 2, Y: 0}}
	got := trackmodel.ProjectOntoPath(waypoints, 5, 0)

	if math.Abs(got.X-2) > tolerance || math.Abs(got.Y-0) > tolerance {
		t.Errorf("closest point = (%v, %v), want clamped to (2, 0)", got.X, got.Y)
	}
	if math.Abs(got.DistanceM-3) > tolerance {
		t.Errorf("DistanceM = %v, want 3", got.DistanceM)
	}
}

func TestProjectOntoPath_DegenerateSinglePointFallsBackToNearest(t *testing.T) {
	t.Parallel()

	waypoints := []trackmodel.Waypoint{{X: 5, Y: 5}}
	got := trackmodel.ProjectOntoPath(waypoints, 0, 0)

	wantDist := math.Hypot(5, 5)
	if math.Abs(got.DistanceM-wantDist) > tolerance {
		t.Errorf("DistanceM = %v, want %v", got.DistanceM, wantDist)
	}
	if got.SignedOffsetM != got.DistanceM {
		t.Errorf("SignedOffsetM = %v, want == DistanceM (unsigned fallback)", got.SignedOffsetM)
	}
}

func TestProjectOntoPath_EmptyPath(t *testing.T) {
	t.Parallel()

	got := trackmodel.ProjectOntoPath(nil, 1, 1)
	if !math.IsInf(got.DistanceM, 1) {
		t.Errorf("DistanceM = %v, want +Inf for an empty path", got.DistanceM)
	}
}

func TestCrossTrackError_MatchesProjectionDistance(t *testing.T) {
	t.Parallel()

	waypoints := []trackmodel.Waypoint{{X: 0, Y: 0}, {X: 2, Y: 0}}
	if got, want := trackmodel.CrossTrackError(waypoints, 1, 0.5), 0.5; math.Abs(got-want) > tolerance {
		t.Errorf("CrossTrackError() = %v, want %v", got, want)
	}
}

func TestPathTurnAhead_TooFewWaypoints(t *testing.T) {
	t.Parallel()

	waypoints := []trackmodel.Waypoint{{X: 0, Y: 0}, {X: 2, Y: 0}}
	if got := trackmodel.PathTurnAhead(waypoints, 0, 1.0); got != 0.0 {
		t.Errorf("PathTurnAhead() = %v, want 0 (fewer than 3 waypoints)", got)
	}
}

func TestPathTurnAhead_StraightSegmentReadsZero(t *testing.T) {
	t.Parallel()

	waypoints := []trackmodel.Waypoint{{X: 0, Y: 0}, {X: 1, Y: 0}, {X: 2, Y: 0}, {X: 3, Y: 0}}
	if got := trackmodel.PathTurnAhead(waypoints, 0, 1.0); got != 0.0 {
		t.Errorf("PathTurnAhead() = %v, want 0 along a straight", got)
	}
}

func TestPathTurnAhead_CornerReadsTheHeadingChange(t *testing.T) {
	t.Parallel()

	// A closed-loop square: (0,0) -> (2,0) -> (2,2) -> (0,2) -> wraps to (0,0).
	waypoints := []trackmodel.Waypoint{{X: 0, Y: 0}, {X: 2, Y: 0}, {X: 2, Y: 2}, {X: 0, Y: 2}}

	got := trackmodel.PathTurnAhead(waypoints, 0, 3.0) // reaches into the second (turning) segment
	want := math.Pi / 2
	if math.Abs(got-want) > tolerance {
		t.Errorf("PathTurnAhead() = %v, want %v (90deg corner)", got, want)
	}
}

func TestPathTurnAhead_WrapsAcrossTheSeam(t *testing.T) {
	t.Parallel()

	waypoints := []trackmodel.Waypoint{{X: 0, Y: 0}, {X: 2, Y: 0}, {X: 2, Y: 2}, {X: 0, Y: 2}}

	// Starting at the LAST waypoint, previewing forward must wrap around to
	// index 0 rather than reporting a straight because the slice ended.
	got := trackmodel.PathTurnAhead(waypoints, 3, 3.0)
	want := math.Pi / 2
	if math.Abs(got-want) > tolerance {
		t.Errorf("PathTurnAhead() at the seam = %v, want %v", got, want)
	}
}
