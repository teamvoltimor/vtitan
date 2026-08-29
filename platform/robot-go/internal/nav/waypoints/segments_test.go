package waypoints_test

import (
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/waypoints"
)

func TestDeduplicateConsecutive_ClosePointsRemoved(t *testing.T) {
	t.Parallel()

	cfg := waypoints.DefaultConfig() // default DedupeDistanceM is 0.001m
	pts := []trackmodel.Waypoint{{X: 0.0, Y: 0.0}, {X: 0.0001, Y: 0.0}, {X: 0.0002, Y: 0.0}, {X: 0.5, Y: 0.0}}

	got := waypoints.DeduplicateConsecutive(pts, cfg)
	if len(got) != 2 {
		t.Fatalf("len = %d, want 2: %v", len(got), got)
	}
	if got[0] != pts[0] || got[1] != pts[3] {
		t.Errorf("got %v, want [%v %v]", got, pts[0], pts[3])
	}
}

func TestValidateBounds(t *testing.T) {
	t.Parallel()

	const minCoord, maxCoord, cornerMin, cornerMax = 0.0, 3.0, 1.0, 2.0

	t.Run("in bounds and outside the corner square passes", func(t *testing.T) {
		t.Parallel()

		pt := []trackmodel.Waypoint{{X: 0.5, Y: 0.5}}
		if err := waypoints.ValidateBounds(pt, minCoord, maxCoord, cornerMin, cornerMax); err != nil {
			t.Errorf("ValidateBounds() = %v, want nil", err)
		}
	})

	t.Run("outside the track bounds errors", func(t *testing.T) {
		t.Parallel()

		pt := []trackmodel.Waypoint{{X: -0.1, Y: 0.5}}
		if err := waypoints.ValidateBounds(pt, minCoord, maxCoord, cornerMin, cornerMax); err == nil {
			t.Error("ValidateBounds() = nil, want an error for a point outside the track")
		}
	})

	t.Run("inside the restricted inner square errors", func(t *testing.T) {
		t.Parallel()

		pt := []trackmodel.Waypoint{{X: 1.5, Y: 1.5}}
		if err := waypoints.ValidateBounds(pt, minCoord, maxCoord, cornerMin, cornerMax); err == nil {
			t.Error("ValidateBounds() = nil, want an error for a point inside the inner square")
		}
	})
}

func TestAssembleLoop(t *testing.T) {
	t.Parallel()

	segments := map[trackmodel.Section][]trackmodel.Waypoint{
		trackmodel.South: {{X: 0, Y: 0}},
		trackmodel.West:  {{X: 1, Y: 1}},
	}
	order := []trackmodel.Section{trackmodel.South, trackmodel.West}

	got := waypoints.AssembleLoop(order, segments)
	want := []trackmodel.Waypoint{{X: 0, Y: 0}, {X: 1, Y: 1}}
	if len(got) != len(want) || got[0] != want[0] || got[1] != want[1] {
		t.Errorf("AssembleLoop() = %v, want %v", got, want)
	}
}

func TestNearestWaypointIndex(t *testing.T) {
	t.Parallel()

	pts := []trackmodel.Waypoint{{X: 0, Y: 0}, {X: 1, Y: 1}, {X: 5, Y: 5}}
	if got := waypoints.NearestWaypointIndex(pts, 0.9, 0.9); got != 1 {
		t.Errorf("NearestWaypointIndex() = %d, want 1", got)
	}
}

// TestBuildAllSegments_SymmetricLoopStaysInBounds assembles a full
// symmetric-corridor loop (BuildAllSegments -> AssembleLoop ->
// BuildWaypointSequence -> ValidateBounds), the same pipeline
// calculate_waypoints composes internally, minus the ScenarioMetadata
// parsing this port defers -- an integration check that the pieces
// actually fit together, not just the individual functions.
func TestBuildAllSegments_SymmetricLoopStaysInBounds(t *testing.T) {
	t.Parallel()

	const (
		minCoord, maxCoord, cornerMinM, cornerMaxM = 0.0, 3.0, 1.0, 2.0
		width                                      = 1.0 // symmetric wide corridors on all 4 sides
		biasM                                      = 0.10
	)

	cfg := waypoints.DefaultConfig()
	radius := waypoints.CornerArcRadius(width, width, biasM, 0.45)
	radii := waypoints.CornerRadii{SE: radius, SW: radius, NW: radius, NE: radius}

	// Centerlines biased toward the inner block on all 4 sides.
	northCY, southCY := maxCoord-width/2-biasM, width/2+biasM
	eastCX, westCX := maxCoord-width/2-biasM, width/2+biasM

	segments := waypoints.BuildAllSegments(northCY, southCY, eastCX, westCX, radii, trackmodel.Clockwise, cfg)
	order := []trackmodel.Section{trackmodel.South, trackmodel.West, trackmodel.North, trackmodel.East}
	fullLoop := waypoints.AssembleLoop(order, segments)

	if len(fullLoop) == 0 {
		t.Fatal("AssembleLoop() produced no waypoints")
	}

	sequence := waypoints.BuildWaypointSequence(fullLoop, segments, order, 1.5, southCY, 1, cfg)
	if len(sequence) < 20 {
		t.Errorf("len(sequence) = %d, want > 20 (matching calculate_waypoints' own sanity bound)", len(sequence))
	}

	if err := waypoints.ValidateBounds(sequence, minCoord, maxCoord, cornerMinM, cornerMaxM); err != nil {
		t.Errorf("ValidateBounds() = %v, want nil", err)
	}
}
