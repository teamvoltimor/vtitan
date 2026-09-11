package waypoints_test

import (
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/waypoints"
)

// cornerMin/cornerMax match track.toml's [track] corner_min/corner_max
// (1.0/2.0), the same fixture values test_waypoints.py's
// TestCorridorForPosition uses.
const (
	cornerMin = 1.0
	cornerMax = 2.0
)

func TestCorridorForPosition(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name string
		x, y float64
		want trackmodel.Section
	}{
		{name: "south corridor", x: 1.5, y: 0.5, want: trackmodel.South},
		{name: "north corridor", x: 1.5, y: 2.5, want: trackmodel.North},
		{name: "east corridor", x: 2.5, y: 1.5, want: trackmodel.East},
		{name: "west corridor", x: 0.5, y: 1.5, want: trackmodel.West},
		{name: "south boundary", x: 1.5, y: 0.99, want: trackmodel.South},
		{name: "north boundary", x: 1.5, y: 2.01, want: trackmodel.North},
		{name: "east boundary", x: 2.01, y: 1.5, want: trackmodel.East},
		{name: "west boundary", x: 0.99, y: 1.5, want: trackmodel.West},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			if got := waypoints.CorridorForPosition(tt.x, tt.y, cornerMin, cornerMax); got != tt.want {
				t.Errorf("CorridorForPosition(%v, %v) = %v, want %v", tt.x, tt.y, got, tt.want)
			}
		})
	}
}

func TestCorridorForPosition_CornerTiesBreakSouthNorthEastWest(t *testing.T) {
	t.Parallel()

	// (0.5, 0.5): dist to south face = dist to west face = 0.5 -- ties
	// resolve to South (checked first), matching the Python dict's
	// insertion-order tie-break.
	if got := waypoints.CorridorForPosition(0.5, 0.5, cornerMin, cornerMax); got != trackmodel.South {
		t.Errorf("SW corner tie = %v, want South", got)
	}
	// (2.5, 2.5): dist to north face = dist to east face = 0.5 -- ties
	// resolve to North (checked before East).
	if got := waypoints.CorridorForPosition(2.5, 2.5, cornerMin, cornerMax); got != trackmodel.North {
		t.Errorf("NE corner tie = %v, want North", got)
	}
}

func TestCorridorForPosition_AllFourSectionsReachable(t *testing.T) {
	t.Parallel()

	got := map[trackmodel.Section]bool{
		waypoints.CorridorForPosition(1.5, 0.3, cornerMin, cornerMax): true,
		waypoints.CorridorForPosition(1.5, 2.7, cornerMin, cornerMax): true,
		waypoints.CorridorForPosition(2.7, 1.5, cornerMin, cornerMax): true,
		waypoints.CorridorForPosition(0.3, 1.5, cornerMin, cornerMax): true,
	}
	for _, want := range []trackmodel.Section{trackmodel.South, trackmodel.North, trackmodel.East, trackmodel.West} {
		if !got[want] {
			t.Errorf("section %v not reached", want)
		}
	}
}
