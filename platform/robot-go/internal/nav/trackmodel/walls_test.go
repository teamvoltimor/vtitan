package trackmodel_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// symmetricWalls builds a 3x3m track with a 1.0m corridor on every side
// (inner block spanning [1,2]x[1,2]), matching a "wide" WRO layout.
func symmetricWalls(t *testing.T) *trackmodel.TrackWalls {
	t.Helper()
	widths := map[trackmodel.Section]float64{
		trackmodel.North: 1.0, trackmodel.South: 1.0, trackmodel.East: 1.0, trackmodel.West: 1.0,
	}
	geometry := trackmodel.CorridorGeometryFromWidths(widths, 3.0)
	return trackmodel.NewTrackWalls(geometry, 0.0, 3.0)
}

func TestTrackWalls_Raycast_HitsInnerBlock(t *testing.T) {
	t.Parallel()

	walls := symmetricWalls(t)
	got := walls.Raycast(0.5, 1.5, 0, []float64{0}, 0.05, 12.0)

	if math.Abs(got[0]-0.5) > tolerance {
		t.Errorf(
			"Raycast() forward from (0.5,1.5) = %v, want 0.5 (inner block's west face at x=1)",
			got[0],
		)
	}
}

func TestTrackWalls_Raycast_HitsOuterWall(t *testing.T) {
	t.Parallel()

	walls := symmetricWalls(t)
	// Robot-frame angle -pi/2 ("right") at yaw=0 points world -y, hitting
	// the south outer wall.
	got := walls.Raycast(0.5, 0.5, 0, []float64{-math.Pi / 2}, 0.05, 12.0)

	if math.Abs(got[0]-0.5) > tolerance {
		t.Errorf("Raycast() right from (0.5,0.5) = %v, want 0.5 (south outer wall at y=0)", got[0])
	}
}

func TestTrackWalls_Raycast_ClampsToSensorMaxRange(t *testing.T) {
	t.Parallel()

	walls := symmetricWalls(t)
	// From (0.5, 0.1) facing +y (world), the ray misses the inner block
	// entirely (x=0.5 is outside its [1,2] span) and the real hit is the
	// north outer wall at y=3, ~2.9m away -- farther than the 1.0m max
	// range supplied here, so the result must clamp to that ceiling.
	got := walls.Raycast(0.5, 0.1, 0, []float64{math.Pi / 2}, 0.05, 1.0)

	if got[0] != 1.0 {
		t.Errorf(
			"Raycast() with a 1.0m max range = %v, want clamped to 1.0 (real hit is ~2.9m away)",
			got[0],
		)
	}
}

func TestTrackWalls_PointInFreeSpace(t *testing.T) {
	t.Parallel()

	walls := symmetricWalls(t)

	tests := []struct {
		name string
		x, y float64
		want bool
	}{
		{name: "inside a corridor", x: 0.5, y: 0.5, want: true},
		{name: "inside the inner block", x: 1.5, y: 1.5, want: false},
		{name: "outside the track entirely", x: -0.1, y: 0.5, want: false},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			if got := walls.PointInFreeSpace(tt.x, tt.y, 0); got != tt.want {
				t.Errorf("PointInFreeSpace(%v, %v) = %v, want %v", tt.x, tt.y, got, tt.want)
			}
		})
	}
}

func TestTrackWalls_PointInFreeSpace_ClearanceMargin(t *testing.T) {
	t.Parallel()

	walls := symmetricWalls(t)

	// 0.5 meters inside the corridor but within 0.6m clearance of the
	// outer wall at x=0 -- must be rejected once a clearance margin is
	// required, even though it passed with zero margin.
	if got := walls.PointInFreeSpace(0.5, 0.5, 0.6); got {
		t.Error(
			"PointInFreeSpace with a 0.6m clearance margin at x=0.5 = true, want false (too close to outer wall)",
		)
	}
}
