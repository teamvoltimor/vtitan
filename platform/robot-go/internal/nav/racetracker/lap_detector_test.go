// Package racetracker_test mirrors tests/unit/test_lap_detector.py case for
// case, including its fixture coordinates -- the geometry is the contract, so
// re-deriving positions here would test a different thing than the oracle.
package racetracker_test

import (
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/racetracker"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// newDetector mirrors test_lap_detector.py's _make: a SOUTH/CLOCKWISE
// detector with its start zone at (1.5, 0.2).
func newDetector(
	t *testing.T,
	section trackmodel.Section,
	direction trackmodel.Direction,
	x, y float64,
) *racetracker.LapDetector {
	t.Helper()

	detector, err := racetracker.NewLapDetector(trackmodel.Waypoint{X: x, Y: y}, section, direction)
	if err != nil {
		t.Fatalf("NewLapDetector() error = %v, want nil", err)
	}
	return detector
}

func newSouthCW(t *testing.T) *racetracker.LapDetector {
	t.Helper()
	return newDetector(t, trackmodel.South, trackmodel.Clockwise, 1.5, 0.2)
}

// feed mirrors _feed: positions are fed in order and the confirmed lap count
// is returned. The oracle's wrap_at parameter is omitted because every case
// here arms the waypoint half once, up front, rather than mid-sequence.
func feed(detector *racetracker.LapDetector, section trackmodel.Section, positions [][2]float64) int {
	laps := 0
	for _, pos := range positions {
		if detector.Update(trackmodel.Waypoint{X: pos[0], Y: pos[1]}, section) {
			laps++
		}
	}
	return laps
}

func TestLapDetector_ForwardCrossingWithWrapCountsLap(t *testing.T) {
	t.Parallel()

	detector := newSouthCW(t)
	detector.NotifyWaypointWrapped()

	// Approach from x > 1.5, where dot < 0 for the CW SOUTH normal (-1, 0).
	if got := feed(detector, trackmodel.South, [][2]float64{{2.0, 0.2}, {1.2, 0.2}}); got != 1 {
		t.Fatalf("laps = %d, want 1", got)
	}
}

func TestLapDetector_CrossingWithoutWrapDoesNotCount(t *testing.T) {
	t.Parallel()

	detector := newSouthCW(t)

	if got := feed(detector, trackmodel.South, [][2]float64{{2.0, 0.2}, {1.2, 0.2}}); got != 0 {
		t.Fatalf("laps = %d, want 0 without a waypoint wrap", got)
	}
}

func TestLapDetector_WrapWithoutCrossingDoesNotCount(t *testing.T) {
	t.Parallel()

	detector := newSouthCW(t)
	detector.NotifyWaypointWrapped()

	// Never reaches the line: dot stays negative throughout.
	if got := feed(detector, trackmodel.South, [][2]float64{{2.5, 0.2}, {2.0, 0.2}, {1.8, 0.2}}); got != 0 {
		t.Fatalf("laps = %d, want 0 without a geometric crossing", got)
	}
}

// TestLapDetector_OvershootAndReturnDoesNotDoubleCount is the failure the
// waypoint corroboration exists to prevent: crossing, overshooting, then
// drifting back through the line is one lap, not two.
func TestLapDetector_OvershootAndReturnDoesNotDoubleCount(t *testing.T) {
	t.Parallel()

	detector := newSouthCW(t)
	detector.NotifyWaypointWrapped()

	positions := [][2]float64{
		{2.0, 0.2}, // behind
		{1.2, 0.2}, // cross -> counts
		{0.9, 0.2}, // overshoot
		{1.6, 0.2}, // back through the line the wrong way
		{1.2, 0.2}, // forward again, but no new wrap has been notified
	}
	if got := feed(detector, trackmodel.South, positions); got != 1 {
		t.Fatalf("laps = %d, want exactly 1", got)
	}
}

// TestLapDetector_StuckLoopDoesNotDoubleCount covers a robot oscillating on
// the line, which without the wrap requirement would count a lap per wobble.
func TestLapDetector_StuckLoopDoesNotDoubleCount(t *testing.T) {
	t.Parallel()

	detector := newSouthCW(t)
	detector.NotifyWaypointWrapped()

	positions := [][2]float64{
		{2.0, 0.2}, {1.2, 0.2}, // cross -> counts
		{1.6, 0.2}, {1.2, 0.2}, // wobble
		{1.6, 0.2}, {1.2, 0.2}, // wobble
	}
	if got := feed(detector, trackmodel.South, positions); got != 1 {
		t.Fatalf("laps = %d, want exactly 1", got)
	}
}

// TestLapDetector_SectionGuard covers why the section test is not redundant
// with the dot product: on a closed loop the dot goes positive once per lap
// wherever the geometry lines up, so without it a crossing could be recorded
// on the far side of the mat.
func TestLapDetector_SectionGuard(t *testing.T) {
	t.Parallel()

	t.Run("wrong section not counted", func(t *testing.T) {
		t.Parallel()

		detector := newSouthCW(t)
		detector.NotifyWaypointWrapped()

		if got := feed(detector, trackmodel.North, [][2]float64{{2.0, 0.2}, {1.2, 0.2}}); got != 0 {
			t.Fatalf("laps = %d, want 0 while reported in the wrong section", got)
		}
	})

	t.Run("correct section counted", func(t *testing.T) {
		t.Parallel()

		detector := newSouthCW(t)
		detector.NotifyWaypointWrapped()

		if got := feed(detector, trackmodel.South, [][2]float64{{2.0, 0.2}, {1.2, 0.2}}); got != 1 {
			t.Fatalf("laps = %d, want 1", got)
		}
	})
}

// TestLapDetector_AllSectionsAndDirections mirrors the oracle's parametrized
// case, with its exact coordinates. This is what pins TravelNormalFor's sign
// convention: a flipped normal passes SOUTH/CW and fails its CCW twin.
func TestLapDetector_AllSectionsAndDirections(t *testing.T) {
	t.Parallel()

	type crossing struct {
		section   trackmodel.Section
		direction trackmodel.Direction
		start     [2]float64
		before    [2]float64
		after     [2]float64
	}

	cw, ccw := trackmodel.Clockwise, trackmodel.Counterclockwise
	tests := map[string]crossing{
		"south_cw": {
			trackmodel.South, cw,
			[2]float64{1.5, 0.2}, [2]float64{2.0, 0.2}, [2]float64{1.0, 0.2},
		},
		"north_cw": {
			trackmodel.North, cw,
			[2]float64{1.5, 2.8}, [2]float64{1.0, 2.8}, [2]float64{2.0, 2.8},
		},
		"east_cw": {
			trackmodel.East, cw,
			[2]float64{2.8, 1.5}, [2]float64{2.8, 2.0}, [2]float64{2.8, 1.0},
		},
		"west_cw": {
			trackmodel.West, cw,
			[2]float64{0.2, 1.5}, [2]float64{0.2, 1.0}, [2]float64{0.2, 2.0},
		},
		"south_ccw": {
			trackmodel.South, ccw,
			[2]float64{1.5, 0.2}, [2]float64{1.0, 0.2}, [2]float64{2.0, 0.2},
		},
		"north_ccw": {
			trackmodel.North, ccw,
			[2]float64{1.5, 2.8}, [2]float64{2.0, 2.8}, [2]float64{1.0, 2.8},
		},
		"east_ccw": {
			trackmodel.East, ccw,
			[2]float64{2.8, 1.5}, [2]float64{2.8, 1.0}, [2]float64{2.8, 2.0},
		},
		"west_ccw": {
			trackmodel.West, ccw,
			[2]float64{0.2, 1.5}, [2]float64{0.2, 2.0}, [2]float64{0.2, 1.0},
		},
	}

	for name, tt := range tests {
		t.Run(name, func(t *testing.T) {
			t.Parallel()

			detector := newDetector(t, tt.section, tt.direction, tt.start[0], tt.start[1])
			detector.NotifyWaypointWrapped()

			if got := feed(detector, tt.section, [][2]float64{tt.before, tt.after}); got != 1 {
				t.Fatalf("laps = %d, want 1", got)
			}
		})
	}
}

func TestLapDetector_ThreeLapSimulation(t *testing.T) {
	t.Parallel()

	detector := newSouthCW(t)
	counted := 0
	for range 3 {
		detector.NotifyWaypointWrapped()
		detector.Update(trackmodel.Waypoint{X: 2.0, Y: 0.2}, trackmodel.South) // behind
		if detector.Update(trackmodel.Waypoint{X: 1.2, Y: 0.2}, trackmodel.South) {
			counted++
		}
	}

	if counted != 3 {
		t.Fatalf("laps = %d, want exactly 3", counted)
	}
}

// TestLapDetector_OvershootEveryLapStillCountsThree is the combination case:
// a robot that overshoots and drifts back on EVERY lap must still finish on
// three, not six.
func TestLapDetector_OvershootEveryLapStillCountsThree(t *testing.T) {
	t.Parallel()

	detector := newSouthCW(t)
	counted := 0
	for range 3 {
		detector.NotifyWaypointWrapped()
		steps := [][2]float64{
			{2.0, 0.2}, // behind
			{1.2, 0.2}, // cross -> counts
			{0.9, 0.2}, // overshoot
			{1.6, 0.2}, // back through the line the wrong way
		}
		counted += feed(detector, trackmodel.South, steps)
	}

	if counted != 3 {
		t.Fatalf("laps = %d, want exactly 3", counted)
	}
}

// TestLapDetector_FirstSampleCannotCross covers the nil prevDot guard: with
// nothing to have crossed FROM, the opening sample must never count, even
// when it already sits past the line.
func TestLapDetector_FirstSampleCannotCross(t *testing.T) {
	t.Parallel()

	detector := newSouthCW(t)
	detector.NotifyWaypointWrapped()

	if detector.Update(trackmodel.Waypoint{X: 1.0, Y: 0.2}, trackmodel.South) {
		t.Fatal("first sample counted a lap, want no crossing without a previous position")
	}
}
