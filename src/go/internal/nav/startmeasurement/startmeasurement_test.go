// Package startmeasurement_test ports tests/unit/test_start_measurement.py's
// cases directly: this package's dependencies (trackmodel.TrackWalls for the
// scan oracle) are already ported, unlike wallheading_test.go's situation.
package startmeasurement_test

import (
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/startmeasurement"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

const (
	lidarSamples  = 360
	trackMinCoord = 0.0
	trackMaxCoord = 3.0
	lidarMinRange = 0.045
	lidarMaxRange = 12.0
)

// uniformWalls builds the 1.0 m uniform layout every case here uses;
// corridor-width variation is covered by the localization suite.
func uniformWalls() *trackmodel.TrackWalls {
	const widthM = 1.0
	widths := map[trackmodel.Section]float64{
		trackmodel.North: widthM, trackmodel.South: widthM,
		trackmodel.East: widthM, trackmodel.West: widthM,
	}
	geometry := trackmodel.CorridorGeometryFromWidths(widths, trackMaxCoord)
	return trackmodel.NewTrackWalls(geometry, trackMinCoord, trackMaxCoord)
}

// scanAngles mirrors the Python oracle's np.linspace(-pi, pi, LIDAR_SAMPLES,
// endpoint=False) -- a half-open interval, matching real driver sampling.
func scanAngles() []float64 {
	return navutil.AngleFan(lidarSamples)
}

// scan raycasts a noise-free sweep at a known pose on a known layout.
func scan(walls *trackmodel.TrackWalls, x, y, yaw float64) (ranges, angles []float64) {
	angles = scanAngles()
	ranges = walls.Raycast(x, y, yaw, angles, lidarMinRange, lidarMaxRange)
	return ranges, angles
}

func testConfig() startmeasurement.Config {
	cfg := startmeasurement.DefaultConfig()
	cfg.TrackMaxCoordM = trackMaxCoord
	cfg.LidarMinRangeM = lidarMinRange
	cfg.LidarMaxRangeM = lidarMaxRange
	return cfg
}

// TestRecoversThePoseItWasTakenAt mirrors TestMeasuredPose's
// test_recovers_the_pose_it_was_taken_at, including the two cases placed
// OUTSIDE the marked starting square -- what actually happened on
// 2026-08-05 -- which the measurement must not care about.
func TestRecoversThePoseItWasTakenAt(t *testing.T) {
	t.Parallel()

	cases := []struct {
		name      string
		x, y      float64
		direction trackmodel.Direction
		yaw       float64
	}{
		{"west_cell_ccw", 1.25, 0.497, trackmodel.Counterclockwise, 0.0},
		{"east_cell_ccw", 1.75, 0.497, trackmodel.Counterclockwise, 0.0},
		{"west_cell_cw", 1.25, 0.497, trackmodel.Clockwise, math.Pi},
		{"east_cell_cw", 1.75, 0.497, trackmodel.Clockwise, math.Pi},
		{"outer_band", 1.25, 0.303, trackmodel.Counterclockwise, 0.0},
		{"inner_band", 1.75, 0.697, trackmodel.Clockwise, math.Pi},
		{"outside_marked_square_ccw", 2.30, 0.500, trackmodel.Counterclockwise, 0.0},
		{"outside_marked_square_cw", 0.70, 0.500, trackmodel.Clockwise, math.Pi},
	}

	walls := uniformWalls()
	cfg := testConfig()

	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()

			ranges, angles := scan(walls, tc.x, tc.y, tc.yaw)

			measured, ok := startmeasurement.MeasureStartPose(
				ranges, angles, tc.direction, trackmodel.South, cfg,
			)

			if !ok {
				t.Fatal("MeasureStartPose = ok false, want true")
			}
			const tolM = 0.02
			if math.Abs(measured.X-tc.x) > tolM || math.Abs(measured.Y-tc.y) > tolM {
				t.Errorf(
					"pose = (%.4f, %.4f), want (%.4f, %.4f) within %v",
					measured.X,
					measured.Y,
					tc.x,
					tc.y,
					tolM,
				)
			}
		})
	}
}

// TestReportsTheTrackActuallyLeftAhead is the number whose absence lost the
// 2026-08-05 rounds: placed at 2.30 traveling counterclockwise there is
// 0.70 m to the wall, not the ~1.5 m a pose assumed at the middle of the
// side implies.
func TestReportsTheTrackActuallyLeftAhead(t *testing.T) {
	t.Parallel()

	ranges, angles := scan(uniformWalls(), 2.30, 0.50, 0.0)
	cfg := testConfig()

	measured, ok := startmeasurement.MeasureStartPose(
		ranges, angles, trackmodel.Counterclockwise, trackmodel.South, cfg,
	)

	if !ok {
		t.Fatal("MeasureStartPose = ok false, want true")
	}
	if math.Abs(measured.DistanceAheadM-0.70) > 0.02 {
		t.Errorf("DistanceAheadM = %v, want ~0.70", measured.DistanceAheadM)
	}
}

// TestMeasuresCorridorWidthWhenBesideInnerBlock checks that, level with the
// block, the side rays span the corridor and nothing else.
func TestMeasuresCorridorWidthWhenBesideInnerBlock(t *testing.T) {
	t.Parallel()

	ranges, angles := scan(uniformWalls(), 1.25, 0.497, 0.0)
	cfg := testConfig()

	measured, ok := startmeasurement.MeasureStartPose(
		ranges, angles, trackmodel.Counterclockwise, trackmodel.South, cfg,
	)

	if !ok {
		t.Fatal("MeasureStartPose = ok false, want true")
	}
	if !measured.CorridorWidthKnown {
		t.Fatal("CorridorWidthKnown = false, want true")
	}
	if math.Abs(measured.CorridorWidthM-1.0) > 0.03 {
		t.Errorf("CorridorWidthM = %v, want ~1.0", measured.CorridorWidthM)
	}
}

// TestReportsNoWidthWhenLevelWithACorner checks that past the block both
// side rays reach outer walls, measuring the mat -- reporting that as a
// corridor width would hand the width estimator a 3 m corridor, so it must
// be withheld instead.
func TestReportsNoWidthWhenLevelWithACorner(t *testing.T) {
	t.Parallel()

	ranges, angles := scan(uniformWalls(), 2.60, 0.50, 0.0)
	cfg := testConfig()

	measured, ok := startmeasurement.MeasureStartPose(
		ranges, angles, trackmodel.Counterclockwise, trackmodel.South, cfg,
	)

	if !ok {
		t.Fatal("MeasureStartPose = ok false, want true")
	}
	if measured.CorridorWidthKnown {
		t.Errorf("CorridorWidthKnown = true (%v), want false", measured.CorridorWidthM)
	}
}

// TestSectionIsAFreeRelabelling checks that the same scan read against
// another section gives that section's pose: with equal corridors the
// track is invariant under a quarter turn, so the robot's choice of
// starting section is a label, not a claim -- the measured pose must
// rotate with it and stay self-consistent.
func TestSectionIsAFreeRelabelling(t *testing.T) {
	t.Parallel()

	ranges, angles := scan(uniformWalls(), 1.25, 0.497, 0.0)
	cfg := testConfig()

	south, ok := startmeasurement.MeasureStartPose(
		ranges, angles, trackmodel.Counterclockwise, trackmodel.South, cfg,
	)
	if !ok {
		t.Fatal("MeasureStartPose(South) = ok false, want true")
	}
	east, ok := startmeasurement.MeasureStartPose(
		ranges, angles, trackmodel.Counterclockwise, trackmodel.East, cfg,
	)
	if !ok {
		t.Fatal("MeasureStartPose(East) = ok false, want true")
	}

	wantEastX, wantEastY := trackMaxCoord-south.Y, south.X
	if math.Abs(east.X-wantEastX) > 1e-6 || math.Abs(east.Y-wantEastY) > 1e-6 {
		t.Errorf("east pose = (%v, %v), want (%v, %v)", east.X, east.Y, wantEastX, wantEastY)
	}
}

// TestRejectsABlockedRay is the realistic case: an operator still standing
// over the robot. The rays either side of the mat must span it; a hand at
// 0.3 m where the wall is at 2.3 m breaks that by two meters, far outside
// any tolerance an imperfect track needs.
func TestRejectsABlockedRay(t *testing.T) {
	t.Parallel()

	ranges, angles := scan(uniformWalls(), 1.25, 0.497, 0.0)
	blocked := make([]float64, len(ranges))
	copy(blocked, ranges)
	for i, a := range angles {
		wrapped := math.Atan2(math.Sin(a-math.Pi), math.Cos(a-math.Pi))
		if math.Abs(wrapped) < 6*math.Pi/180 {
			blocked[i] = 0.3
		}
	}

	_, ok := startmeasurement.MeasureStartPose(
		blocked, angles, trackmodel.Counterclockwise, trackmodel.South, testConfig(),
	)

	if ok {
		t.Error("MeasureStartPose with a blocked ray = ok true, want false")
	}
}

// TestAcceptsATrackThatIsNotPerfect checks real mats are not nominal, and
// rejecting them would be useless: two real rounds closed to 2.978 m and
// 2.971 m against a nominal 3.0 before any noise, so a measurement must
// survive several centimeters of error in the mat itself.
func TestAcceptsATrackThatIsNotPerfect(t *testing.T) {
	t.Parallel()

	cfg := testConfig()
	ranges, angles := scan(uniformWalls(), 1.25, 0.497, 0.0)
	shrink := 1.0 - 0.9*cfg.ClosingToleranceM/trackMaxCoord
	shrunk := make([]float64, len(ranges))
	for i, r := range ranges {
		shrunk[i] = r * shrink
	}

	_, ok := startmeasurement.MeasureStartPose(
		shrunk, angles, trackmodel.Counterclockwise, trackmodel.South, cfg,
	)

	if !ok {
		t.Error("MeasureStartPose with a slightly shrunk track = ok false, want true")
	}
}

// TestRejectsWhenNoRayReturns checks that off the track entirely -- a
// bench, a table -- measures nothing.
func TestRejectsWhenNoRayReturns(t *testing.T) {
	t.Parallel()

	angles := scanAngles()
	empty := make([]float64, len(angles))
	for i := range empty {
		empty[i] = lidarMaxRange
	}

	_, ok := startmeasurement.MeasureStartPose(
		empty, angles, trackmodel.Counterclockwise, trackmodel.South, testConfig(),
	)

	if ok {
		t.Error("MeasureStartPose with no valid returns = ok true, want false")
	}
}
