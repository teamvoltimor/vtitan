// Package localization_test mirrors tests/unit/test_lidar_localizer.py,
// including its fixture poses and corridor layouts.
package localization_test

import (
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/localization"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// lidarSamples matches robot.toml's [lidar] samples: one 360 deg sweep.
const lidarSamples = 500

// trackMinCoord/trackMaxCoord match robot.toml's track bounds. With uniform
// 1.0 m corridors the inner block spans [1, 2] on both axes.
const (
	trackMinCoord = 0.0
	trackMaxCoord = 3.0
)

// scanAngles mirrors _ANGLES: linspace(-pi, pi, samples, endpoint=False), an
// OPEN interval, so the last ray stops short of +pi rather than duplicating
// the first.
func scanAngles() []float64 {
	return navutil.AngleFan(lidarSamples)
}

func wallsForWidths(widths map[trackmodel.Section]float64) *trackmodel.TrackWalls {
	geometry := trackmodel.CorridorGeometryFromWidths(widths, trackMaxCoord)
	return trackmodel.NewTrackWalls(geometry, trackMinCoord, trackMaxCoord)
}

func uniformWidths(width float64) map[trackmodel.Section]float64 {
	return map[trackmodel.Section]float64{
		trackmodel.North: width,
		trackmodel.South: width,
		trackmodel.East:  width,
		trackmodel.West:  width,
	}
}

// sensorScan is the ranges a robot at (x, y, yaw) would measure, mirroring
// _sensor_scan.
//
// Cast from the LIDAR, which sits LidarMountXOffsetM forward of the chassis
// center, NOT from the center. Casting from the center is what the Python
// tests did until 2026-08-21, and it agreed with both the simulator and the
// localizer because all three shared the omission -- so the suite passed
// while the modeled sensor sat 12.2 cm behind the real one.
func sensorScan(walls *trackmodel.TrackWalls, x, y, yaw float64, angles []float64) []float64 {
	cfg := localization.DefaultConfig()
	return walls.Raycast(
		x+cfg.LidarMountXOffsetM*math.Cos(yaw),
		y+cfg.LidarMountXOffsetM*math.Sin(yaw),
		yaw, angles, cfg.LidarMinRangeM, cfg.LidarMaxRangeM,
	)
}

func newLocalizer(
	widths map[trackmodel.Section]float64,
) (*localization.LidarLocalizer, *trackmodel.TrackWalls) {
	walls := wallsForWidths(widths)
	return localization.New(walls, localization.DefaultConfig()), walls
}

// TestRecoversExactPoseFromCleanScan mirrors the oracle's straight and corner
// pose sweep. The prior is offset by a plausible per-tick displacement rather
// than the exact truth, so the search has something to actually find.
func TestRecoversExactPoseFromCleanScan(t *testing.T) {
	t.Parallel()

	poses := map[string][3]float64{
		"south straight facing east": {1.5, 0.5, 0.0},
		"south straight facing west": {1.5, 0.5, math.Pi},
		"east straight facing north": {2.5, 1.5, math.Pi / 2},
		"west straight facing south": {0.5, 1.5, -math.Pi / 2},
		"north straight off-axis":    {1.5, 2.5, 0.3},
		"southwest corner":           {0.6, 0.6, math.Pi / 4},
		"southeast corner":           {2.4, 0.6, -math.Pi / 4},
		"northeast corner":           {2.4, 2.4, 3 * math.Pi / 4},
		"northwest corner":           {0.6, 2.4, -3 * math.Pi / 4},
	}

	angles := scanAngles()
	for name, pose := range poses {
		t.Run(name, func(t *testing.T) {
			t.Parallel()

			localizer, walls := newLocalizer(uniformWidths(1.0))
			x, y, yaw := pose[0], pose[1], pose[2]
			ranges := sensorScan(walls, x, y, yaw, angles)

			got := localizer.EstimatePosition(
				trackmodel.Waypoint{X: x - 0.03, Y: y + 0.02}, yaw, ranges, angles, nil,
			)

			if math.Abs(got.X-x) > 0.02 || math.Abs(got.Y-y) > 0.02 {
				t.Fatalf(
					"estimate = (%.4f, %.4f), want (%.4f, %.4f) within 0.02",
					got.X,
					got.Y,
					x,
					y,
				)
			}
		})
	}
}

// TestRecoversPoseAcrossCorridorWidths uses a position valid under all three
// layouts -- the narrowest corridor still leaves room at the midline.
func TestRecoversPoseAcrossCorridorWidths(t *testing.T) {
	t.Parallel()

	layouts := map[string]map[trackmodel.Section]float64{
		"uniform 1.0": uniformWidths(1.0),
		"narrow 0.6":  uniformWidths(0.6),
		"mixed": {
			trackmodel.North: 1.0, trackmodel.South: 0.6,
			trackmodel.East: 1.0, trackmodel.West: 0.6,
		},
	}

	angles := scanAngles()
	const x, y, yaw = 1.5, 0.3, 0.2

	for name, widths := range layouts {
		t.Run(name, func(t *testing.T) {
			t.Parallel()

			localizer, walls := newLocalizer(widths)
			ranges := sensorScan(walls, x, y, yaw, angles)

			got := localizer.EstimatePosition(
				trackmodel.Waypoint{X: x - 0.03, Y: y - 0.03}, yaw, ranges, angles, nil,
			)

			if math.Abs(got.X-x) > 0.02 || math.Abs(got.Y-y) > 0.02 {
				t.Fatalf(
					"estimate = (%.4f, %.4f), want (%.4f, %.4f) within 0.02",
					got.X,
					got.Y,
					x,
					y,
				)
			}
		})
	}
}

// TestRejectsResultOutsideTrackBounds covers the free-space guard against the
// real 2026-08-04 failure: a match snapping off-track during a k-turn escape
// and staying there for the rest of the run.
func TestRejectsResultOutsideTrackBounds(t *testing.T) {
	t.Parallel()

	localizer, walls := newLocalizer(uniformWidths(1.0))
	angles := scanAngles()
	prior := trackmodel.Waypoint{X: 0.05, Y: 1.5}

	// Mathematically valid raycast geometry from x < 0; physically impossible.
	ranges := sensorScan(walls, -0.2, 1.5, 0.0, angles)

	if got := localizer.EstimatePosition(prior, 0.0, ranges, angles, nil); got != prior {
		t.Fatalf("estimate = %+v, want the prior %+v held", got, prior)
	}
}

// TestRejectsResultInsideInnerBlock covers the other half of the same guard:
// being inside the solid island is exactly as impossible as being outside the
// outer walls, so it is rejected by the same check rather than a narrower
// bounds-only one.
func TestRejectsResultInsideInnerBlock(t *testing.T) {
	t.Parallel()

	localizer, walls := newLocalizer(uniformWidths(1.0))
	angles := scanAngles()
	prior := trackmodel.Waypoint{X: 0.9, Y: 1.5}

	ranges := sensorScan(walls, 1.5, 1.5, 0.0, angles)

	if got := localizer.EstimatePosition(prior, 0.0, ranges, angles, nil); got != prior {
		t.Fatalf("estimate = %+v, want the prior %+v held", got, prior)
	}
}

// TestAcceptsLargeInBoundsCorrection is the start-placement-absorption case:
// a big single-tick jump is legitimate as long as it lands inside the track
// and there is no elapsed-time baseline yet.
func TestAcceptsLargeInBoundsCorrection(t *testing.T) {
	t.Parallel()

	localizer, walls := newLocalizer(uniformWidths(1.0))
	angles := scanAngles()
	const trueX, trueY = 1.5, 0.5

	ranges := sensorScan(walls, trueX, trueY, 0.0, angles)
	got := localizer.EstimatePosition(
		trackmodel.Waypoint{X: 1.35, Y: 0.5},
		0.0,
		ranges,
		angles,
		nil,
	)

	if math.Abs(got.X-trueX) > 0.02 || math.Abs(got.Y-trueY) > 0.02 {
		t.Fatalf(
			"estimate = (%.4f, %.4f), want (%.4f, %.4f) within 0.02",
			got.X,
			got.Y,
			trueX,
			trueY,
		)
	}
}

// TestRejectsImplausiblyFastJump is the same kind of jump as the test above,
// but on a later tick where an elapsed-time baseline exists -- which is the
// whole distinction between the two.
func TestRejectsImplausiblyFastJump(t *testing.T) {
	t.Parallel()

	localizer, walls := newLocalizer(uniformWidths(1.0))
	angles := scanAngles()
	const x0, y0 = 1.5, 0.5

	t0 := 0.0
	ranges0 := sensorScan(walls, x0, y0, 0.0, angles)
	first := localizer.EstimatePosition(
		trackmodel.Waypoint{X: x0, Y: y0},
		0.0,
		ranges0,
		angles,
		&t0,
	)

	// 0.15 m in 0.05 s implies 3 m/s, far beyond MaxSpeedMPS.
	t1 := 0.05
	ranges1 := sensorScan(walls, x0+0.15, y0, 0.0, angles)
	second := localizer.EstimatePosition(first, 0.0, ranges1, angles, &t1)

	if second != first {
		t.Fatalf("estimate = %+v, want the prior %+v held on an impossible jump", second, first)
	}
}

// TestConfirmsRepeatedJumpOnNextTick covers the other side of the speed
// guard: a real correction reconverges to nearly the same position from an
// independent scan, so the same candidate winning twice is trusted.
func TestConfirmsRepeatedJumpOnNextTick(t *testing.T) {
	t.Parallel()

	localizer, walls := newLocalizer(uniformWidths(1.0))
	angles := scanAngles()
	const x0, y0 = 1.5, 0.5

	t0 := 0.0
	ranges0 := sensorScan(walls, x0, y0, 0.0, angles)
	first := localizer.EstimatePosition(
		trackmodel.Waypoint{X: x0, Y: y0},
		0.0,
		ranges0,
		angles,
		&t0,
	)

	farRanges := sensorScan(walls, x0+0.15, y0, 0.0, angles)

	t1 := 0.05
	held := localizer.EstimatePosition(first, 0.0, farRanges, angles, &t1)
	if held != first {
		t.Fatalf("first impossible jump was accepted: %+v", held)
	}

	// Same candidate, next tick: now trusted.
	t2 := 0.10
	confirmed := localizer.EstimatePosition(first, 0.0, farRanges, angles, &t2)
	if confirmed == first {
		t.Fatal("repeated jump was not confirmed on the second tick")
	}
	if math.Abs(confirmed.X-(x0+0.15)) > 0.02 {
		t.Fatalf("confirmed X = %.4f, want %.4f within 0.02", confirmed.X, x0+0.15)
	}
}

// TestResetTrackingDiscardsHeldCandidate covers why ResetTracking exists: a
// candidate held from the old frame must not be able to confirm the first
// estimate after a re-seed, which is exactly the corruption the re-seed is
// meant to discard.
func TestResetTrackingDiscardsHeldCandidate(t *testing.T) {
	t.Parallel()

	localizer, walls := newLocalizer(uniformWidths(1.0))
	angles := scanAngles()
	const x0, y0 = 1.5, 0.5

	t0 := 0.0
	ranges0 := sensorScan(walls, x0, y0, 0.0, angles)
	first := localizer.EstimatePosition(
		trackmodel.Waypoint{X: x0, Y: y0},
		0.0,
		ranges0,
		angles,
		&t0,
	)

	farRanges := sensorScan(walls, x0+0.15, y0, 0.0, angles)
	t1 := 0.05
	if held := localizer.EstimatePosition(first, 0.0, farRanges, angles, &t1); held != first {
		t.Fatalf("first impossible jump was accepted: %+v", held)
	}

	localizer.ResetTracking()

	// Without the reset this tick would confirm the held candidate. With it,
	// there is no elapsed-time baseline either, so the guard is skipped and
	// the match is accepted on its own merits rather than by confirmation.
	t2 := 0.10
	after := localizer.EstimatePosition(first, 0.0, farRanges, angles, &t2)
	if math.Abs(after.X-(x0+0.15)) > 0.02 {
		t.Fatalf("estimate X = %.4f, want %.4f within 0.02", after.X, x0+0.15)
	}
}

// TestMismatchedScanLengthsHoldPrior covers the guard Python gets for free
// from a numpy broadcast error: bearings that do not match ranges cannot be
// scored, and holding the prior is the same outcome as a rejected match.
func TestMismatchedScanLengthsHoldPrior(t *testing.T) {
	t.Parallel()

	localizer, _ := newLocalizer(uniformWidths(1.0))
	prior := trackmodel.Waypoint{X: 1.5, Y: 0.5}

	got := localizer.EstimatePosition(prior, 0.0, []float64{1.0, 2.0}, []float64{0.0}, nil)
	if got != prior {
		t.Fatalf("estimate = %+v, want the prior %+v held", got, prior)
	}
}
