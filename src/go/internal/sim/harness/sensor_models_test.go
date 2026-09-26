package harness_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/wallheading"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/collision"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/harness"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/kinematics"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/sensorerrors"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/sensormodel"
)

// squareTrack is a 3 m mat with four 1 m corridors.
func squareTrack() *collision.TrackModel {
	widths := map[trackmodel.Section]float64{
		trackmodel.North: 1.0, trackmodel.South: 1.0, trackmodel.East: 1.0, trackmodel.West: 1.0,
	}
	return collision.NewTrackModel(collision.NewTrackModelParams{
		Geometry:  trackmodel.CorridorGeometryFromWidths(widths, 3.0),
		MinCoordM: 0.0,
		MaxCoordM: 3.0,
	})
}

func newGateway(cfg harness.Config) *harness.SimHardwareGateway {
	return harness.NewSimHardwareGateway(
		cfg, squareTrack(), kinematics.AckermannState{X: 1.5, Y: 0.5},
		kinematics.NewAckermannKinematics(kinematics.DefaultParams()), 1,
	)
}

// With the bands, the rear rays read no return or a sub-floor self-return,
// the rays outside them are untouched by the bands, and the noise and
// invalid-ray draws outside them stay those of a run without bands.
func TestLidarBands_BlindTheRearAndSpareTheRest(t *testing.T) {
	t.Parallel()

	cfg := harness.DefaultConfig()
	cfg.LidarSamples = 500
	cfg.LidarMinRangeM = 0.045
	cfg.LidarMaxRangeM = 12
	plain := newGateway(cfg)
	p := sensormodel.DefaultParams().Lidar
	cfg.LidarBands = &p
	banded := newGateway(cfg)

	plainScan, _ := plain.GetLidarScan()
	scan, _ := banded.GetLidarScan()
	var inBand, blind int
	for i, a := range scan.AnglesRad {
		abs := math.Abs(a)
		if abs < p.BandMinRad || abs > p.BandMaxRad {
			if scan.RangesM[i] != plainScan.RangesM[i] && plainScan.RangesM[i] > cfg.LidarMinRangeM {
				t.Fatalf("ray %d outside the bands changed: %v, was %v", i, scan.RangesM[i], plainScan.RangesM[i])
			}
			continue
		}
		inBand++
		if r := scan.RangesM[i]; r == cfg.LidarMaxRangeM || r < 0.044 {
			blind++
		}
	}
	if inBand == 0 || blind != inBand {
		t.Errorf("%d of %d band rays read the world, want none", inBand-blind, inBand)
	}
}

// A yaw bias the walls can see is pulled out by the heading correction and
// kept without it.
func TestHeadingCorrection_PullsABiasOut(t *testing.T) {
	t.Parallel()

	const bias = 0.1
	cfg := harness.DefaultConfig()
	cfg.LidarSamples = 500
	cfg.LidarMaxRangeM = 12
	cfg.SensorErrors = sensorerrors.Errors{YawBiasRad: bias}
	uncorrected := newGateway(cfg)
	cfg.HeadingCorrection = &harness.HeadingCorrection{Gain: 0.05, Walls: wallheading.DefaultConfig()}
	corrected := newGateway(cfg)

	for range 400 {
		uncorrected.Advance(0.05)
		corrected.Advance(0.05)
	}
	u, _ := uncorrected.GetCurrentPose()
	c, _ := corrected.GetCurrentPose()
	if math.Abs(math.Abs(u.Yaw)-bias) > 1e-9 {
		t.Errorf("uncorrected yaw error = %.4f, want the %.2f bias", math.Abs(u.Yaw), bias)
	}
	if math.Abs(c.Yaw) > bias/4 {
		t.Errorf("corrected yaw error = %.4f, want under %.3f", math.Abs(c.Yaw), bias/4)
	}
}
