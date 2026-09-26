package sensormodel

import (
	"log/slog"
	"math"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/simulation"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/sensorerrors"
)

// LidarParams is the measured C1 model. Provenance for every value is in
// simulation.toml; the rates come from 5.76 M rays over three hardware
// rounds (2026-09-14), the band angles from the robot-frame re-measurement
// of 2026-09-15.
type LidarParams struct {
	// InvalidRayRate is the no-return rate OUTSIDE the bands (0.095). The
	// whole-sweep 25% is dominated by the bands and must not be applied
	// uniformly.
	InvalidRayRate float64
	// BandMinRad / BandMaxRad bound |bearing| of the two symmetric rear
	// bands the chassis blocks, robot frame (0 = nose).
	BandMinRad float64
	BandMaxRad float64
	// BandDropoutRate is the share of band rays with no return (0.689).
	BandDropoutRate float64
	// SelfReturnM / SelfReturnStdM describe the rest: a return off the
	// chassis itself, clipped to +-3 sigma, every one below the 0.044 m
	// filter floor.
	SelfReturnM    float64
	SelfReturnStdM float64
}

// VisionParams is the measured camera model: which detections arrive and
// how wrong they are. Latency is not here: it stays the transport
// emulation's DetectionDelayS, measured and swept on its own.
type VisionParams struct {
	// RangeModel keeps a sign with probability 1/(1+exp((d-R50M)/FalloffM)),
	// d measured from the chassis centre: the real detector's median range
	// is 0.70 m, not the 10 m far clip.
	RangeModel bool
	R50M       float64
	FalloffM   float64
	// FrameMissRate drops each surviving sign independently, calibrated to
	// the hardware's 29.6% share of ticks with a usable color observation.
	FrameMissRate float64
	// ConfidenceQuantiles at ConfidenceLevels is the measured confidence
	// distribution; a mismatched pair falls back to FallbackConfidence.
	ConfidenceQuantiles []float64
	ConfidenceLevels    []float64
	FallbackConfidence  float64
	// BearingScatterRad rotates each detection about the reporting pose by
	// a Gaussian angle, keeping its range.
	BearingScatterRad float64
	// ColorFlipRate swaps red and green per detection (111 of 2,162).
	ColorFlipRate float64
}

// Params is every model's measured values.
type Params struct {
	Lidar  LidarParams
	Vision VisionParams
	// IMU is the error budget the IMU model switches on.
	IMU sensorerrors.Errors
	// TickPeriodQuantiles at TickPeriodLevels is the measured control
	// period (9,552 ticks over five rounds on 2026-09-15).
	TickPeriodQuantiles []float64
	TickPeriodLevels    []float64
}

// Shipped simulation.toml values, the fallback without a config root.
const (
	defaultInvalidRayRate  = 0.095
	defaultBandMinDeg      = 120.0
	defaultBandMaxDeg      = 160.0
	defaultBandDropoutRate = 0.689
	defaultSelfReturnM     = 0.0207
	defaultSelfReturnStdM  = 0.0052

	defaultR50M               = 1.1
	defaultFalloffM           = 0.15
	defaultFrameMissRate      = 0.3
	defaultConfidence         = 0.9
	defaultBearingScatterRad  = 0.116
	defaultColorFlipRate      = 0.051
	defaultStartPosErrorM     = 0.05
	defaultYawBiasRad         = 0.03
	defaultIMUDriftRadPerS    = 0.000145
	defaultGyroScaleError     = 0.005
	defaultIMUNoiseRad        = 0.005
	degreesPerHalfCircle      = 180.0
	defaultConfidenceQuantile = 0.958
)

// DefaultParams returns the shipped simulation.toml values.
func DefaultParams() Params {
	return Params{
		Lidar: LidarParams{
			InvalidRayRate:  defaultInvalidRayRate,
			BandMinRad:      defaultBandMinDeg * math.Pi / degreesPerHalfCircle,
			BandMaxRad:      defaultBandMaxDeg * math.Pi / degreesPerHalfCircle,
			BandDropoutRate: defaultBandDropoutRate,
			SelfReturnM:     defaultSelfReturnM,
			SelfReturnStdM:  defaultSelfReturnStdM,
		},
		Vision: VisionParams{
			RangeModel:          true,
			R50M:                defaultR50M,
			FalloffM:            defaultFalloffM,
			FrameMissRate:       defaultFrameMissRate,
			ConfidenceQuantiles: []float64{0.451, 0.515, 0.76, 0.917, defaultConfidenceQuantile},
			ConfidenceLevels:    []float64{0.0, 0.1, 0.5, 0.9, 1.0},
			FallbackConfidence:  defaultConfidence,
			BearingScatterRad:   defaultBearingScatterRad,
			ColorFlipRate:       defaultColorFlipRate,
		},
		IMU: sensorerrors.Errors{
			StartPosErrorM:  defaultStartPosErrorM,
			YawBiasRad:      defaultYawBiasRad,
			IMUDriftRadPerS: defaultIMUDriftRadPerS,
			GyroScaleError:  defaultGyroScaleError,
			IMUNoiseRad:     defaultIMUNoiseRad,
		},
		TickPeriodQuantiles: []float64{0.044, 0.05, 0.0633, 0.0795},
		TickPeriodLevels:    []float64{0.0, 0.5, 0.9, 0.99},
	}
}

// ParamsFor resolves the Params to run with: DefaultParams, overlaid with
// simulation.toml from <configRoot>/profile.DefaultSimulationTOMLPath when
// configRoot is non-empty and the load succeeds; otherwise the defaults,
// logged.
func ParamsFor(logger *slog.Logger, configRoot string) Params {
	p := DefaultParams()
	if configRoot == "" {
		return p
	}
	profile.Apply(logger, filepath.Join(configRoot, profile.DefaultSimulationTOMLPath), nil,
		func(sim simulation.NavigationSimulationSimulation) {
			p.Lidar = LidarParams{
				InvalidRayRate:  sim.LidarInvalidRayRate,
				BandMinRad:      sim.LidarOcclusionMinDeg * math.Pi / degreesPerHalfCircle,
				BandMaxRad:      sim.LidarOcclusionMaxDeg * math.Pi / degreesPerHalfCircle,
				BandDropoutRate: sim.LidarOcclusionDropoutRate,
				SelfReturnM:     sim.LidarOcclusionSelfReturnM,
				SelfReturnStdM:  sim.LidarOcclusionSelfReturnStdM,
			}
			p.Vision = VisionParams{
				RangeModel:          sim.VisionRangeModel,
				R50M:                sim.VisionDetectR50M,
				FalloffM:            sim.VisionDetectFalloffM,
				FrameMissRate:       sim.VisionFrameMissRate,
				ConfidenceQuantiles: sim.VisionConfidenceQuantiles,
				ConfidenceLevels:    sim.VisionConfidenceLevels,
				FallbackConfidence:  sim.DetectionConfidence,
				BearingScatterRad:   sim.VisionBearingScatterRad,
				ColorFlipRate:       sim.VisionColorFlipRate,
			}
			p.IMU = sensorerrors.Errors{
				StartPosErrorM:  sim.SensorStartPosErrorM,
				YawBiasRad:      sim.SensorYawBiasRad,
				IMUDriftRadPerS: sim.SensorImuDriftRadPerS,
				GyroScaleError:  sim.SensorGyroScaleError,
				IMUNoiseRad:     sim.SensorImuNoiseRad,
			}
			p.TickPeriodQuantiles = sim.SimTickPeriodQuantiles
			p.TickPeriodLevels = sim.SimTickPeriodLevels
		})
	return p
}
