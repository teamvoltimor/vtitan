package harness

import "math"

// Config configures a SimHardwareGateway, the Go-native replacement for the
// Python SimulatedHardwareGateway. Every field has a sane default so a
// caller can construct an all-zero Config and still get a working (if
// noiseless, non-localizing) simulation; the native runner overrides the
// ones it sources from scenario metadata / tuning.
type Config struct {
	// ControlHz is the control loop rate (ticks/second). dt = 1/ControlHz.
	ControlHz float64
	// LidarHz is the LIDAR sweep rate (sweeps/second). Zero means "regenerate
	// the scan every control tick" (the old always-fresh behaviour).
	LidarHz float64
	// LidarSamples is the number of rays per sweep.
	LidarSamples int
	// LidarMinRangeM / LidarMaxRangeM are the sensor's floor/ceiling.
	LidarMinRangeM float64
	LidarMaxRangeM float64
	// LidarNoiseStd is the Gaussian range noise stddev (meters). Mirrors
	// RobotSpecs.LIDAR_NOISE_STDDEV; that constant is not ported to Go yet
	// (see plan §2), so it is supplied here rather than read from a spec.
	LidarNoiseStd float64
	// InvalidRayRate is the fraction of rays dropped to max-range (Slamtec
	// no-return model). Mirrors simulation.LIDAR_INVALID_RAY_RATE; unported
	// (plan §2), defaulted to 0.01.
	InvalidRayRate float64
	// Localize enables LIDAR-matched pose estimation. Off by default so the
	// navigator sees perfect odometry (ground truth), isolating control
	// behaviour from state-estimation error, exactly as the Python gateway's
	// localize=False arm. wall_heading localization is NOT implemented yet
	// (see gateway.go TODO), so Localize=true is currently unsupported and
	// falls back to ground truth.
	Localize bool
	// CollisionMarginM is the keep-out margin handed to collision.TrackModel.
	// Mirrors simulation.COLLISION_MARGIN_M; unported (plan §2), defaulted 0.0.
	CollisionMarginM float64
	// TrackMaxCoordM is the mat's outer-boundary coordinate. Mirrors
	// track.toml's max_coord; unported here, defaulted to 3.0.
	TrackMaxCoordM float64
	// ChassisLengthM / ChassisWidthM size the footprint passed to the
	// collision check. Mirrors robot.toml; unported (plan §2), defaulted to
	// the known WRO chassis (0.30 x 0.194).
	ChassisLengthM float64
	ChassisWidthM  float64
}

// DefaultConfig returns the all-default Config: 20 Hz control, 360-ray LIDAR
// at 0.15–8 m, no noise, no dropout, perfect odometry, 3 m mat, 0.30x0.194 m
// chassis. The noise/dropout/margin values are parity defaults for the
// unported simulation.toml fields (plan §2), not measured values.
func DefaultConfig() Config {
	return Config{
		ControlHz:       20.0,
		LidarHz:         0.0,
		LidarSamples:    360,
		LidarMinRangeM:  0.15,
		LidarMaxRangeM:  8.0,
		LidarNoiseStd:   0.03,
		InvalidRayRate:  0.01,
		Localize:        false,
		CollisionMarginM: 0.0,
		TrackMaxCoordM:  3.0,
		ChassisLengthM:  0.30,
		ChassisWidthM:   0.194,
	}
}

// dt returns the control interval (seconds).
func (c Config) dt() float64 {
	if c.ControlHz <= 0 {
		return 1.0 / 20.0
	}
	return 1.0 / c.ControlHz
}

// ControlDt returns the control interval (seconds), exposed for callers that
// drive the loop (e.g. the scenario native runner).
func (c Config) ControlDt() float64 { return c.dt() }

// lidarPeriodS returns the sweep period, or 0 (always fresh) when LidarHz<=0.
func (c Config) lidarPeriodS() float64 {
	if c.LidarHz <= 0 {
		return 0.0
	}
	return 1.0 / c.LidarHz
}

// ensure math import is used (WrapAngle helper references would live elsewhere).
var _ = math.Pi
