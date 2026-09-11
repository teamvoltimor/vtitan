package harness

import (
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/localization"
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/sim/sensorerrors"
)

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
	// LidarMountXOffsetM is how far forward of the chassis centre the sensor
	// sits, from robot.toml's [lidar] mount_x_offset. Rays are cast from
	// THERE, not from the body origin, matching
	// SimulatedHardwareGateway's own Pose.sensor_origin call.
	LidarMountXOffsetM float64
	// LidarNoiseStd is the Gaussian range noise stddev (meters). Mirrors
	// RobotSpecs.LIDAR_NOISE_STDDEV; that constant is not ported to Go yet
	// (see plan §2), so it is supplied here rather than read from a spec.
	LidarNoiseStd float64
	// InvalidRayRate is the fraction of rays dropped to max-range (Slamtec
	// no-return model). Mirrors simulation.LIDAR_INVALID_RAY_RATE; unported
	// (plan §2), defaulted to 0.01.
	InvalidRayRate float64
	// Localize runs the LIDAR scan-matcher and reports its estimate as the
	// robot's pose, instead of handing the navigator ground truth.
	//
	// Off by default here, which is NOT the Python default: ScenarioSimulator
	// takes use_lidar_localization=True and forces it on for blind. True is
	// the HARDER condition (the robot navigates on an estimate that can
	// drift) and is what the real robot does, so a cross-stack comparison
	// against the Python oracle needs this ON to be like-for-like.
	Localize bool
	// LocalizationConfig parameterizes that scan-matcher. Nil takes
	// localization.DefaultConfig(); prefer localization.ConfigFor so a run
	// reads the shipped localization.toml and robot.toml [lidar] geometry.
	LocalizationConfig *localization.Config
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
	// SensorErrors is what the robot may be wrong about regarding ITSELF:
	// where it was placed and which way it thinks it points. The zero value
	// is a perfect robot, which is what every corpus number in this repo
	// was measured against -- and what the Python default is too, since
	// every SensorErrors field defaults to 0.0 there. Switching any of it
	// on is an explicit A/B.
	SensorErrors sensorerrors.Errors

	// DetectionConfidence is the fixed confidence internal/sim/visionsim
	// reports for every emulated sign detection. Mirrors
	// simulation.toml's detection_confidence; unported (plan §2), defaulted
	// to 0.9.
	DetectionConfidence float64

	// NoProgressWindowS / NoProgressDisplacementM end a run early, scored as
	// stuck, when the chassis fails to move NoProgressDisplacementM in
	// NoProgressWindowS. They mirror SimulationParams.NO_PROGRESS_WINDOW_S
	// (30.0) and NO_PROGRESS_DISPLACEMENT_M (0.08).
	//
	// These were hardcoded at 2.0 s / 0.05 m until 2026-09-06 -- a FIFTEENFOLD
	// shorter window than Python's. An escape manoeuvre reverses, reorients
	// and re-approaches, which cannot finish inside two seconds, so the runner
	// killed the run mid-escape: 142/256 blind runs ended stuck, 138 of them
	// in a CORNER, at the LIDAR's 0.045 m floor, with ZERO collisions, having
	// travelled a median 4.63 m. Python escapes 33 times a lap on the same
	// corpus and recovers.
	NoProgressWindowS       float64
	NoProgressDisplacementM float64

	// StartCollisionWindowS / StartCollisionGraceS govern the same forgiveness
	// the native runner's contactTracker applies to a legal starting pose the
	// track generator allows a few mm from a wall: contact that BEGINS within
	// StartCollisionWindowS of run start is forgiven for up to
	// StartCollisionGraceS before it counts as a real (terminal) crash;
	// contact beginning later, or lasting longer, is terminal on the first
	// tick. Mirrors SimulationParams.START_COLLISION_WINDOW_S (2.0) /
	// START_COLLISION_GRACE_S (15.0); unported (plan §2), defaulted to the
	// same values.
	//
	// Before this existed the native runner had NO grace at all: any tick of
	// contact with a forbidden surface ended the run instantly, so a legal
	// start pose a few mm from the OUTER wall was scored as an immediate
	// collision. Measured on the balanced-128 Open corpus (seed 2026): 17/128
	// collisions native vs 0/128 on the frozen Python oracle running the same
	// scenarios.
	StartCollisionWindowS float64
	StartCollisionGraceS  float64
}

// DefaultConfig returns the all-default Config: 20 Hz control, 360-ray LIDAR
// at 0.15–8 m, no noise, no dropout, perfect odometry, 3 m mat, 0.30x0.194 m
// chassis. The noise/dropout/margin values are parity defaults for the
// unported simulation.toml fields (plan §2), not measured values.
func DefaultConfig() Config {
	return Config{
		ControlHz:               20.0,
		NoProgressWindowS:       30.0,
		NoProgressDisplacementM: 0.08,
		LidarHz:                 0.0,
		LidarSamples:            360,
		LidarMinRangeM:          0.15,
		LidarMaxRangeM:          8.0,
		LidarNoiseStd:           0.03,
		InvalidRayRate:          0.01,
		Localize:                false,
		CollisionMarginM:        0.0,
		TrackMaxCoordM:          3.0,
		ChassisLengthM:          0.30,
		ChassisWidthM:           0.194,
		DetectionConfidence:     0.9,
		StartCollisionWindowS:   2.0,
		StartCollisionGraceS:    15.0,
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
func (c Config) ControlDt() float64 {
	return c.dt()
}

// lidarPeriodS returns the sweep period, or 0 (always fresh) when LidarHz<=0.
func (c Config) lidarPeriodS() float64 {
	if c.LidarHz <= 0 {
		return 0.0
	}
	return 1.0 / c.LidarHz
}

// ensure math import is used (WrapAngle helper references would live elsewhere).
var _ = math.Pi
