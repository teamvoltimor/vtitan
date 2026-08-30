package kinematics

// Config parameterizes AckermannKinematics with the one value this
// package's own tuning group actually owns: the servo slew-rate limit.
// Wheelbase, max steer angle, max accel, max speed, and rear-steer ratio
// are robot-physical constants that come from profile.RobotConfig
// (robot.toml) instead -- see Params, which takes those directly rather
// than this package re-deriving them.
type Config struct {
	// MaxSteeringRateRadPerS matches
	// PurePursuitParams.MAX_STEERING_RATE: the servo's slew rate limit
	// (rad/s).
	MaxSteeringRateRadPerS float64
}

// DefaultMaxSteeringRateRadPerS matches
// shared.config.navigation_tuning.motion.PurePursuitParams.MAX_STEERING_RATE's
// Pydantic field default. The shipped pursuit.toml currently overrides
// this to 1.2 (a provisional hardware-tuned value) -- ConfigFor prefers
// that TOML value when available, and this literal is only the fallback.
const DefaultMaxSteeringRateRadPerS = 2.0

// DefaultConfig returns the Config matching the Python tuning default.
func DefaultConfig() Config {
	return Config{MaxSteeringRateRadPerS: DefaultMaxSteeringRateRadPerS}
}

// DefaultParams returns the Ackermann integrator parameters for the shipped
// robot, matching the RobotSpecs/RobotDrivetrain constants in robot.toml
// (wheelbase 0.20 m, max steer 1.2252 rad, counter-phase rear steer -1.0,
// yaw gain 0.55). This is the single source of truth for the Ackermann
// parameter block previously hand-assembled at every call site; the
// SubstepCount default is applied by NewAckermannKinematics when Substeps<=0.
func DefaultParams() Params {
	return Params{
		WheelbaseM:          0.20,
		MaxSteerRad:         1.2252,
		MaxSteerRateRadPerS: DefaultMaxSteeringRateRadPerS,
		MaxAccelMPS2:        0.5,
		MaxSpeedMPS:         1.0,
		RearSteerRatio:      -1.0,
		SpeedTauS:           0.1,
		YawGain:             0.55,
		Substeps:            DefaultSubsteps,
	}
}
