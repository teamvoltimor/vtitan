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
