package profile

// DefaultMotorsTOMLPath is
// platform/robot/config/hardware/motors/motors.toml, relative to the repo
// root.
const DefaultMotorsTOMLPath = "platform/robot/config/hardware/motors/motors.toml"

// MotorsConfig mirrors the subset of
// platform/robot/config/hardware/motors/motors.toml (src/hardware/motors/
// config.py) that robot-go currently consumes.
type MotorsConfig struct {
	Drive struct {
		// SpeedScale matches motors.toml's drive.speed_scale: motor_speed =
		// velocity_m_s * scale.
		SpeedScale float64 `mapstructure:"speed_scale"`
	} `mapstructure:"drive"`
}
