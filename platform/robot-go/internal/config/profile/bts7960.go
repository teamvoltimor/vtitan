package profile

// BTS7960Config mirrors bts7960.toml (src/hardware/motors/bts7960/config.py),
// the BTS7960/IBT-2 drive H-bridge's PWM/GPIO wiring. A flat top-level
// table, not sectioned. ForwardPWMPin is documentation of which physical
// pin PWMChip/PWMChannel maps to on this board -- internal/driver/motor
// addresses RPWM via PWMChip/PWMChannel, not a raw GPIO line, so it has no
// Go counterpart.
type BTS7960Config struct {
	PWMChip       int `mapstructure:"pwmchip"`
	PWMChannel    int `mapstructure:"pwm_channel"`
	FrequencyHz   int `mapstructure:"frequency_hz"`
	ForwardPWMPin int `mapstructure:"forward_pwm_pin"`
	// ReversePWMPin matches internal/driver/motor.Config.ReversePWMLine.
	ReversePWMPin int `mapstructure:"reverse_pwm_pin"`
	// REnPin matches internal/driver/motor.Config.REnLine.
	REnPin int `mapstructure:"r_en_pin"`
	// LEnPin matches internal/driver/motor.Config.LEnLine.
	LEnPin int `mapstructure:"l_en_pin"`
}

// DefaultBTS7960TOMLPath is
// platform/config/hardware/motors/bts7960.toml, relative to the repo
// root.
const DefaultBTS7960TOMLPath = "platform/config/hardware/motors/bts7960.toml"
