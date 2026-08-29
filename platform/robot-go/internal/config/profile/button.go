package profile

// DefaultButtonGPIOTOMLPath is
// platform/robot/config/hardware/button/gpio.toml, relative to the repo
// root.
const DefaultButtonGPIOTOMLPath = "platform/robot/config/hardware/button/gpio.toml"

// DefaultButtonNodeTOMLPath is
// platform/robot/config/hardware/button/button_node.toml, relative to the
// repo root -- the ROS2 node's own poll cadence, a separate file from
// gpio.toml's driver-level debounce/threshold config.
const DefaultButtonNodeTOMLPath = "platform/robot/config/hardware/button/button_node.toml"

// ButtonSection mirrors gpio.toml's [button] section.
type ButtonSection struct {
	// PullUp matches internal/driver/button.Config.PullUp.
	PullUp bool `mapstructure:"pull_up"`
	// DebounceMs matches internal/driver/button.Thresholds.DebounceInterval
	// (milliseconds; caller converts to time.Duration).
	DebounceMs float64 `mapstructure:"debounce_ms"`
	// LongPressThresholdSec matches
	// internal/driver/button.Thresholds.LongPressThreshold (seconds).
	LongPressThresholdSec float64 `mapstructure:"long_press_threshold_sec"`
	// ShutdownPressThresholdSec matches
	// internal/driver/button.Thresholds.ShutdownPressThreshold (seconds).
	ShutdownPressThresholdSec float64 `mapstructure:"shutdown_press_threshold_sec"`
}

// ButtonGPIOConfig mirrors gpio.toml (src/hardware/button/gpio/driver.py).
// button_gpio_pin is a top-level key (not sectioned); [button] holds
// debounce/threshold tuning.
type ButtonGPIOConfig struct {
	// ButtonGPIOPin matches internal/driver/button.Config.Line.
	ButtonGPIOPin int           `mapstructure:"button_gpio_pin"`
	Button        ButtonSection `mapstructure:"button"`
}

// ButtonNodeConfig mirrors button_node.toml
// (vtitan_drivers/vtitan_drivers/button_node.py's own poll cadence).
type ButtonNodeConfig struct {
	// PollHz matches internal/driver/button.Config.PollInterval (Hz;
	// caller converts to time.Duration as time.Second / PollHz).
	PollHz float64 `mapstructure:"poll_hz"`
}
