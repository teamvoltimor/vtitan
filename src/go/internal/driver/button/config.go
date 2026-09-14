//go:build linux

package button

import (
	"log/slog"
	"path/filepath"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/hardware/button"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
)

// ConfigFor resolves the Config to Connect with: DefaultGPIOChip,
// DefaultPollInterval, and DefaultThresholds, overlaid with
// button.HardwareButtonGpio (from
// <configRoot>/profile.DefaultButtonGPIOTOMLPath) and
// button.HardwareButtonButtonNode (from
// <configRoot>/profile.DefaultButtonNodeTOMLPath), both overlaid with the
// profiles named in profile.ActiveNames(), if configRoot is non-empty and
// loading succeeds; otherwise the literal defaults, logging why on
// failure. Each of the two files loads and falls back independently, since
// they're unrelated failure domains (driver wiring vs. node poll rate).
func ConfigFor(logger *slog.Logger, configRoot string) Config {
	cfg := Config{
		GPIOChip:     DefaultGPIOChip,
		PollInterval: DefaultPollInterval,
		Thresholds:   DefaultThresholds(),
	}
	if configRoot == "" {
		return cfg
	}

	gpioPath := filepath.Join(configRoot, profile.DefaultButtonGPIOTOMLPath)
	profile.Apply(logger, gpioPath, profile.ActiveNames(), func(gpioCfg button.HardwareButtonGpio) {
		cfg.Line = gpioCfg.ButtonGpioPin
		cfg.PullUp = gpioCfg.Button.PullUp
		cfg.Thresholds = Thresholds{
			DebounceInterval:       time.Duration(float64(gpioCfg.Button.DebounceMs) * float64(time.Millisecond)),
			LongPressThreshold:     time.Duration(gpioCfg.Button.LongPressThresholdSec * float64(time.Second)),
			ShutdownPressThreshold: time.Duration(gpioCfg.Button.ShutdownPressThresholdSec * float64(time.Second)),
		}
	})

	nodePath := filepath.Join(configRoot, profile.DefaultButtonNodeTOMLPath)
	profile.Apply(logger, nodePath, profile.ActiveNames(), func(nodeCfg button.HardwareButtonButtonNode) {
		if nodeCfg.PollHz > 0 {
			cfg.PollInterval = time.Duration(float64(time.Second) / nodeCfg.PollHz)
		}
	})

	return cfg
}
