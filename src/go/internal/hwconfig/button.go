//go:build linux

package hwconfig

import (
	"log/slog"
	"path/filepath"
	"time"

	genbutton "github.com/teamvoltimor/vtitan/src/go/internal/config/generated/hardware/button"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	driverbutton "github.com/teamvoltimor/vtitan/src/go/pkg/driver/button"
)

// Button resolves the Config to Connect with: driverbutton.DefaultGPIOChip,
// driverbutton.DefaultPollInterval, and DefaultThresholds, overlaid with
// genbutton.HardwareButtonGpio (from
// <configRoot>/profile.DefaultButtonGPIOTOMLPath) and
// genbutton.HardwareButtonButtonNode (from
// <configRoot>/profile.DefaultButtonNodeTOMLPath), both overlaid with the
// profiles named in profile.ActiveNames(), if configRoot is non-empty and
// loading succeeds; otherwise the literal defaults, logging why on
// failure. Each of the two files loads and falls back independently, since
// they're unrelated failure domains (driver wiring vs. node poll rate).
func Button(logger *slog.Logger, configRoot string) driverbutton.Config {
	cfg := driverbutton.Config{
		GPIOChip:     driverbutton.DefaultGPIOChip,
		PollInterval: driverbutton.DefaultPollInterval,
		Thresholds:   driverbutton.DefaultThresholds(),
	}
	if configRoot == "" {
		return cfg
	}

	gpioPath := filepath.Join(configRoot, profile.DefaultButtonGPIOTOMLPath)
	profile.Apply(logger, gpioPath, profile.ActiveNames(), func(gpioCfg genbutton.HardwareButtonGpio) {
		cfg.Line = gpioCfg.ButtonGpioPin
		cfg.PullUp = gpioCfg.Button.PullUp
		cfg.Thresholds = driverbutton.Thresholds{
			DebounceInterval:       time.Duration(float64(gpioCfg.Button.DebounceMs) * float64(time.Millisecond)),
			LongPressThreshold:     time.Duration(gpioCfg.Button.LongPressThresholdSec * float64(time.Second)),
			ShutdownPressThreshold: time.Duration(gpioCfg.Button.ShutdownPressThresholdSec * float64(time.Second)),
		}
	})

	nodePath := filepath.Join(configRoot, profile.DefaultButtonNodeTOMLPath)
	profile.Apply(logger, nodePath, profile.ActiveNames(), func(nodeCfg genbutton.HardwareButtonButtonNode) {
		if nodeCfg.PollHz > 0 {
			cfg.PollInterval = time.Duration(float64(time.Second) / nodeCfg.PollHz)
		}
	})

	return cfg
}
