//go:build linux

package hwconfig

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/hardware/motors"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	drivermotor "github.com/teamvoltimor/vtitan/src/go/pkg/driver/motor"
)

// Motor resolves the Config to Connect with: if configRoot is
// non-empty, it starts from DefaultConfig and overlays every field
// motors.HardwareMotorsBts7960 carries, loaded from
// <configRoot>/profile.DefaultBTS7960TOMLPath overlaid with the profiles
// named in profile.ActiveNames(), plus Invert from motors.toml's
// drive.reversed (the source the Zero's removed --motor-invert flag and the
// Pi 5's removed --pico-motor-invert flag used to set); otherwise, or if
// loading fails, it returns DefaultConfig unchanged (logging why on
// failure). GPIOChip has no bts7960.toml counterpart (see
// motors.HardwareMotorsBts7960's doc comment) and is always DefaultConfig's
// own value.
func Motor(logger *slog.Logger, configRoot string) drivermotor.Config {
	cfg := drivermotor.DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	basePath := filepath.Join(configRoot, profile.DefaultBTS7960TOMLPath)
	profile.Apply(logger, basePath, profile.ActiveNames(), func(loaded motors.HardwareMotorsBts7960) {
		cfg.PWMChip = loaded.Pwmchip
		cfg.PWMChannel = loaded.PwmChannel
		cfg.FrequencyHz = loaded.FrequencyHz
		cfg.ReversePWMLine = loaded.ReversePwmPin
		cfg.REnLine = loaded.REnPin
		cfg.LEnLine = loaded.LEnPin
	})

	motorsPath := filepath.Join(configRoot, profile.DefaultMotorsTOMLPath)
	profile.Apply(logger, motorsPath, profile.ActiveNames(), func(loaded motors.HardwareMotorsMotors) {
		cfg.Invert = loaded.Drive.Reversed
	})
	return cfg
}
