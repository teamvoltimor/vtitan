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
// named in profile.ActiveNames(); otherwise, or if loading fails, it
// returns DefaultConfig unchanged (logging why on failure). GPIOChip and
// Invert have no bts7960.toml counterpart (see motors.HardwareMotorsBts7960's doc
// comment) and are always DefaultConfig's/the caller's own value.
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
	return cfg
}
