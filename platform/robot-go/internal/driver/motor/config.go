//go:build linux

package motor

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/config/profile"
)

// ConfigFor resolves the Config to Connect with: if configRoot is
// non-empty, it starts from DefaultConfig and overlays every field
// profile.BTS7960Config carries, loaded from
// <configRoot>/profile.DefaultBTS7960TOMLPath overlaid with the profiles
// named in profile.ActiveNames(); otherwise, or if loading fails, it
// returns DefaultConfig unchanged (logging why on failure). GPIOChip and
// Invert have no bts7960.toml counterpart (see profile.BTS7960Config's doc
// comment) and are always DefaultConfig's/the caller's own value.
func ConfigFor(logger *slog.Logger, configRoot string) Config {
	cfg := DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	basePath := filepath.Join(configRoot, profile.DefaultBTS7960TOMLPath)
	loaded, err := profile.Load[profile.BTS7960Config](basePath, profile.ActiveNames())
	if err != nil {
		logger.Warn("driver/motor: loading hardware profile, falling back to default wiring config",
			"config_root", configRoot, "error", err)
		return cfg
	}

	cfg.PWMChip = loaded.PWMChip
	cfg.PWMChannel = loaded.PWMChannel
	cfg.FrequencyHz = loaded.FrequencyHz
	cfg.ReversePWMLine = loaded.ReversePWMPin
	cfg.REnLine = loaded.REnPin
	cfg.LEnLine = loaded.LEnPin
	return cfg
}
