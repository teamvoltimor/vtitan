package ssd1306

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/hardware/display"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
)

// ConfigFor resolves the Config to Connect with: if configRoot is
// non-empty, it loads display.HardwareDisplaySsd1306 from
// <configRoot>/profile.DefaultSSD1306TOMLPath (overlaid with the profiles
// named in profile.ActiveNames()); otherwise, or if loading or parsing its
// hex I2CAddressHex fails, it returns DefaultConfig unchanged, logging why.
func ConfigFor(logger *slog.Logger, configRoot string) Config {
	cfg := DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	basePath := filepath.Join(configRoot, profile.DefaultSSD1306TOMLPath)
	loaded, err := profile.Load[display.HardwareDisplaySsd1306](basePath, profile.ActiveNames())
	if err != nil {
		logger.Warn(
			"driver/display/ssd1306: loading hardware profile, falling back to default config",
			"config_root",
			configRoot,
			"error",
			err,
		)
		return cfg
	}

	addr, err := profile.ParseI2CAddress(loaded)
	if err != nil {
		logger.Warn(
			"driver/display/ssd1306: parsing hardware profile's i2c_address, falling back to default config",
			"config_root",
			configRoot,
			"error",
			err,
		)
		return cfg
	}

	cfg.Width = loaded.Width
	cfg.Height = loaded.Height
	cfg.I2CAddress = addr
	cfg.I2CBus = loaded.I2CBus
	return cfg
}
