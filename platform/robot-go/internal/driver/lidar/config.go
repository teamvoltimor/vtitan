package lidar

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/config/profile"
)

// DefaultPort matches lidar.toml's serial_port default -- the fallback
// used when no profile or flag overrides it.
const DefaultPort = "/dev/ttyUSB0"

// ConfigFor resolves the Config to Connect with: DefaultPort and
// DefaultBaudRate, overlaid with profile.LidarLaunchConfig from
// <configRoot>/profile.DefaultLidarLaunchTOMLPath (overlaid with the
// profiles named in profile.ActiveNames()) if configRoot is non-empty and
// loading succeeds; otherwise the literal defaults, logging why on
// failure.
func ConfigFor(logger *slog.Logger, configRoot string) Config {
	cfg := Config{Port: DefaultPort, BaudRate: DefaultBaudRate}
	if configRoot == "" {
		return cfg
	}

	basePath := filepath.Join(configRoot, profile.DefaultLidarLaunchTOMLPath)
	loaded, err := profile.Load[profile.LidarLaunchConfig](basePath, profile.ActiveNames())
	if err != nil {
		logger.Warn("driver/lidar: loading hardware profile, falling back to default serial config",
			"config_root", configRoot, "error", err)
		return cfg
	}

	cfg.Port = loaded.SerialPort
	cfg.BaudRate = loaded.SerialBaudrate
	return cfg
}
