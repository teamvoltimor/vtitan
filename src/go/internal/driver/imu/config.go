package imu

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
)

// DefaultPort matches bno08x_uart_rvc.toml's default_port -- the fallback
// port used when MCP2221 VID/PID auto-detection isn't available (that
// auto-detection, find_mcp2221_port in the Python driver, isn't ported
// here) and no profile or flag overrides it.
const DefaultPort = "/dev/ttyACM0"

// ConfigFor resolves the Config to Connect with: DefaultPort and
// DefaultBaudRate, overlaid with profile.IMUUARTRVCConfig from
// <configRoot>/profile.DefaultIMUUARTRVCTOMLPath (overlaid with the
// profiles named in profile.ActiveNames()) if configRoot is non-empty and
// loading succeeds; otherwise the literal defaults, logging why on
// failure.
func ConfigFor(logger *slog.Logger, configRoot string) Config {
	cfg := Config{Port: DefaultPort, BaudRate: DefaultBaudRate}
	if configRoot == "" {
		return cfg
	}

	basePath := filepath.Join(configRoot, profile.DefaultIMUUARTRVCTOMLPath)
	loaded, err := profile.Load[profile.IMUUARTRVCConfig](basePath, profile.ActiveNames())
	if err != nil {
		logger.Warn("driver/imu: loading hardware profile, falling back to default serial config",
			"config_root", configRoot, "error", err)
		return cfg
	}

	cfg.Port = loaded.DefaultPort
	cfg.BaudRate = loaded.Baudrate
	return cfg
}
