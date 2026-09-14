package imu

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/hardware/imu"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
)

// DefaultPort matches bno08x_uart_rvc.toml's default_port -- the fallback
// port used when MCP2221 VID/PID auto-detection isn't available (that
// auto-detection, find_mcp2221_port in the Python driver, isn't ported
// here) and no profile or flag overrides it.
const DefaultPort = "/dev/ttyACM0"

// ConfigFor resolves the Config to Connect with: DefaultPort and
// DefaultBaudRate, overlaid with imu.HardwareImuBno08XUartRvc from
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
	profile.Apply(logger, basePath, profile.ActiveNames(), func(loaded imu.HardwareImuBno08XUartRvc) {
		cfg.Port = loaded.DefaultPort
		cfg.BaudRate = loaded.Baudrate
	})
	return cfg
}
