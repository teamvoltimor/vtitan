package hwconfig

import (
	"log/slog"
	"path/filepath"

	genimu "github.com/teamvoltimor/vtitan/src/go/internal/config/generated/hardware/imu"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	driverimu "github.com/teamvoltimor/vtitan/src/go/pkg/driver/imu"
)

// IMU resolves the Config to Connect with: DefaultPort and
// DefaultBaudRate, overlaid with genimu.HardwareImuBno08XUartRvc from
// <configRoot>/profile.DefaultIMUUARTRVCTOMLPath (overlaid with the
// profiles named in profile.ActiveNames()) if configRoot is non-empty and
// loading succeeds; otherwise the literal defaults, logging why on
// failure.
func IMU(logger *slog.Logger, configRoot string) driverimu.Config {
	cfg := driverimu.Config{Port: driverimu.DefaultPort, BaudRate: driverimu.DefaultBaudRate}
	if configRoot == "" {
		return cfg
	}

	basePath := filepath.Join(configRoot, profile.DefaultIMUUARTRVCTOMLPath)
	profile.Apply(logger, basePath, profile.ActiveNames(), func(loaded genimu.HardwareImuBno08XUartRvc) {
		cfg.Port = loaded.DefaultPort
		cfg.BaudRate = loaded.Baudrate
	})
	return cfg
}
