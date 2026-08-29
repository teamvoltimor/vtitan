package diag

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/config/profile"
)

// LidarYawOffsetRadFor resolves the raw-scan-bearing yaw offset (see
// Config.LidarYawOffsetRad's doc comment): if configRoot is non-empty, it
// loads profile.RobotConfig from
// <configRoot>/profile.DefaultRobotTOMLPath overlaid with the profiles
// named in profile.ActiveNames() and returns its derived
// LidarYawOffsetRad(); otherwise, or if loading fails, it falls back to 0
// and logs why.
func LidarYawOffsetRadFor(logger *slog.Logger, configRoot string) float64 {
	if configRoot == "" {
		return 0
	}

	basePath := filepath.Join(configRoot, profile.DefaultRobotTOMLPath)
	cfg, err := profile.Load[profile.RobotConfig](basePath, profile.ActiveNames())
	if err != nil {
		logger.Warn("diag: loading hardware profile, falling back to zero LIDAR yaw offset",
			"config_root", configRoot, "error", err)
		return 0
	}
	return cfg.LidarYawOffsetRad()
}
