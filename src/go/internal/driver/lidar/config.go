package lidar

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
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
//
// The mount correction (Config.Inverted, Config.YawOffsetDeg) comes from a
// second file -- robot.toml's [lidar] section, the single source of truth
// for the physical mount -- because lidar.toml describes the serial link,
// not where the sensor is bolted. Resolving it here means the driver emits
// robot-frame bearings and no downstream consumer applies its own
// correction; publishing raw bearings and leaving each consumer to correct
// them is what let the nav gateway apply nothing at all while telemetry
// applied a rotation that the 2026-08-31 bearing test had already refuted.
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
	cfg.Inverted, cfg.YawOffsetDeg = mountCorrectionFor(logger, configRoot)
	return cfg
}

// mountCorrectionFor resolves robot.toml's [lidar].inverted and
// [lidar].mount_yaw_offset_deg. A load failure yields the uncorrected
// (false, 0) pair and a warning rather than an error: an unreadable
// robot.toml must not stop the LIDAR from streaming, and a wrong frame is
// visible in the scan where a dead sensor is not.
//
// profile.Load, not profile.LoadRobotConfig: the latter also enforces the
// steering and drivetrain keys that robot.toml deliberately leaves to an
// active hardware profile, so using it would make the mount correction
// silently vanish whenever VTITAN_HARDWARE_PROFILE is unset -- a servo spec
// the LIDAR does not read deciding whether the LIDAR points forward. Both
// fields live in the base file, so no profile is required to reach them.
func mountCorrectionFor(logger *slog.Logger, configRoot string) (bool, float64) {
	robotPath := filepath.Join(configRoot, profile.DefaultRobotTOMLPath)
	robotCfg, err := profile.Load[profile.RobotConfig](robotPath, profile.ActiveNames())
	if err != nil {
		logger.Warn("driver/lidar: loading robot.toml, publishing uncorrected mount frame",
			"config_root", configRoot, "error", err)
		return false, 0
	}
	return robotCfg.Lidar.Inverted, robotCfg.Lidar.MountYawOffsetDeg
}
