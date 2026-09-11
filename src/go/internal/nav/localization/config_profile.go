package localization

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
)

// ConfigFor resolves the Config to run with: DefaultConfig's literals overlaid
// with localization.toml (the search/plausibility knobs) and robot.toml's
// [lidar] section (where the sensor sits and what it can see), each loaded
// from <configRoot>, if configRoot is non-empty and loading succeeds;
// otherwise the literal defaults for that file, logging why.
//
// The LIDAR geometry comes from robot.toml rather than being restated here
// because the search predicts ranges from where the SENSOR is: a localizer
// disagreeing with the driver about the mount offset scan-matches against a
// sensor that does not exist.
func ConfigFor(logger *slog.Logger, configRoot string) Config {
	cfg := DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	locPath := filepath.Join(configRoot, profile.DefaultLocalizationTOMLPath)
	if lc, err := profile.Load[profile.LocalizationConfig](locPath, nil); err != nil {
		logger.Warn("localization: loading localization.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
	} else {
		cfg.SearchRadiusM = lc.SearchRadiusM
		cfg.Passes = lc.Passes
		cfg.GridPoints = lc.GridPoints
		cfg.ResidualClipM = lc.ResidualClipM
		cfg.MaxSpeedMPS = lc.MaxSpeedMPS
		cfg.JumpConfirmToleranceM = lc.JumpConfirmToleranceM
		cfg.RelocalizeCostThreshold = lc.RelocalizeCostThreshold
		cfg.RelocalizeAfterScans = lc.RelocalizeAfterScans
		cfg.RelocalizeGridStepM = lc.RelocalizeGridStepM
		cfg.RelocalizeAcceptRatio = lc.RelocalizeAcceptRatio
	}

	robotPath := filepath.Join(configRoot, profile.DefaultRobotTOMLPath)
	if rc, err := profile.Load[profile.RobotConfig](robotPath, nil); err != nil {
		logger.Warn("localization: loading robot.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
	} else {
		cfg.LidarMountXOffsetM = rc.Lidar.MountXOffset
		cfg.LidarMinRangeM = rc.Lidar.MinRange
		cfg.LidarMaxRangeM = rc.Lidar.MaxRange
	}

	return cfg
}
