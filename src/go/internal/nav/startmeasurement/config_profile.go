package startmeasurement

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/config/profile"
)

// ConfigFor resolves the Config to run with: DefaultConfig's literals,
// overlaid with profile.StartMeasurementConfig (from
// <configRoot>/profile.DefaultStartMeasurementTOMLPath),
// profile.TrackConfig's track geometry and profile.RobotConfig's LIDAR
// range, when configRoot is non-empty and each load succeeds; otherwise, or
// on a load failure, that piece keeps its literal default, logged.
func ConfigFor(logger *slog.Logger, configRoot string) Config {
	cfg := DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	sensorPath := filepath.Join(configRoot, profile.DefaultStartMeasurementTOMLPath)
	if loaded, err := profile.Load[profile.StartMeasurementConfig](sensorPath, nil); err != nil {
		logger.Warn("startmeasurement: loading start_measurement.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
	} else {
		cfg.RayHalfWidthDeg = loaded.RayHalfWidthDeg
		cfg.ClosingToleranceM = loaded.ClosingToleranceM
	}

	trackPath := filepath.Join(configRoot, profile.DefaultTrackTOMLPath)
	if loaded, err := profile.Load[profile.TrackConfig](trackPath, nil); err != nil {
		logger.Warn("startmeasurement: loading track.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
	} else {
		cfg.TrackMaxCoordM = loaded.Track.MaxCoord
	}

	robotPath := filepath.Join(configRoot, profile.DefaultRobotTOMLPath)
	if loaded, err := profile.Load[profile.RobotConfig](robotPath, nil); err != nil {
		logger.Warn("startmeasurement: loading robot.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
	} else {
		cfg.LidarMinRangeM = loaded.Lidar.MinRange
		cfg.LidarMaxRangeM = loaded.Lidar.MaxRange
	}

	return cfg
}
