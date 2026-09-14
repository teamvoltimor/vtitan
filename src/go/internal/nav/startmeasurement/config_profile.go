package startmeasurement

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/sensors"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
)

// ConfigFor resolves the Config to run with: DefaultConfig's literals,
// overlaid with sensors.NavigationSensorsStartMeasurement (from
// <configRoot>/profile.DefaultStartMeasurementTOMLPath),
// generated.TrackConfig's track geometry and profile.RobotConfig's LIDAR
// range, when configRoot is non-empty and each load succeeds; otherwise, or
// on a load failure, that piece keeps its literal default, logged.
func ConfigFor(logger *slog.Logger, configRoot string) Config {
	cfg := DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	profile.Apply(logger, filepath.Join(configRoot, profile.DefaultStartMeasurementTOMLPath), nil,
		func(loaded sensors.NavigationSensorsStartMeasurement) {
			cfg.RayHalfWidthDeg = loaded.RayHalfWidthDeg
			cfg.ClosingToleranceM = loaded.ClosingToleranceM
		})

	profile.Apply(logger, filepath.Join(configRoot, profile.DefaultTrackTOMLPath), nil,
		func(loaded generated.TrackConfig) {
			cfg.TrackMaxCoordM = loaded.Track.MaxCoord
		})

	robotPath := filepath.Join(configRoot, profile.DefaultRobotTOMLPath)
	if loaded, err := profile.LoadRobotValues(robotPath, nil); err != nil {
		logger.Warn("startmeasurement: loading robot.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
	} else {
		cfg.LidarMinRangeM = loaded.Lidar.MinRange
		cfg.LidarMaxRangeM = loaded.Lidar.MaxRange
	}

	return cfg
}
