package directionestimator

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
)

// ConfigFor resolves the Config to run with: DefaultConfig's literals,
// overlaid with profile.DirectionEstimatorConfig, profile.LidarSectorsConfig,
// and profile.CorridorFollowerConfig (each loaded from
// <configRoot>/profile.DefaultXxxTOMLPath) if configRoot is non-empty and
// loading succeeds; otherwise, or on any load failure, the literal
// defaults, logging why. The three files load and fall back
// independently, since they're unrelated failure domains (an operator
// tuning direction_estimator.toml shouldn't be blocked by a typo in
// corridor_follower.toml).
func ConfigFor(logger *slog.Logger, configRoot string) Config {
	cfg := DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	dePath := filepath.Join(configRoot, profile.DefaultDirectionEstimatorTOMLPath)
	if de, err := profile.Load[profile.DirectionEstimatorConfig](dePath, nil); err != nil {
		logger.Warn(
			"directionestimator: loading direction_estimator.toml, falling back to defaults",
			"config_root",
			configRoot,
			"error",
			err,
		)
	} else {
		cfg.AlignmentToleranceRad = de.AlignmentToleranceRad
		cfg.MaxInTrackRangeM = de.MaxInTrackRangeM
		cfg.PlausibleSpanThresholdM = de.PlausibleSpanThresholdM
		cfg.MinAsymmetryM = de.MinAsymmetryM
		cfg.MinVotes = de.MinVotes
	}

	lsPath := filepath.Join(configRoot, profile.DefaultLidarSectorsTOMLPath)
	if ls, err := profile.Load[profile.LidarSectorsConfig](lsPath, nil); err != nil {
		logger.Warn("directionestimator: loading lidar_sectors.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
	} else {
		cfg.DirectionArcHalfFovDeg = ls.DirectionArcHalfFovDeg
		cfg.MinValidRangeM = ls.MinValidRangeM
	}

	cfPath := filepath.Join(configRoot, profile.DefaultCorridorFollowerTOMLPath)
	if cf, err := profile.Load[profile.CorridorFollowerConfig](cfPath, nil); err != nil {
		logger.Warn("directionestimator: loading corridor_follower.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
	} else {
		cfg.MinForwardClearanceM = cf.MinForwardClearanceM
		cfg.TurnClearanceM = cf.TurnClearanceM
		cfg.BayWallClearanceM = cf.BayWallClearanceM
	}

	return cfg
}
