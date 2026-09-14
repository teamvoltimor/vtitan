package directionestimator

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/blind_nav"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/sensors"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
)

// ConfigFor resolves the Config to run with: DefaultConfig's literals,
// overlaid with blind_nav.NavigationBlindNavDirectionEstimator, sensors.NavigationSensorsLidarSectors,
// and blind_nav.NavigationBlindNavCorridorFollower (each loaded from
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

	profile.Apply(logger, filepath.Join(configRoot, profile.DefaultDirectionEstimatorTOMLPath), nil,
		func(de blind_nav.NavigationBlindNavDirectionEstimator) {
			cfg.AlignmentToleranceRad = de.AlignmentToleranceRad
			cfg.MaxInTrackRangeM = de.MaxInTrackRangeM
			cfg.PlausibleSpanThresholdM = de.PlausibleSpanThresholdM
			cfg.MinAsymmetryM = de.MinAsymmetryM
			cfg.MinVotes = de.MinVotes
		})

	profile.Apply(logger, filepath.Join(configRoot, profile.DefaultLidarSectorsTOMLPath), nil,
		func(ls sensors.NavigationSensorsLidarSectors) {
			cfg.DirectionArcHalfFovDeg = ls.DirectionArcHalfFovDeg
			cfg.MinValidRangeM = ls.MinValidRangeM
		})

	profile.Apply(logger, filepath.Join(configRoot, profile.DefaultCorridorFollowerTOMLPath), nil,
		func(cf blind_nav.NavigationBlindNavCorridorFollower) {
			cfg.MinForwardClearanceM = cf.MinForwardClearanceM
			cfg.TurnClearanceM = cf.TurnClearanceM
			cfg.BayWallClearanceM = cf.BayWallClearanceM
		})

	return cfg
}
