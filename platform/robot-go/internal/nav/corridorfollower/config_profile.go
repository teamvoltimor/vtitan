package corridorfollower

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/config/profile"
)

// ConfigFor resolves the Config to run with: DefaultConfig's literals,
// overlaid with the shipped TOML if configRoot is non-empty and loading
// succeeds; otherwise, or on any load failure, the literal defaults, logging
// why.
//
// This package had no loader at all until now, so every constant below read
// a Go literal and no edit to corridor_follower.toml reached the running
// follower -- including steer_cap_from_commit_distance, which had no Go
// field either. The literals happened to agree with the shipped values, so
// the omission was latent rather than visible: it would have surfaced as the
// first tuning change that silently did nothing.
//
// Each file loads and falls back independently, since they are unrelated
// failure domains: an operator tuning corridor_follower.toml should not be
// blocked by a typo in lidar_sectors.toml.
//
// The bay-exit fields are not overlaid here -- they belong to
// internal/nav/bayexit's own TOML and loader, and are mirrored on this
// Config only because FollowCorridor's callers hand it one config.
func ConfigFor(
	logger *slog.Logger,
	configRoot string,
	hardwareProfileNames []string,
) Config {
	cfg := DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	cfPath := filepath.Join(configRoot, profile.DefaultCorridorFollowerTOMLPath)
	if cf, err := profile.LoadWithDefaults[profile.CorridorFollowerConfig](
		cfPath, nil, profile.CorridorFollowerDefaults(),
	); err != nil {
		logger.Warn("corridorfollower: loading corridor_follower.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
	} else {
		cfg.TurnClearanceM = cf.TurnClearanceM
		cfg.NarrowTurnClearanceM = cf.NarrowTurnClearanceM
		cfg.CenteringGainDegPerM = cf.CenteringGainDegPerM
		cfg.HeadingGain = cf.HeadingGain
		cfg.MaxCenteringSteerDeg = cf.MaxCenteringSteerDeg
		cfg.MaxCornerSteerDeg = cf.MaxCornerSteerDeg
		cfg.SteerCapFromCommitDistance = cf.SteerCapFromCommitDistance
		cfg.CornerSpeedScale = cf.CornerSpeedScale
		cfg.ReverseSpeedScale = cf.ReverseSpeedScale
		cfg.TurnArcHalfFovDeg = cf.TurnArcHalfFovDeg
		cfg.TurnOpenRangeM = cf.TurnOpenRangeM
		cfg.CornerLeakMarginM = cf.CornerLeakMarginM
		cfg.MinForwardClearanceM = cf.MinForwardClearanceM
		cfg.MinReverseClearanceM = cf.MinReverseClearanceM
	}

	cePath := filepath.Join(configRoot, profile.DefaultCorridorEstimatorTOMLPath)
	if ce, err := profile.Load[profile.CorridorEstimatorConfig](cePath, nil); err != nil {
		logger.Warn("corridorfollower: loading corridor_estimator.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
	} else {
		cfg.DecisionBoundaryM = ce.DecisionBoundaryM
	}

	lsPath := filepath.Join(configRoot, profile.DefaultLidarSectorsTOMLPath)
	if ls, err := profile.Load[profile.LidarSectorsConfig](lsPath, nil); err != nil {
		logger.Warn("corridorfollower: loading lidar_sectors.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
	} else {
		cfg.MinValidRangeM = ls.MinValidRangeM
	}

	robotPath := filepath.Join(configRoot, profile.DefaultRobotTOMLPath)
	if r, err := profile.LoadRobotConfig(robotPath, hardwareProfileNames); err != nil {
		logger.Warn("corridorfollower: loading robot.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
	} else {
		cfg.MaxSteeringAngleRad = r.MaxSteeringAngle()
	}

	return cfg
}
