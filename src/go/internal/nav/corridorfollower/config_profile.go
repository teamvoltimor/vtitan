package corridorfollower

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/blind_nav"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/sensors"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
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
// The bay-exit fields ARE overlaid here as of 2026-09-05. They were left out
// on the grounds that they belonged to "internal/nav/bayexit's own TOML and
// loader" -- neither of which was ever written, so in practice every bay
// constant read a Go literal, corridor_follower.toml did not name them
// either, and Go drifted from Python unnoticed: the clearance guard was still
// OFF and the arc still 0.3 while Python shipped the solved full-lock
// ratchet. They live on this Config (see bayexit.Config.Follower), so this is
// the loader that owns them.
func ConfigFor(
	logger *slog.Logger,
	configRoot string,
	hardwareProfileNames []string,
) Config {
	cfg := DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	profile.Apply(logger, filepath.Join(configRoot, profile.DefaultCorridorFollowerTOMLPath), nil,
		func(cf blind_nav.NavigationBlindNavCorridorFollower) {
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

			cfg.AssumeBayStart = cf.AssumeBayStart
			cfg.BayExitClearanceGuard = cf.BayExitClearanceGuard
			cfg.BayExitClearanceMarginM = cf.BayExitClearanceMarginM
			cfg.BayExitClearanceToleranceM = cf.BayExitClearanceToleranceM
			cfg.BayExitArcSteerNorm = cf.BayExitArcSteerNorm
			cfg.BayExitSpeedScale = cf.BayExitSpeedScale
			cfg.BayExitCycle = cf.BayExitCycle
			cfg.BayExitCycleReverseM = cf.BayExitCycleReverseM
			cfg.BayExitCycleReverseSteerNorm = cf.BayExitCycleReverseSteerNorm
			cfg.BayExitForwardM = cf.BayExitForwardM
			cfg.BayExitReverseM = cf.BayExitReverseM
			cfg.BayExitSteerNorm = cf.BayExitSteerNorm
			cfg.BayExitReverseSteerNorm = cf.BayExitReverseSteerNorm
			cfg.BayExitHoldSteer = cf.BayExitHoldSteer
			cfg.BayExitLegStallTicks = cf.BayExitLegStallTicks
			cfg.BayExitLatchDirection = cf.BayExitLatchDirection
			cfg.BayExitLatchReverse = cf.BayExitLatchReverse
			cfg.BayExitFallbackFrames = cf.BayExitFallbackFrames
			cfg.BayExitMaxFrames = cf.BayExitMaxFrames

			cfg.BayExitGuardOverlapRecovery = cf.BayExitGuardOverlapRecovery
			cfg.BayExitOpenSideSectorDeg = cf.BayExitOpenSideSectorDeg
			cfg.BayExitOpenSideVotes = cf.BayExitOpenSideVotes
			cfg.BayExitSpeedMPS = cf.BayExitSpeedMps
			cfg.BayExitContactDistM = cf.BayExitContactDistM
			cfg.BayExitContactRecoveryTicks = cf.BayExitContactRecoveryTicks
			cfg.BayExitTargetYawDeg = cf.BayExitTargetYawDeg
			cfg.BayExitLegMaxS = cf.BayExitLegMaxS
			cfg.BayExitGuardBlockTicks = cf.BayExitGuardBlockTicks
			cfg.BayExitGuardMeasuredCoast = cf.BayExitGuardMeasuredCoast
			cfg.BayExitGuardMirrorsReverse = cf.BayExitGuardMirrorsReverse
			cfg.BayExitDrUsesMeasuredYaw = cf.BayExitDrUsesMeasuredYaw
		})

	profile.Apply(logger, filepath.Join(configRoot, profile.DefaultCorridorEstimatorTOMLPath), nil,
		func(ce blind_nav.NavigationBlindNavCorridorEstimator) {
			cfg.DecisionBoundaryM = ce.DecisionBoundaryM
		})

	profile.Apply(logger, filepath.Join(configRoot, profile.DefaultLidarSectorsTOMLPath), nil,
		func(ls sensors.NavigationSensorsLidarSectors) {
			cfg.MinValidRangeM = ls.MinValidRangeM
		})

	robotPath := filepath.Join(configRoot, profile.DefaultRobotTOMLPath)
	if r, err := profile.LoadRobotConfig(robotPath, hardwareProfileNames); err != nil {
		logger.Warn("corridorfollower: loading robot.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
	} else {
		cfg.MaxSteeringAngleRad = r.MaxSteeringAngle()
	}

	return cfg
}
