package controllers

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/config/profile"
)

// ConfigFor resolves the Config to run with: DefaultConfig's literals,
// overlaid with each source TOML file (clearance/control/pursuit/
// lidar_sectors/escape/waypoints, each loaded independently from
// <configRoot>/profile.DefaultXxxTOMLPath) plus robot.toml + the active
// hardwareProfileNames for the RobotSpecs-derived fields, if configRoot is
// non-empty and loading succeeds; otherwise, or on any load failure, the
// literal defaults, logging why. Each file loads and falls back
// independently, since they're unrelated failure domains.
func ConfigFor(logger *slog.Logger, configRoot string, hardwareProfileNames []string) Config {
	cfg := DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	clearancePath := filepath.Join(configRoot, profile.DefaultClearanceTOMLPath)
	if loaded, err := profile.Load[profile.ClearanceConfig](clearancePath, nil); err != nil {
		logger.Warn("controllers: loading clearance.toml, falling back to defaults", "error", err)
	} else {
		cfg.ContactDist = loaded.ContactDist
		cfg.ObstaclesContactDist = loaded.ObstaclesContactDist
		cfg.SlowDist = loaded.SlowDist
		cfg.FastDist = loaded.FastDist
		cfg.PathMargin = loaded.PathMargin
	}

	controlPath := filepath.Join(configRoot, profile.DefaultControlTOMLPath)
	if loaded, err := profile.Load[profile.ControlConfig](controlPath, nil); err != nil {
		logger.Warn("controllers: loading control.toml, falling back to defaults", "error", err)
	} else {
		cfg.ControlHz = loaded.ControlHz
	}

	pursuitPath := filepath.Join(configRoot, profile.DefaultPursuitTOMLPath)
	if loaded, err := profile.Load[profile.PursuitConfig](pursuitPath, nil); err != nil {
		logger.Warn("controllers: loading pursuit.toml, falling back to defaults", "error", err)
	} else {
		cfg.LookaheadShort = loaded.LookaheadShort
		cfg.LookaheadLong = loaded.LookaheadLong
		cfg.LookaheadTransition = loaded.LookaheadTransition
		cfg.LookaheadBlendStart = loaded.LookaheadBlendStart
		cfg.SteerKp = loaded.SteerKp
		cfg.MaxSteeringRate = loaded.MaxSteeringRate
		cfg.CornerTurnThresholdRad = loaded.CornerTurnThresholdRad
	}

	lidarSectorsPath := filepath.Join(configRoot, profile.DefaultLidarSectorsTOMLPath)
	if loaded, err := profile.Load[profile.LidarSectorsConfig](lidarSectorsPath, nil); err != nil {
		logger.Warn(
			"controllers: loading lidar_sectors.toml, falling back to defaults",
			"error",
			err,
		)
	} else {
		cfg.FrontHalfFovDeg = loaded.FrontHalfFovDeg
		cfg.ThreatHalfFovDeg = loaded.ThreatHalfFovDeg
		cfg.SelfDetectionThresholdM = loaded.SelfDetectionThresholdM
		cfg.MinValidRangeM = loaded.MinValidRangeM
		cfg.ThreatNoDetectionRangeM = loaded.ThreatNoDetectionRangeM
		cfg.NoDataRangeM = loaded.NoDataRangeM
		cfg.BlindWedgeLeftMinDeg = loaded.BlindWedgeLeftMinDeg
		cfg.BlindWedgeLeftMaxDeg = loaded.BlindWedgeLeftMaxDeg
		cfg.BlindWedgeRightMinDeg = loaded.BlindWedgeRightMinDeg
		cfg.BlindWedgeRightMaxDeg = loaded.BlindWedgeRightMaxDeg
	}

	escapePath := filepath.Join(configRoot, profile.DefaultEscapeTOMLPath)
	escapeDefaults := map[string]any{"min_history_for_distance": DefaultMinHistoryForDistance}
	if loaded, err := profile.LoadWithDefaults[profile.EscapeConfig](escapePath, nil, escapeDefaults); err != nil {
		logger.Warn("controllers: loading escape.toml, falling back to defaults", "error", err)
	} else {
		cfg.RevSpeed = loaded.RevSpeed
		cfg.RevSteerDeg = loaded.RevSteerDeg
		cfg.KTurnMinFrames = profile.Frames(loaded.KTurnMinS, cfg.ControlHz)
		cfg.KTurnMaxFrames = profile.Frames(loaded.KTurnMaxS, cfg.ControlHz)
		cfg.StuckMoveThreshold = loaded.StuckMoveThreshold
		cfg.StuckTimeoutFrames = profile.Frames(loaded.StuckTimeoutS, cfg.ControlHz)
		cfg.SideCorrectionSteerDeg = loaded.SideCorrectionSteerDeg
		cfg.SideCorrectionSpeed = loaded.SideCorrectionSpeed
		cfg.SideCorrectionFrames = profile.Frames(loaded.SideCorrectionS, cfg.ControlHz)
		cfg.StuckConfirmationChecks = loaded.StuckConfirmationChecks
		cfg.StuckHistoryFloor = profile.Frames(loaded.StuckHistoryFloorS, cfg.ControlHz)
		cfg.MinHistoryForDistance = loaded.MinHistoryForDistance
	}

	waypointsPath := filepath.Join(configRoot, profile.DefaultWaypointsTOMLPath)
	if loaded, err := profile.Load[profile.WaypointsConfig](waypointsPath, nil); err != nil {
		logger.Warn("controllers: loading waypoints.toml, falling back to defaults", "error", err)
	} else {
		cfg.ControllerReachedDistanceM = loaded.ControllerReachedDistanceM
	}

	robotPath := filepath.Join(configRoot, profile.DefaultRobotTOMLPath)
	if loaded, err := profile.LoadRobotConfig(robotPath, hardwareProfileNames); err != nil {
		logger.Warn("controllers: loading robot.toml, falling back to defaults", "error", err)
	} else {
		cfg.WheelbaseM = loaded.Ackermann.Wheelbase
		cfg.ChassisWidthM = loaded.Chassis.Width
		cfg.MaxSteeringAngleRad = loaded.MaxSteeringAngle()
		cfg.LidarToFrontBumperM = loaded.LidarToFrontBumper()
		cfg.LidarToRearBumperM = loaded.LidarToRearBumper()
		cfg.LidarMaxRangeM = loaded.Lidar.MaxRange
	}

	return cfg
}
