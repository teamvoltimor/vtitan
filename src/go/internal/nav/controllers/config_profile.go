package controllers

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/escape"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/motion"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/sensors"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/waypoint"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
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
		cfg.ForwardPathAheadOfBumper = loaded.ForwardPathAheadOfBumper
		// Absent from the file means 0, which RobustMinRange reads as the bare
		// minimum -- keep the shipped default instead of silently disabling it.
		if loaded.RiskRayWindow > 0 {
			cfg.RiskRayWindow = loaded.RiskRayWindow
		}
	}

	controlPath := filepath.Join(configRoot, profile.DefaultControlTOMLPath)
	if loaded, err := profile.Load[motion.NavigationMotionControl](controlPath, nil); err != nil {
		logger.Warn("controllers: loading control.toml, falling back to defaults", "error", err)
	} else {
		cfg.ControlHz = loaded.ControlHz
	}

	pursuitPath := filepath.Join(configRoot, profile.DefaultPursuitTOMLPath)
	if loaded, err := profile.Load[motion.NavigationMotionPursuit](pursuitPath, nil); err != nil {
		logger.Warn("controllers: loading pursuit.toml, falling back to defaults", "error", err)
	} else {
		cfg.LookaheadShort = loaded.LookaheadShort
		cfg.LookaheadLong = loaded.LookaheadLong
		cfg.OpenLookaheadLong = loaded.OpenLookaheadLong
		cfg.YawGainCompensation = loaded.YawGainCompensation
		cfg.ObstaclesYawGainCompensation = loaded.ObstaclesYawGainCompensation
		cfg.LookaheadTransition = loaded.LookaheadTransition
		cfg.LookaheadBlendStart = loaded.LookaheadBlendStart
		cfg.SteerKp = loaded.SteerKp
		cfg.MaxSteeringRate = loaded.MaxSteeringRate
		cfg.ServoSlewRateRadS = loaded.ServoSlewRateRadS
		cfg.TargetSearchSpanM = loaded.TargetSearchSpanM
		cfg.TargetSenseGate = loaded.TargetSenseGate
		cfg.CornerTurnThresholdRad = loaded.CornerTurnThresholdRad
	}

	lidarSectorsPath := filepath.Join(configRoot, profile.DefaultLidarSectorsTOMLPath)
	// lidar_sectors.toml is the single source: it carries
	// rear_self_detection_from_chassis, so the value is read straight from the
	// file rather than from a per-type fallback.
	if loaded, err := profile.Load[sensors.NavigationSensorsLidarSectors](lidarSectorsPath, nil); err != nil {
		logger.Warn(
			"controllers: loading lidar_sectors.toml, falling back to defaults",
			"error",
			err,
		)
	} else {
		cfg.FrontHalfFovDeg = loaded.FrontHalfFovDeg
		cfg.ThreatHalfFovDeg = loaded.ThreatHalfFovDeg
		cfg.SelfDetectionThresholdM = loaded.SelfDetectionThresholdM
		cfg.RearSelfDetectionFromChassis = loaded.RearSelfDetectionFromChassis
		cfg.MinValidRangeM = loaded.MinValidRangeM
		cfg.ThreatNoDetectionRangeM = loaded.ThreatNoDetectionRangeM
		cfg.NoDataRangeM = loaded.NoDataRangeM
		cfg.BlindWedgeLeftMinDeg = loaded.BlindWedgeLeftMinDeg
		cfg.BlindWedgeLeftMaxDeg = loaded.BlindWedgeLeftMaxDeg
		cfg.BlindWedgeRightMinDeg = loaded.BlindWedgeRightMinDeg
		cfg.BlindWedgeRightMaxDeg = loaded.BlindWedgeRightMaxDeg
	}

	escapePath := filepath.Join(configRoot, profile.DefaultEscapeTOMLPath)
	if loaded, err := profile.Load[escape.NavigationEscapeEscape](escapePath, nil); err != nil {
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
	if loaded, err := profile.Load[waypoint.NavigationWaypointWaypoints](waypointsPath, nil); err != nil {
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
