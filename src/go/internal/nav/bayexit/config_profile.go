package bayexit

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/motion"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/corridorfollower"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/parking"
)

// ConfigFor resolves the Config to run with: DefaultConfig's literals,
// overlaid with the shipped TOML tree if configRoot is non-empty and loading
// succeeds; otherwise, or on any load failure, the literal defaults, logging
// why.
//
// This package had no loader at all until now, so navigator.Params.BayExitConfig
// was never set at either call site and every bay-exit run -- simulated or on
// hardware -- used Go's literal defaults regardless of --config-root or the
// shipped TOML, including corridorfollower.DefaultBayExitClearanceGuard
// disagreeing outright with corridor_follower.toml's shipped `true`.
//
// Mirrors corridorfollower.ConfigFor's defensive pattern: each source loads
// and falls back independently, since they are unrelated failure domains.
func ConfigFor(logger *slog.Logger, configRoot string, hardwareProfileNames []string) Config {
	cfg := DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	// corridorfollower.ConfigFor already loads every BayExit* field, plus the
	// base follower fields it needs (MinForwardClearanceM,
	// ForwardArcHalfFovRad, MinValidRangeM, MaxSteeringAngleRad, ...), from
	// corridor_follower.toml/corridor_estimator.toml/lidar_sectors.toml/
	// robot.toml. Re-loading any of those here would just duplicate that
	// work and risk drifting from it.
	cfg.Follower = corridorfollower.ConfigFor(logger, configRoot, hardwareProfileNames)

	robotPath := filepath.Join(configRoot, profile.DefaultRobotTOMLPath)
	if r, err := profile.LoadRobotConfig(robotPath, hardwareProfileNames); err != nil {
		logger.Warn("bayexit: loading robot.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
	} else {
		cfg.WheelbaseM = r.Ackermann.Wheelbase
		cfg.RearSteerRatio = r.Drivetrain.RearSteerRatio
		cfg.YawGain = r.Drivetrain.YawGain
		cfg.ChassisLengthM = r.Chassis.Length
		cfg.ChassisWidthM = r.Chassis.Width
		cfg.MinTurnRadiusM = r.Drivetrain.MinTurnRadiusM
		cfg.SpeedResponseTauS = r.Drivetrain.SpeedResponseTauS
		cfg.LidarMaxRangeM = r.Lidar.MaxRange
	}

	profile.Apply(logger, filepath.Join(configRoot, profile.DefaultTrackTOMLPath), nil,
		func(t generated.TrackConfig) {
			cfg.ParkingLot = parking.ParkingLotSpecs{
				Length:             t.Parking.Length,
				Width:              t.Parking.Width,
				WallOffsetM:        t.Parking.WallOffset,
				BlockSpacingFactor: t.Parking.SpacingFactor,
			}
		})

	profile.Apply(logger, filepath.Join(configRoot, profile.DefaultControlTOMLPath), nil,
		func(c motion.NavigationMotionControl) {
			cfg.ControlHz = c.ControlHz
		})

	profile.Apply(logger, filepath.Join(configRoot, profile.DefaultPursuitTOMLPath), nil,
		func(p motion.NavigationMotionPursuit) {
			cfg.MaxSteeringRateRadPerS = p.MaxSteeringRate
			cfg.ServoSlewRateRadPerS = p.ServoSlewRateRadS
		})

	return cfg
}
