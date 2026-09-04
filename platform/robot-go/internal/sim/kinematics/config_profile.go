package kinematics

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/config/profile"
)

// ConfigFor resolves the Config to run with: DefaultConfig's literal,
// overlaid with profile.PursuitConfig loaded from
// <configRoot>/profile.DefaultPursuitTOMLPath if configRoot is non-empty
// and loading succeeds; otherwise, or on any load failure, the literal
// default, logging why.
func ConfigFor(logger *slog.Logger, configRoot string) Config {
	cfg := DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	path := filepath.Join(configRoot, profile.DefaultPursuitTOMLPath)
	pursuit, err := profile.Load[profile.PursuitConfig](path, nil)
	if err != nil {
		logger.Warn("kinematics: loading pursuit.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
		return cfg
	}

	cfg.MaxSteeringRateRadPerS = pursuit.MaxSteeringRate
	return cfg
}

// ParamsFor resolves the full integrator Params from the shipped TOML tree:
// robot.toml (overlaid with hardwareProfileNames) for the chassis and
// drivetrain facts, pursuit.toml for the steering slew rate. Mirrors Python's
// _KinematicsConstants.from_tuning, which reads the same two sources.
//
// An empty configRoot, or a load failure, returns DefaultParams. That is NOT
// the shipped robot and the gap is large enough to change what a sweep
// measures: the literals were written before the profile loader existed and
// have since drifted from the profile by 4x on acceleration (0.5 vs 2.0
// m/s2) and 3.5x on drive lag (0.1 vs 0.35 s), with max steer 70.2 deg
// against the servo's 85. Those distortions are mild at the base ladder's
// 0.156 m/s and severe at the Open ladder's 0.50.
func ParamsFor(logger *slog.Logger, configRoot string, hardwareProfileNames []string) Params {
	params := DefaultParams()
	if configRoot == "" {
		return params
	}

	params.MaxSteerRateRadPerS = ConfigFor(logger, configRoot).MaxSteeringRateRadPerS

	robotPath := filepath.Join(configRoot, profile.DefaultRobotTOMLPath)
	loaded, err := profile.LoadRobotConfig(robotPath, hardwareProfileNames)
	if err != nil {
		logger.Warn("kinematics: loading robot.toml, falling back to default chassis params",
			"config_root", configRoot, "error", err)
		return params
	}

	params.WheelbaseM = loaded.Ackermann.Wheelbase
	params.MaxSteerRad = loaded.MaxSteeringAngle()
	params.MaxAccelMPS2 = loaded.Drivetrain.MaxAccelMPS2
	params.MaxSpeedMPS = loaded.Drivetrain.MaxSpeedMPS
	params.SpeedTauS = loaded.Drivetrain.SpeedResponseTauS
	params.YawGain = loaded.Drivetrain.YawGain
	// Sign is irrelevant here -- turnReferenceLen takes the magnitude, and
	// nothing else reads the field -- but the config value is carried rather
	// than the literal so a future sign-dependent use starts from the truth.
	params.RearSteerRatio = loaded.Drivetrain.RearSteerRatio
	return params
}
