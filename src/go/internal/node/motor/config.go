//go:build linux

package motor

import (
	"errors"
	"fmt"
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/hardware/motors"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/actuation"
)

// SpeedScaleFor resolves the drive.speed_scale to run with: if configRoot is
// non-empty, it loads motors.HardwareMotorsMotors from
// <configRoot>/profile.DefaultMotorsTOMLPath overlaid with the profiles
// named in profile.ActiveNames(); otherwise, or if loading fails, it falls
// back to actuation.DefaultSpeedScalePercentPerMPS and logs why.
func SpeedScaleFor(logger *slog.Logger, configRoot string) float64 {
	if configRoot == "" {
		return actuation.DefaultSpeedScalePercentPerMPS
	}

	basePath := filepath.Join(configRoot, profile.DefaultMotorsTOMLPath)
	scale := actuation.DefaultSpeedScalePercentPerMPS
	profile.Apply(logger, basePath, profile.ActiveNames(), func(cfg motors.HardwareMotorsMotors) {
		scale = cfg.Drive.SpeedScale
	})
	return scale
}

// SteeringFor resolves the wheel-to-servo conversion from configRoot:
// servo_max_angle_deg and the linkage ratio from robot.toml (both need the
// active servo profile - profile.LoadRobotConfig refuses without it), and
// the offset trim from motors.toml's [steering] table, all overlaid with
// profile.ActiveNames().
//
// It returns an error rather than a fallback: the linkage ratio is a
// property of the fitted servo, and a guessed one steers every command by
// the wrong amount.
func SteeringFor(configRoot string) (actuation.SteeringConfig, error) {
	if configRoot == "" {
		return actuation.SteeringConfig{}, errors.New(
			"node/motor: a config root is required to load the steering geometry",
		)
	}

	names := profile.ActiveNames()
	robot, err := profile.LoadRobotConfig(
		filepath.Join(configRoot, filepath.FromSlash(profile.DefaultRobotTOMLPath)),
		names,
	)
	if err != nil {
		return actuation.SteeringConfig{}, fmt.Errorf("node/motor: loading robot.toml: %w", err)
	}
	motorsCfg, err := profile.Load[motors.HardwareMotorsMotors](
		filepath.Join(configRoot, filepath.FromSlash(profile.DefaultMotorsTOMLPath)),
		names,
	)
	if err != nil {
		return actuation.SteeringConfig{}, fmt.Errorf("node/motor: loading motors.toml: %w", err)
	}

	cfg := actuation.SteeringConfig{
		LinkageRatio:     robot.LinkageRatio(),
		ServoMaxAngleDeg: robot.Steering.ServoMaxAngleDeg,
		OffsetDeg:        motorsCfg.Steering.Offset,
	}
	if err = cfg.Validate(); err != nil {
		return actuation.SteeringConfig{}, fmt.Errorf("node/motor: %w", err)
	}
	return cfg, nil
}
