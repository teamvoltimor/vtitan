//go:build linux

package motor

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/hardware/motors"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
)

// SpeedScaleFor resolves the drive.speed_scale to run with: if configRoot is
// non-empty, it loads motors.HardwareMotorsMotors from
// <configRoot>/profile.DefaultMotorsTOMLPath overlaid with the profiles
// named in profile.ActiveNames(); otherwise, or if loading fails, it falls
// back to DefaultSpeedScalePercentPerMPS and logs why.
func SpeedScaleFor(logger *slog.Logger, configRoot string) float64 {
	if configRoot == "" {
		return DefaultSpeedScalePercentPerMPS
	}

	basePath := filepath.Join(configRoot, profile.DefaultMotorsTOMLPath)
	scale := DefaultSpeedScalePercentPerMPS
	profile.Apply(logger, basePath, profile.ActiveNames(), func(cfg motors.HardwareMotorsMotors) {
		scale = cfg.Drive.SpeedScale
	})
	return scale
}
