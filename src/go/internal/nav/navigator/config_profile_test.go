package navigator_test

import (
	"log/slog"
	"path/filepath"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navigator"
)

// hardwareProfileNames matches the currently active profile
// (VTITAN_HARDWARE_PROFILE = "270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm"),
// the same pair internal/nav/controllers' own config test uses. robot.toml
// deliberately omits drivetrain.max_speed_mps and steering.max_wheel_angle_deg
// -- they describe a specific motor and servo -- so without profiles there is
// nothing legal to load them from.
var hardwareProfileNames = []string{"270deg-hiwonder-35kg", "rev-hd-hex-motor-6000rpm"}

// repoRoot walks up from this test file's package directory
// (platform/robot-go/internal/nav/navigator) to the repo root, so ConfigFor
// runs against the real checked-in TOML tree.
func repoRoot(t *testing.T) string {
	t.Helper()
	return filepath.Join("..", "..", "..", "..", "..")
}

// TestConfigFor_ProfileSourcedRobotFieldsAreNonZero is a regression test for
// a silent-zero config bug.
//
// ConfigFor applies robot.toml's fields UNCONDITIONALLY, so a key that fails
// to load lands as 0 rather than leaving DefaultConfig's value in place. It
// used to read robot.toml through the generic loader with no profile names,
// which cannot see drivetrain.max_speed_mps at all -- leaving
// DrivetrainMaxSpeedMPS at 0, which makes every Config.*SpeedMPS() method
// return min(tier, 0). A robot that cannot move.
//
// It surfaced as navigation failure, not as a config error: the first native
// corpus sweep run with a config root scored 640/640 STUCK at max speed
// 0.000. Assert both fields are positive so the zero can never come back
// wearing that disguise.
func TestConfigFor_ProfileSourcedRobotFieldsAreNonZero(t *testing.T) {
	t.Parallel()

	logger := slog.New(slog.DiscardHandler)
	cfg := navigator.ConfigFor(logger, repoRoot(t), hardwareProfileNames)

	if cfg.DrivetrainMaxSpeedMPS <= 0 {
		t.Errorf("DrivetrainMaxSpeedMPS = %v, want > 0 (robot.toml + hardware profile)",
			cfg.DrivetrainMaxSpeedMPS)
	}
	if cfg.MaxSteeringAngleRad <= 0 {
		t.Errorf("MaxSteeringAngleRad = %v, want > 0 (robot.toml + hardware profile)",
			cfg.MaxSteeringAngleRad)
	}
	// The speed accessors are the reason the zero mattered, so check the
	// effective value a caller actually drives at, not just the raw field.
	if cfg.MaxSpeedMPS() <= 0 {
		t.Errorf("MaxSpeedMPS() = %v, want > 0", cfg.MaxSpeedMPS())
	}
}

// TestConfigFor_WithoutProfilesKeepsDefaults checks the other half: robot.toml
// cannot be satisfied without a profile, and the right response is to keep
// DefaultConfig's values (and warn), not to write zeros over them.
func TestConfigFor_WithoutProfilesKeepsDefaults(t *testing.T) {
	t.Parallel()

	logger := slog.New(slog.DiscardHandler)
	cfg := navigator.ConfigFor(logger, repoRoot(t), nil)
	def := navigator.DefaultConfig()

	if cfg.DrivetrainMaxSpeedMPS != def.DrivetrainMaxSpeedMPS {
		t.Errorf("DrivetrainMaxSpeedMPS = %v, want the default %v",
			cfg.DrivetrainMaxSpeedMPS, def.DrivetrainMaxSpeedMPS)
	}
	if cfg.MaxSteeringAngleRad != def.MaxSteeringAngleRad {
		t.Errorf("MaxSteeringAngleRad = %v, want the default %v",
			cfg.MaxSteeringAngleRad, def.MaxSteeringAngleRad)
	}
}
