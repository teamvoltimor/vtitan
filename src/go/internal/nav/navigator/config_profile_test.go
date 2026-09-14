package navigator_test

import (
	"log/slog"
	"os"
	"path/filepath"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
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

// TestConfigFor_FirstLapCornerCautionLoadsTrue pins
// first_lap_corner_caution's wiring against the real checked-in
// waypoints.toml, which ships true. Loaded via loadApplyTOML +
// navWaypointsTOML's `default` tag, so a missing key must ALSO read true, not
// the Go zero value -- see DefaultFirstLapCornerCaution's registration in
// ConfigFor.
func TestConfigFor_FirstLapCornerCautionLoadsTrue(t *testing.T) {
	t.Parallel()

	logger := slog.New(slog.DiscardHandler)
	cfg := navigator.ConfigFor(logger, repoRoot(t), hardwareProfileNames)
	if !cfg.FirstLapCornerCaution {
		t.Error("ConfigFor(...).FirstLapCornerCaution = false, want true (the shipped value)")
	}
}

// TestConfigFor_OmittedKeysResolveShippedDefaults guards the defaults
// registry the generated DTOs rely on.
//
// The navigator decodes into the same generated structs every other consumer
// uses, and none of them carry `default` tags, so a key absent from the TOML
// resolves through configDefaultsByType. These are the values the removed
// hand-written navigator structs used to tag; a silent regression would show
// up as a zero (false / 0.0) that still loads cleanly and overwrites
// DefaultConfig.
func TestConfigFor_OmittedKeysResolveShippedDefaults(t *testing.T) {
	t.Parallel()

	root := t.TempDir()
	writeTOML(t, root, profile.DefaultWaypointsTOMLPath, "arc_radius = 0.45\n")
	writeTOML(t, root, profile.DefaultSignRouterTOMLPath, "activation_dist_m = 1.40\n")

	logger := slog.New(slog.DiscardHandler)
	cfg := navigator.ConfigFor(logger, root, nil)

	if !cfg.FirstLapCornerCaution {
		t.Error("FirstLapCornerCaution = false, want true (registry default)")
	}
	if cfg.FinishApproachM != 0.40 {
		t.Errorf("FinishApproachM = %v, want 0.40 (registry default)", cfg.FinishApproachM)
	}
	if !cfg.SignLanePlanner {
		t.Error("SignLanePlanner = false, want true (registry default)")
	}
	if cfg.SignLaneRampM != 0.90 {
		t.Errorf("SignLaneRampM = %v, want 0.90 (registry default)", cfg.SignLaneRampM)
	}
	if cfg.SignLaneHoldM != 0.25 {
		t.Errorf("SignLaneHoldM = %v, want 0.25 (registry default)", cfg.SignLaneHoldM)
	}
	if !cfg.SignAwareSpeed {
		t.Error("SignAwareSpeed = false, want true (registry default)")
	}
}

// writeTOML creates relPath under root, making parent directories, so a
// ConfigFor test can present a config root holding only the files it cares
// about and exercise the omitted-key fallbacks.
func writeTOML(t *testing.T, root, relPath, contents string) {
	t.Helper()

	full := filepath.Join(root, filepath.FromSlash(relPath))
	if err := os.MkdirAll(filepath.Dir(full), 0o755); err != nil {
		t.Fatalf("mkdir %s: %v", filepath.Dir(full), err)
	}
	if err := os.WriteFile(full, []byte(contents), 0o600); err != nil {
		t.Fatalf("write %s: %v", full, err)
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
