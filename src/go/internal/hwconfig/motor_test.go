//go:build linux

package hwconfig_test

import (
	"log/slog"
	"os"
	"path/filepath"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/internal/hwconfig"
)

// TestMotor_ReadsDriveReversedFromMotorsTOML pins the drive-invert source:
// motors.toml's drive.reversed, not a --motor-invert/--pico-motor-invert
// flag (removed in favor of this single config source).
func TestMotor_ReadsDriveReversedFromMotorsTOML(t *testing.T) {
	t.Parallel()

	root := t.TempDir()
	full := filepath.Join(root, profile.DefaultMotorsTOMLPath)
	if err := os.MkdirAll(filepath.Dir(full), 0o750); err != nil {
		t.Fatalf("MkdirAll: %v", err)
	}
	if err := os.WriteFile(full, []byte("[drive]\nreversed = true\n"), 0o600); err != nil {
		t.Fatalf("WriteFile: %v", err)
	}

	cfg := hwconfig.Motor(slog.Default(), root)

	if !cfg.Invert {
		t.Error("Invert = false, want true from motors.toml's drive.reversed")
	}
}

// TestMotor_NoConfigRootLeavesInvertFalse pins the fallback: without a
// config root Motor returns DefaultConfig unchanged, not a guess.
func TestMotor_NoConfigRootLeavesInvertFalse(t *testing.T) {
	t.Parallel()

	if cfg := hwconfig.Motor(slog.Default(), ""); cfg.Invert {
		t.Error("Invert = true with no config root, want false")
	}
}
