package stateestimator_test

import (
	"log/slog"
	"os"
	"path/filepath"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/stateestimator"
)

// The gain must come from the file, not the literal default: a tree whose
// state_estimator.toml differs from the default must resolve the file's value.
func TestConfigFor_ReadsYawCorrectionGain(t *testing.T) {
	t.Parallel()

	root := t.TempDir()
	path := filepath.Join(root, filepath.FromSlash(profile.DefaultStateEstimatorTOMLPath))
	if err := os.MkdirAll(filepath.Dir(path), 0o750); err != nil {
		t.Fatalf("MkdirAll: %v", err)
	}
	if err := os.WriteFile(path, []byte("yaw_correction_gain = 0.11\n"), 0o600); err != nil {
		t.Fatalf("WriteFile: %v", err)
	}

	cfg := stateestimator.ConfigFor(slog.New(slog.DiscardHandler), root)
	if cfg.YawCorrectionGain != 0.11 {
		t.Errorf("YawCorrectionGain = %v, want 0.11 from the file", cfg.YawCorrectionGain)
	}
}

func TestConfigFor_EmptyRootUsesDefault(t *testing.T) {
	t.Parallel()

	cfg := stateestimator.ConfigFor(slog.New(slog.DiscardHandler), "")
	if cfg.YawCorrectionGain != stateestimator.DefaultYawCorrectionGain {
		t.Errorf("YawCorrectionGain = %v, want default %v",
			cfg.YawCorrectionGain, stateestimator.DefaultYawCorrectionGain)
	}
}
