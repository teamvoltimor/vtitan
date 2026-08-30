package directionestimator_test

import (
	"log/slog"
	"path/filepath"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/directionestimator"
)

// repoRoot walks up from this test file's package directory
// (platform/robot-go/internal/nav/directionestimator) to the repo root,
// so ConfigFor can be exercised against the real checked-in TOML files --
// catching a path/field-name mismatch a testdata-fixture-only test
// wouldn't.
func repoRoot(t *testing.T) string {
	t.Helper()
	return filepath.Join("..", "..", "..", "..", "..")
}

func TestConfigFor_LoadsRealNavigationTuningFiles(t *testing.T) {
	t.Parallel()

	logger := slog.New(slog.DiscardHandler)
	cfg := directionestimator.ConfigFor(logger, repoRoot(t))

	def := directionestimator.DefaultConfig()
	if cfg.MinVotes != def.MinVotes {
		t.Errorf("MinVotes = %v, want the checked-in default %v (direction_estimator.toml wiring)",
			cfg.MinVotes, def.MinVotes)
	}
	if cfg.AlignmentToleranceRad != def.AlignmentToleranceRad {
		t.Errorf(
			"AlignmentToleranceRad = %v, want %v",
			cfg.AlignmentToleranceRad,
			def.AlignmentToleranceRad,
		)
	}
	if cfg.DirectionArcHalfFovDeg != def.DirectionArcHalfFovDeg {
		t.Errorf("DirectionArcHalfFovDeg = %v, want %v (lidar_sectors.toml wiring)",
			cfg.DirectionArcHalfFovDeg, def.DirectionArcHalfFovDeg)
	}
	if cfg.TurnClearanceM != def.TurnClearanceM {
		t.Errorf(
			"TurnClearanceM = %v, want %v (corridor_follower.toml wiring)",
			cfg.TurnClearanceM,
			def.TurnClearanceM,
		)
	}
	if cfg.BayWallClearanceM != def.BayWallClearanceM {
		t.Errorf(
			"BayWallClearanceM = %v, want %v (Pydantic default, absent from the checked-in TOML)",
			cfg.BayWallClearanceM,
			def.BayWallClearanceM,
		)
	}
}
