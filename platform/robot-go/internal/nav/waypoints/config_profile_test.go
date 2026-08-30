package waypoints_test

import (
	"log/slog"
	"path/filepath"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/waypoints"
)

// repoRoot walks up from this test file's package directory
// (platform/robot-go/internal/nav/waypoints) to the repo root, so
// ConfigFor can be exercised against the real checked-in waypoints.toml
// -- catching a path/field-name mismatch a testdata-fixture-only test
// wouldn't.
func repoRoot(t *testing.T) string {
	t.Helper()
	return filepath.Join("..", "..", "..", "..", "..")
}

func TestConfigFor_LoadsRealWaypointsTOML(t *testing.T) {
	t.Parallel()

	logger := slog.New(slog.DiscardHandler)
	cfg := waypoints.ConfigFor(logger, repoRoot(t))

	def := waypoints.DefaultConfig()
	if cfg.DedupeDistanceM != def.DedupeDistanceM {
		t.Errorf(
			"DedupeDistanceM = %v, want the checked-in default %v",
			cfg.DedupeDistanceM,
			def.DedupeDistanceM,
		)
	}
	countsDiffer := cfg.NumIntermediateArcPoints != def.NumIntermediateArcPoints ||
		cfg.StraightWaypointCount != def.StraightWaypointCount
	if countsDiffer {
		t.Errorf("counts = %v/%v, want %v/%v",
			cfg.NumIntermediateArcPoints, cfg.StraightWaypointCount,
			def.NumIntermediateArcPoints, def.StraightWaypointCount)
	}
	if cfg.WideCenterBiasSide != def.WideCenterBiasSide ||
		cfg.NarrowCenterBiasSide != def.NarrowCenterBiasSide {
		t.Errorf(
			"sides = %v/%v, want %v/%v (string-to-CorridorSide parsing)",
			cfg.WideCenterBiasSide,
			cfg.NarrowCenterBiasSide,
			def.WideCenterBiasSide,
			def.NarrowCenterBiasSide,
		)
	}
}
