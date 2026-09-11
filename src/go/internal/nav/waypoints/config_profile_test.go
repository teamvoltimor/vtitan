package waypoints_test

import (
	"log/slog"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/waypoints"
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
	if !cfg.CornerArcAssumeWide {
		t.Errorf("CornerArcAssumeWide = false, want true (Python CORNER_ARC_ASSUME_WIDE default)")
	}
	if !cfg.DeferCurrentCorridorReplan {
		t.Errorf("DeferCurrentCorridorReplan = false, want true (Python DEFER_CURRENT_CORRIDOR_REPLAN default)")
	}
	if cfg.UnconfirmedWidthInnerBiasM != def.UnconfirmedWidthInnerBiasM {
		t.Errorf(
			"UnconfirmedWidthInnerBiasM = %v, want the checked-in default %v",
			cfg.UnconfirmedWidthInnerBiasM,
			def.UnconfirmedWidthInnerBiasM,
		)
	}
	if cfg.ObstaclesCenterBiasM != def.ObstaclesCenterBiasM {
		t.Errorf(
			"ObstaclesCenterBiasM = %v, want the checked-in default %v",
			cfg.ObstaclesCenterBiasM,
			def.ObstaclesCenterBiasM,
		)
	}
}

// TestWaypointsTOML_SpellsOutTheWidthBeliefFlags asserts the three fields
// that carry the 596 -> 638/640 Open result are WRITTEN IN the shared
// waypoints.toml, not merely defaulted.
//
// ConfigFor registers viper defaults for all three, so a missing key reads
// as the shipped value and every behavioural test still passes -- the
// revert would be silent, and the file a reader consults to see what the
// robot drives would not mention them at all. Assert on the file's text so
// deleting a key fails here rather than in a corpus sweep months later.
func TestWaypointsTOML_SpellsOutTheWidthBeliefFlags(t *testing.T) {
	t.Parallel()

	path := filepath.Join(repoRoot(t), profile.DefaultWaypointsTOMLPath)
	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatalf("reading %s: %v", profile.DefaultWaypointsTOMLPath, err)
	}
	for _, assignment := range []string{
		"corner_arc_assume_wide = true",
		"defer_current_corridor_replan = true",
		"unconfirmed_width_inner_bias_m = 0.05",
	} {
		if !strings.Contains(string(raw), assignment) {
			t.Errorf("%s is missing %q; ConfigFor's viper default would hide the revert",
				profile.DefaultWaypointsTOMLPath, assignment)
		}
	}
}
