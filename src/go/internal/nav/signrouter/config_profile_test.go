package signrouter_test

import (
	"log/slog"
	"path/filepath"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/signrouter"
)

// repoRoot walks up from this test file's package directory
// (src/go/internal/nav/signrouter) to the repo root, so ConfigFor can be
// exercised against the real checked-in TOML tree, matching the pattern in
// internal/nav/controllers/config_profile_test.go.
func repoRoot(t *testing.T) string {
	t.Helper()
	return filepath.Join("..", "..", "..", "..", "..")
}

// TestConfigFor_LoadsSlotMapAndSignGrid confirms the shipped sign_router.toml
// and track.toml wire the slot map's knobs in: SLOT_SIGN_MAP ships TRUE, so a
// Go run must select SlotSignMap over ObservedSignMap, and the sign lattice
// geometry must come from track.toml rather than the literal defaults.
func TestConfigFor_LoadsSlotMapAndSignGrid(t *testing.T) {
	t.Parallel()

	logger := slog.New(slog.DiscardHandler)
	cfg := signrouter.ConfigFor(logger, repoRoot(t))

	if !cfg.SlotSignMap {
		t.Error("ConfigFor(...).SlotSignMap = false, want true (the shipped sign_router.toml sets it)")
	}
	if cfg.SignWidthM != 0.05 {
		t.Errorf("SignWidthM = %v, want 0.05", cfg.SignWidthM)
	}
	if cfg.GridDepthNear != 1.0 || cfg.GridDepthMiddle != 1.5 || cfg.GridDepthFar != 2.0 {
		t.Errorf("GridDepth = %v/%v/%v, want 1.0/1.5/2.0",
			cfg.GridDepthNear, cfg.GridDepthMiddle, cfg.GridDepthFar)
	}
	if cfg.GridWidthOuter != 0.40 || cfg.GridWidthInner != 0.60 {
		t.Errorf("GridWidth = %v/%v, want 0.40/0.60", cfg.GridWidthOuter, cfg.GridWidthInner)
	}
	if cfg.TrackSizeM != 3.0 {
		t.Errorf("TrackSizeM = %v, want 3.0", cfg.TrackSizeM)
	}
	if cfg.SlotAcceptRadiusM != signrouter.DefaultSlotAcceptRadiusM {
		t.Errorf("SlotAcceptRadiusM = %v, want %v", cfg.SlotAcceptRadiusM, signrouter.DefaultSlotAcceptRadiusM)
	}
}
