package signrouter_test

import (
	"log/slog"
	"path/filepath"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/signrouter"
)

// shippedMaxIngestRangeM is sign_discovery.toml's max_ingest_range_m.
//
// It is deliberately NOT taken from DefaultMaxIngestRangeM: that literal is
// 2.0 and the shipped file says 1.5, so comparing against the default would
// assert the opposite of what ships -- and, worse, would pass if
// DiscoveryConfigFor silently failed to read the file at all, which is the
// exact failure adr:0068-go-parallel-track-single-cutover names (a Go loader
// can succeed without reading a key, through a missing mapstructure tag).
// The divergence is what gives this assertion its teeth.
const shippedMaxIngestRangeM = 1.5

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

// TestDiscoveryConfigFor_LoadsSignDiscoveryTOML confirms every key
// DiscoveryConfigFor maps out of the shipped sign_discovery.toml actually
// arrives. The router half of this file already had a pin test; the discovery
// half did not, which left the five keys below able to drift from the values
// the Python stack races on without anything saying so.
func TestDiscoveryConfigFor_LoadsSignDiscoveryTOML(t *testing.T) {
	t.Parallel()

	logger := slog.New(slog.DiscardHandler)
	cfg := signrouter.DiscoveryConfigFor(logger, repoRoot(t))

	if cfg.MaxIngestRangeM != shippedMaxIngestRangeM {
		t.Errorf("sign_discovery.toml wiring: MaxIngestRangeM = %v, want %v (Go's own default is %v, "+
			"so this value means the file was not read)",
			cfg.MaxIngestRangeM, shippedMaxIngestRangeM, signrouter.DefaultMaxIngestRangeM)
	}
	if cfg.AssociationDistM != 0.25 {
		t.Errorf("sign_discovery.toml wiring: AssociationDistM = %v, want 0.25", cfg.AssociationDistM)
	}
	if cfg.MinHits != 3 {
		t.Errorf("sign_discovery.toml wiring: MinHits = %v, want 3", cfg.MinHits)
	}
	if cfg.MinReliableBBoxHeightPX != 5 {
		t.Errorf("sign_discovery.toml wiring: MinReliableBBoxHeightPX = %v, want 5",
			cfg.MinReliableBBoxHeightPX)
	}
	if cfg.RobotCorridorFlipTicks != 5 {
		t.Errorf("sign_discovery.toml wiring: RobotCorridorFlipTicks = %v, want 5",
			cfg.RobotCorridorFlipTicks)
	}
}
