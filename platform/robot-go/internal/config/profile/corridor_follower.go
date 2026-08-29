package profile

// CorridorFollowerConfig mirrors the subset of
// platform/shared/config/navigation/blind_nav/corridor_follower.toml
// (shared.config.navigation_tuning.blind_nav.CorridorFollowerParams) that
// internal/nav/directionestimator.DirectionFromParkingBay currently
// consumes. corridor_follower.py's own blind-creep/corner-turn control
// loop isn't ported to Go yet, so the rest of that TOML's fields
// (centering/heading gains, speed scales, corner-detection arc, etc.)
// are omitted here rather than mirrored unused.
type CorridorFollowerConfig struct {
	// MinForwardClearanceM matches MIN_FORWARD_CLEARANCE_M.
	MinForwardClearanceM float64 `mapstructure:"min_forward_clearance_m"`
	// TurnClearanceM matches TURN_CLEARANCE_M.
	TurnClearanceM float64 `mapstructure:"turn_clearance_m"`
	// BayWallClearanceM matches BAY_WALL_CLEARANCE_M. Not present in the
	// checked-in corridor_follower.toml -- every deployment currently
	// relies on the Pydantic model's own 0.20 default, applied here via
	// LoadWithDefaults rather than silently reading as 0.0.
	BayWallClearanceM float64 `mapstructure:"bay_wall_clearance_m"`
}

// DefaultCorridorFollowerTOMLPath is
// platform/shared/config/navigation/blind_nav/corridor_follower.toml,
// relative to the repo root. No per-component profile overlays -- pass
// nil profileNames to Load.
const DefaultCorridorFollowerTOMLPath = "platform/shared/config/navigation/blind_nav/corridor_follower.toml"

// DefaultBayWallClearanceM matches CorridorFollowerParams.BAY_WALL_CLEARANCE_M's
// Pydantic default -- see CorridorFollowerConfig.BayWallClearanceM's doc
// comment for why this needs a code default at all.
const DefaultBayWallClearanceM = 0.20

// CorridorFollowerDefaults is the LoadWithDefaults defaults map for
// CorridorFollowerConfig.
func CorridorFollowerDefaults() map[string]any {
	return map[string]any{"bay_wall_clearance_m": DefaultBayWallClearanceM}
}
