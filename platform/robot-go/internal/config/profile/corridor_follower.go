package profile

// CorridorFollowerConfig mirrors
// platform/config/navigation/blind_nav/corridor_follower.toml
// (shared.config.navigation_tuning.blind_nav.CorridorFollowerParams) in
// full.
//
// It once mirrored only the three fields
// internal/nav/directionestimator.DirectionFromParkingBay consumes, on the
// grounds that corridor_follower.py's own blind-creep/corner-turn control
// loop was not ported. It since was -- internal/nav/corridorfollower -- and
// the mirror was not extended with it, so every one of those constants read
// a Go literal and no edit to this TOML reached the running follower.
type CorridorFollowerConfig struct {
	// MinForwardClearanceM matches MIN_FORWARD_CLEARANCE_M.
	MinForwardClearanceM float64 `mapstructure:"min_forward_clearance_m"`
	// TurnClearanceM matches TURN_CLEARANCE_M.
	TurnClearanceM float64 `mapstructure:"turn_clearance_m"`
	// NarrowTurnClearanceM matches NARROW_TURN_CLEARANCE_M.
	NarrowTurnClearanceM float64 `mapstructure:"narrow_turn_clearance_m"`
	// CenteringGainDegPerM matches CENTERING_GAIN_DEG_PER_M.
	CenteringGainDegPerM float64 `mapstructure:"centering_gain_deg_per_m"`
	// HeadingGain matches HEADING_GAIN.
	HeadingGain float64 `mapstructure:"heading_gain"`
	// MaxCenteringSteerDeg matches MAX_CENTERING_STEER_DEG.
	MaxCenteringSteerDeg float64 `mapstructure:"max_centering_steer_deg"`
	// MaxCornerSteerDeg matches MAX_CORNER_STEER_DEG.
	MaxCornerSteerDeg float64 `mapstructure:"max_corner_steer_deg"`
	// SteerCapFromCommitDistance matches STEER_CAP_FROM_COMMIT_DISTANCE.
	SteerCapFromCommitDistance bool `mapstructure:"steer_cap_from_commit_distance"`
	// CornerSpeedScale matches CORNER_SPEED_SCALE.
	CornerSpeedScale float64 `mapstructure:"corner_speed_scale"`
	// ReverseSpeedScale matches REVERSE_SPEED_SCALE.
	ReverseSpeedScale float64 `mapstructure:"reverse_speed_scale"`
	// TurnArcHalfFovDeg matches TURN_ARC_HALF_FOV_DEG.
	TurnArcHalfFovDeg float64 `mapstructure:"turn_arc_half_fov_deg"`
	// TurnOpenRangeM matches TURN_OPEN_RANGE_M.
	TurnOpenRangeM float64 `mapstructure:"turn_open_range_m"`
	// CornerLeakMarginM matches CORNER_LEAK_MARGIN_M.
	CornerLeakMarginM float64 `mapstructure:"corner_leak_margin_m"`
	// MinReverseClearanceM matches MIN_REVERSE_CLEARANCE_M.
	MinReverseClearanceM float64 `mapstructure:"min_reverse_clearance_m"`
	// BayWallClearanceM matches BAY_WALL_CLEARANCE_M. Not present in the
	// checked-in corridor_follower.toml -- every deployment currently
	// relies on the Pydantic model's own 0.20 default, applied here via
	// LoadWithDefaults rather than silently reading as 0.0.
	BayWallClearanceM float64 `mapstructure:"bay_wall_clearance_m"`
}

// DefaultCorridorFollowerTOMLPath is
// platform/config/navigation/blind_nav/corridor_follower.toml,
// relative to the repo root. No per-component profile overlays -- pass
// nil profileNames to Load.
const DefaultCorridorFollowerTOMLPath = "platform/config/navigation/blind_nav/corridor_follower.toml"

// DefaultBayWallClearanceM matches CorridorFollowerParams.BAY_WALL_CLEARANCE_M's
// Pydantic default -- see CorridorFollowerConfig.BayWallClearanceM's doc
// comment for why this needs a code default at all.
const DefaultBayWallClearanceM = 0.20

// CorridorFollowerDefaults is the LoadWithDefaults defaults map for
// CorridorFollowerConfig.
func CorridorFollowerDefaults() map[string]any {
	return map[string]any{"bay_wall_clearance_m": DefaultBayWallClearanceM}
}
