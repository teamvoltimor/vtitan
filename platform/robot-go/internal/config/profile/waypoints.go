package profile

// WaypointsConfig mirrors the subset of
// platform/shared/config/navigation/waypoint/waypoints.toml
// (shared.config.navigation_tuning.waypoint.WaypointParams) that
// internal/nav/waypoints currently consumes. ArcRadius/
// MainLoopReachedDistanceM/ControllerReachedDistanceM/
// ReplanHeadingTieMarginM belong to calculate_waypoints and the
// navigator/controller consumers, none ported to Go yet, so they're
// omitted here rather than mirrored unused. WideCenterBiasSide/
// NarrowCenterBiasSide stay strings -- viper/mapstructure has no decode
// hook for trackmodel.CorridorSide's "inner"/"outer" TOML values, so
// internal/nav/waypoints.ConfigFor parses them itself.
type WaypointsConfig struct {
	// DedupeDistanceM matches DEDUPE_DISTANCE_M.
	DedupeDistanceM float64 `mapstructure:"dedupe_distance_m"`
	// WideCenterBiasM/WideCenterBiasSide match WIDE_CENTER_BIAS_M/
	// WIDE_CENTER_BIAS_SIDE.
	WideCenterBiasM    float64 `mapstructure:"wide_center_bias_m"`
	WideCenterBiasSide string  `mapstructure:"wide_center_bias_side"`
	// NarrowCenterBiasM/NarrowCenterBiasSide match NARROW_CENTER_BIAS_M/
	// NARROW_CENTER_BIAS_SIDE.
	NarrowCenterBiasM    float64 `mapstructure:"narrow_center_bias_m"`
	NarrowCenterBiasSide string  `mapstructure:"narrow_center_bias_side"`
	// NarrowWidthThresholdM matches NARROW_WIDTH_THRESHOLD_M.
	NarrowWidthThresholdM float64 `mapstructure:"narrow_width_threshold_m"`
	// NumIntermediateArcPoints matches NUM_INTERMEDIATE_ARC_POINTS.
	NumIntermediateArcPoints int `mapstructure:"num_intermediate_arc_points"`
	// StraightWaypointCount matches STRAIGHT_WAYPOINT_COUNT.
	StraightWaypointCount int `mapstructure:"straight_waypoint_count"`
}

// DefaultWaypointsTOMLPath is
// platform/shared/config/navigation/waypoint/waypoints.toml, relative to
// the repo root. No per-component profile overlays -- pass nil
// profileNames to Load.
const DefaultWaypointsTOMLPath = "platform/shared/config/navigation/waypoint/waypoints.toml"
