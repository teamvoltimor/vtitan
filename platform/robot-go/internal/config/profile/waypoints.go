package profile

// WaypointsConfig mirrors the subset of
// platform/shared/config/navigation/waypoint/waypoints.toml
// (shared.config.navigation_tuning.waypoint.WaypointParams) that
// internal/nav/waypoints and internal/nav/controllers currently consume.
// ArcRadius/MainLoopReachedDistanceM/ReplanHeadingTieMarginM belong to
// calculate_waypoints and the navigator consumer, not ported to Go yet, so
// they're omitted here rather than mirrored unused.
// ControllerReachedDistanceM IS mirrored -- see its own doc comment for why
// it's consumed by internal/nav/controllers rather than this package.
// WideCenterBiasSide/NarrowCenterBiasSide stay strings -- viper/mapstructure
// has no decode hook for trackmodel.CorridorSide's "inner"/"outer" TOML
// values, so internal/nav/waypoints.ConfigFor parses them itself.
type WaypointsConfig struct {
	// ArcRadius matches ARC_RADIUS -- the corner-arc ceiling (m). Consumed by
	// internal/nav/parking as the staging-point clearance in front of the bay
	// opening (the Python park controller reads tuning.waypoints.ARC_RADIUS
	// for the same purpose), even though WaypointController's own polyline
	// generation is not ported to Go yet.
	ArcRadius float64 `mapstructure:"arc_radius"`
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
	// ControllerReachedDistanceM matches CONTROLLER_REACHED_DISTANCE_M --
	// consumed by internal/nav/controllers.Config, not by this package
	// (WaypointsConfig only mirrors the TOML; CONTROLLER_REACHED_DISTANCE_M
	// belongs conceptually to WaypointController, sourced from the same file).
	ControllerReachedDistanceM float64 `mapstructure:"controller_reached_distance_m"`
}

// DefaultWaypointsTOMLPath is
// platform/shared/config/navigation/waypoint/waypoints.toml, relative to
// the repo root. No per-component profile overlays -- pass nil
// profileNames to Load.
const DefaultWaypointsTOMLPath = "platform/shared/config/navigation/waypoint/waypoints.toml"
