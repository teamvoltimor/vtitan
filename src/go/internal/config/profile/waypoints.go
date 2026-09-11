package profile

// WaypointsConfig mirrors the subset of
// src/config/navigation/waypoint/waypoints.toml
// (shared.config.navigation_tuning.waypoint.WaypointParams) that
// internal/nav/waypoints and internal/nav/controllers currently consume.
// MainLoopReachedDistanceM/ReplanHeadingTieMarginM belong to the navigator
// consumer, not this package, so they're omitted here rather than mirrored
// unused (see internal/nav/navigator's own navWaypointsTOML).
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
	// CornerArcAssumeWide matches CORNER_ARC_ASSUME_WIDE -- when true,
	// CalculateWaypoints sizes every corner as if both corridors were WIDE
	// (see waypoints.Config.CornerArcAssumeWide), so the turn-entry point is
	// independent of a corridor-width belief that starts out wrong. Blind
	// rounds begin believing every corridor narrow, so without this a
	// narrow->wide corner plans a late entry.
	CornerArcAssumeWide bool `mapstructure:"corner_arc_assume_wide"`
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
	// ObstaclesCenterBiasM matches OBSTACLES_CENTER_BIAS_M -- the flat
	// Obstacles Challenge override, applied uniformly instead of the
	// Wide/Narrow split. See waypoints.Config.ObstaclesCenterBiasM.
	ObstaclesCenterBiasM float64 `mapstructure:"obstacles_center_bias_m"`
	// NarrowWidthThresholdM matches NARROW_WIDTH_THRESHOLD_M.
	NarrowWidthThresholdM float64 `mapstructure:"narrow_width_threshold_m"`
	// UnconfirmedWidthInnerBiasM matches UNCONFIRMED_WIDTH_INNER_BIAS_M --
	// the inner bias a narrow corridor takes while its width is still the
	// blind prior rather than a measurement. See
	// waypoints.Config.UnconfirmedWidthInnerBiasM.
	UnconfirmedWidthInnerBiasM float64 `mapstructure:"unconfirmed_width_inner_bias_m"`
	// DeferCurrentCorridorReplan matches DEFER_CURRENT_CORRIDOR_REPLAN --
	// hold a width change back until the robot has left the corridor it
	// describes. See internal/nav/widthbelief.
	DeferCurrentCorridorReplan bool `mapstructure:"defer_current_corridor_replan"`
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
// src/config/navigation/waypoint/waypoints.toml, relative to
// the repo root. No per-component profile overlays -- pass nil
// profileNames to Load.
const DefaultWaypointsTOMLPath = "src/config/navigation/waypoint/waypoints.toml"
