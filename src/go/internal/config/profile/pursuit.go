package profile

// PursuitConfig mirrors
// src/config/navigation/motion/pursuit.toml
// (shared.config.navigation_tuning.motion.PurePursuitParams) in full.
// WallMarginSafetyM/MinLookaheadTransitionM are mirrored even though
// internal/nav/controllers.WaypointController does not (yet) derive a
// crosstrack budget the way CoreNavigator does -- kept for completeness
// and to keep this struct a faithful 1:1 mirror of the TOML.
type PursuitConfig struct {
	// LookaheadShort matches LOOKAHEAD_SHORT -- corner lookahead (m).
	LookaheadShort float64 `mapstructure:"lookahead_short"`
	// LookaheadLong matches LOOKAHEAD_LONG -- straight lookahead (m).
	LookaheadLong float64 `mapstructure:"lookahead_long"`
	// OpenLookaheadLong matches OPEN_LOOKAHEAD_LONG -- the Open Challenge's
	// own straight lookahead (m), which REPLACES LookaheadLong on an Open
	// run. Obstacles is untouched, so this cannot shadow the base constant
	// on an Obstacles sweep.
	OpenLookaheadLong float64 `mapstructure:"open_lookahead_long"`
	// LookaheadTransition matches LOOKAHEAD_TRANSITION -- crosstrack
	// threshold ceiling (m).
	LookaheadTransition float64 `mapstructure:"lookahead_transition"`
	// LookaheadBlendStart matches LOOKAHEAD_BLEND_START.
	LookaheadBlendStart float64 `mapstructure:"lookahead_blend_start"`
	// SteerKp matches STEER_KP -- no longer consumed by compute_steering
	// (see WaypointController's doc comment), kept only so the field has
	// somewhere to land.
	SteerKp float64 `mapstructure:"steer_kp"`
	// MaxSteeringRate matches MAX_STEERING_RATE (rad/s).
	MaxSteeringRate float64 `mapstructure:"max_steering_rate"`
	// WallMarginSafetyM matches WALL_MARGIN_SAFETY_M (m).
	WallMarginSafetyM float64 `mapstructure:"wall_margin_safety_m"`
	// MinLookaheadTransitionM matches MIN_LOOKAHEAD_TRANSITION_M (m).
	MinLookaheadTransitionM float64 `mapstructure:"min_lookahead_transition_m"`
	// CornerPreviewDistanceM matches CORNER_PREVIEW_DISTANCE_M (m).
	CornerPreviewDistanceM float64 `mapstructure:"corner_preview_distance_m"`
	// CornerTurnThresholdRad matches CORNER_TURN_THRESHOLD_RAD (rad).
	CornerTurnThresholdRad float64 `mapstructure:"corner_turn_threshold_rad"`
	// YawGainCompensation matches YAW_GAIN_COMPENSATION -- the base/Open
	// value. ObstaclesYawGainCompensation matches
	// OBSTACLES_YAW_GAIN_COMPENSATION and replaces it on an Obstacles run.
	YawGainCompensation          float64 `mapstructure:"yaw_gain_compensation"`
	ObstaclesYawGainCompensation float64 `mapstructure:"obstacles_yaw_gain_compensation"`
}

// DefaultPursuitTOMLPath is
// src/config/navigation/motion/pursuit.toml, relative to the
// repo root. No per-component profile overlays -- pass nil profileNames to
// Load.
const DefaultPursuitTOMLPath = "src/config/navigation/motion/pursuit.toml"
