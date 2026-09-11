package profile

// WallHeadingConfig mirrors
// src/config/navigation/sensors/wall_heading.toml
// (shared.config.navigation_tuning.WallHeadingParams), the parameters for
// internal/nav/wallheading's absolute-heading estimate.
type WallHeadingConfig struct {
	// MinConcentration matches MIN_CONCENTRATION: how aligned the fitted
	// segment directions must be before the estimate is trusted. Low
	// concentration means the returns disagree about where the wall runs --
	// what a corner, a traffic sign or an open side looks like -- and there
	// the heading reference is worse than none.
	MinConcentration float64 `mapstructure:"min_concentration"`
	// BaselineRays matches BASELINE_RAYS: how far apart, in rays, the two
	// returns forming one segment are taken. Wider is less sensitive to
	// per-ray range noise but blurs genuine corners into the straight it is
	// trying to measure.
	BaselineRays int `mapstructure:"baseline_rays"`
	// MaxSegmentJumpM matches MAX_SEGMENT_JUMP_M: the range step above which
	// two returns are different surfaces rather than one continuous wall.
	MaxSegmentJumpM float64 `mapstructure:"max_segment_jump_m"`
	// MinSegmentM matches MIN_SEGMENT_M: segments shorter than this are
	// dominated by range noise rather than wall direction.
	MinSegmentM float64 `mapstructure:"min_segment_m"`
	// NearMaxRangeM matches NEAR_MAX_RANGE_M: returns at or beyond this are
	// no-return rays sanitized to max range by the driver, not real
	// surfaces. The C1 reports 12 m; this sits just under it.
	NearMaxRangeM float64 `mapstructure:"near_max_range_m"`
	// MinReturns matches MIN_RETURNS: fewer usable returns than this cannot
	// form a segment at all.
	MinReturns int `mapstructure:"min_returns"`
}

// DefaultWallHeadingTOMLPath is
// src/config/navigation/sensors/wall_heading.toml, relative to
// the repo root. No per-component profile overlays -- pass nil profileNames
// to Load.
const DefaultWallHeadingTOMLPath = "src/config/navigation/sensors/wall_heading.toml"
