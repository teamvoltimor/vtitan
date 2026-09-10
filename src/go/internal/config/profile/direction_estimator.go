package profile

// DirectionEstimatorConfig mirrors the subset of
// platform/shared/config/navigation/blind_nav/direction_estimator.toml
// (shared.config.navigation_tuning.blind_nav.DirectionEstimatorParams)
// that internal/nav/directionestimator currently consumes.
// CornerClearanceM/GateLogPeriodTicks belong to corridor_follower.py's
// blind-turn logic and track_navigator_node's diagnostic logging, neither
// ported to Go yet, so they're omitted here rather than mirrored unused.
type DirectionEstimatorConfig struct {
	// AlignmentToleranceRad matches ALIGNMENT_TOLERANCE_RAD.
	AlignmentToleranceRad float64 `mapstructure:"alignment_tolerance_rad"`
	// MaxInTrackRangeM matches MAX_IN_TRACK_RANGE_M.
	MaxInTrackRangeM float64 `mapstructure:"max_in_track_range_m"`
	// MinAsymmetryM matches MIN_ASYMMETRY_M.
	MinAsymmetryM float64 `mapstructure:"min_asymmetry_m"`
	// PlausibleSpanThresholdM matches PLAUSIBLE_SPAN_THRESHOLD_M.
	PlausibleSpanThresholdM float64 `mapstructure:"plausible_span_threshold_m"`
	// MinVotes matches MIN_VOTES.
	MinVotes int `mapstructure:"min_votes"`
}

// DefaultDirectionEstimatorTOMLPath is
// platform/shared/config/navigation/blind_nav/direction_estimator.toml,
// relative to the repo root. No per-component profile overlays -- pass
// nil profileNames to Load.
const DefaultDirectionEstimatorTOMLPath = "platform/shared/config/navigation/blind_nav/direction_estimator.toml"
