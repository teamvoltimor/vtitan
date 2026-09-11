package profile

// CorridorEstimatorConfig mirrors
// src/config/navigation/blind_nav/corridor_estimator.toml
// (shared.config.navigation_tuning.blind_nav.CorridorEstimatorParams) in
// full.
type CorridorEstimatorConfig struct {
	// MinSamples matches MIN_SAMPLES -- readings a corridor needs before
	// its width is believed.
	MinSamples int `mapstructure:"min_samples"`
	// PlausibleWidthMarginM matches PLAUSIBLE_WIDTH_MARGIN_M -- how far
	// outside the two legal widths a reading may fall and still be taken.
	PlausibleWidthMarginM float64 `mapstructure:"plausible_width_margin_m"`
	// MaxStartSamples matches MAX_START_SAMPLES -- the cap on the
	// creep-phase width buffer, which is taken before any direction exists
	// to file the readings under.
	MaxStartSamples int `mapstructure:"max_start_samples"`
	// DecisionBoundaryM matches DECISION_BOUNDARY_M -- the width a reading
	// is snapped to NARROW below and WIDE above.
	DecisionBoundaryM float64 `mapstructure:"decision_boundary_m"`
}

// DefaultCorridorEstimatorTOMLPath is
// src/config/navigation/blind_nav/corridor_estimator.toml,
// relative to the repo root. No per-component profile overlays -- pass nil
// profileNames to Load.
const DefaultCorridorEstimatorTOMLPath = "src/config/navigation/blind_nav/corridor_estimator.toml"
