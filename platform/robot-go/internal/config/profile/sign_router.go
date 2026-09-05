package profile

// SignRouterConfig mirrors
// platform/config/navigation/signs/sign_router.toml
// (shared.config.navigation_tuning.signs.SignRouterParams) restricted to
// the fields internal/nav/signrouter currently consumes -- the lane-planner
// (SIGN_LANE_*), escape-mask, retrace and sign-contact-evade knobs belong to
// navigation logic not ported to Go yet, so they're omitted here rather
// than mirrored unused.
type SignRouterConfig struct {
	// SignClearanceMarginM matches SIGN_CLEARANCE_MARGIN_M.
	SignClearanceMarginM float64 `mapstructure:"sign_clearance_margin_m"`
	// WallClearanceMarginM matches WALL_CLEARANCE_MARGIN_M.
	WallClearanceMarginM float64 `mapstructure:"wall_clearance_margin_m"`
	// DeformDepthBufferM matches DEFORM_DEPTH_BUFFER_M.
	DeformDepthBufferM float64 `mapstructure:"deform_depth_buffer_m"`
	// ActivationDistM matches ACTIVATION_DIST_M.
	ActivationDistM float64 `mapstructure:"activation_dist_m"`
	// PassedDistM matches PASSED_DIST_M.
	PassedDistM float64 `mapstructure:"passed_dist_m"`
	// DetectionMatchDistM matches DETECTION_MATCH_DIST_M.
	DetectionMatchDistM float64 `mapstructure:"detection_match_dist_m"`
	// MinConfidence matches MIN_CONFIDENCE.
	MinConfidence float64 `mapstructure:"min_confidence"`
	// SettleTicks matches SETTLE_TICKS.
	SettleTicks int `mapstructure:"settle_ticks"`
	// CommitHysteresis matches COMMIT_HYSTERESIS.
	CommitHysteresis bool `mapstructure:"commit_hysteresis"`
	// CorridorFlipTicks matches CORRIDOR_FLIP_TICKS.
	CorridorFlipTicks int `mapstructure:"corridor_flip_ticks"`

	// DepthPin/PinCornerGuard/PinHeadingGuard/PinHeadingGuardDeg match
	// DEPTH_PIN/PIN_CORNER_GUARD/PIN_HEADING_GUARD/PIN_HEADING_GUARD_DEG.
	// None of the four are present in the checked-in sign_router.toml --
	// every deployment currently relies on the Pydantic model's own
	// defaults (True/True/True/35.0), applied here via LoadWithDefaults
	// rather than silently reading as false/0.0.
	DepthPin           bool    `mapstructure:"depth_pin"`
	PinCornerGuard     bool    `mapstructure:"pin_corner_guard"`
	PinHeadingGuard    bool    `mapstructure:"pin_heading_guard"`
	PinHeadingGuardDeg float64 `mapstructure:"pin_heading_guard_deg"`

	// RelabelUnsatisfiable/DepthConsistentCorridor match
	// SIGN_LANE_RELABEL_UNSATISFIABLE/SIGN_LANE_DEPTH_CONSISTENT_CORRIDOR.
	// Also absent from the checked-in TOML -- both Pydantic-default True,
	// applied via LoadWithDefaults for the same reason as the pin guards
	// above. Named for the corridor label they affect (not "lane", despite
	// the SIGN_LANE_ prefix): SignRouter._corridor_for_spec/
	// _geometric_corridor consume both outside the lane planner too.
	RelabelUnsatisfiable    bool `mapstructure:"sign_lane_relabel_unsatisfiable"`
	DepthConsistentCorridor bool `mapstructure:"sign_lane_depth_consistent_corridor"`
}

// DefaultSignRouterTOMLPath is
// platform/config/navigation/signs/sign_router.toml, relative to the
// repo root. No per-component profile overlays -- pass nil profileNames to
// Load/LoadWithDefaults.
const DefaultSignRouterTOMLPath = "platform/config/navigation/signs/sign_router.toml"

// Default* match SignRouterParams' Pydantic defaults for the four fields
// missing from the checked-in sign_router.toml -- see SignRouterConfig's
// doc comment.
const (
	DefaultDepthPin                = true
	DefaultPinCornerGuard          = true
	DefaultPinHeadingGuard         = true
	DefaultPinHeadingGuardDeg      = 35.0
	DefaultRelabelUnsatisfiable    = true
	DefaultDepthConsistentCorridor = true
)

// SignRouterDefaults is the LoadWithDefaults defaults map for
// SignRouterConfig.
func SignRouterDefaults() map[string]any {
	return map[string]any{
		"depth_pin":                           DefaultDepthPin,
		"pin_corner_guard":                    DefaultPinCornerGuard,
		"pin_heading_guard":                   DefaultPinHeadingGuard,
		"pin_heading_guard_deg":               DefaultPinHeadingGuardDeg,
		"sign_lane_relabel_unsatisfiable":     DefaultRelabelUnsatisfiable,
		"sign_lane_depth_consistent_corridor": DefaultDepthConsistentCorridor,
	}
}
