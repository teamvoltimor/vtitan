package profile

// LocalizationConfig mirrors
// platform/shared/config/navigation/blind_nav/localization.toml
// (shared.config.navigation_tuning.blind_nav.LocalizationParams), the search
// parameters for internal/nav/localization's LidarLocalizer.
type LocalizationConfig struct {
	// SearchRadiusM matches SEARCH_RADIUS_M: half-width of the initial pose
	// search window. Sized for search robustness, NOT as a physical
	// displacement bound -- see MaxSpeedMPS for the guard that does bound it.
	SearchRadiusM float64 `mapstructure:"search_radius_m"`
	// Passes matches PASSES: coarse-to-fine grid-search passes.
	Passes int `mapstructure:"passes"`
	// GridPoints matches GRID_POINTS: candidates per axis per pass.
	GridPoints int `mapstructure:"grid_points"`
	// ResidualClipM matches RESIDUAL_CLIP_M: per-ray residual clipping for
	// outlier rejection.
	ResidualClipM float64 `mapstructure:"residual_clip_m"`
	// MaxSpeedMPS matches MAX_SPEED_MPS: the implausible-jump speed bound.
	//
	// Was 0.25, sized against the RETIRED motor's 0.156 m/s ceiling. The
	// current drivetrain measures ~0.58 m/s at max_duty=0.5, so that value
	// sat BELOW real operating speed and silently froze pose whenever the
	// robot drove quickly. Worse, pose was being used as the independent
	// check on encoder distance, and both were suppressed in the same
	// direction -- two suppressed measurements agreeing is not
	// corroboration. Raise this whenever the drivetrain gets faster.
	MaxSpeedMPS float64 `mapstructure:"max_speed_mps"`
	// JumpConfirmToleranceM matches JUMP_CONFIRM_TOLERANCE_M: how close two
	// consecutive rejected candidates must be to count as one correction
	// confirming itself.
	JumpConfirmToleranceM float64 `mapstructure:"jump_confirm_tolerance_m"`
}

// DefaultLocalizationTOMLPath is
// platform/shared/config/navigation/blind_nav/localization.toml, relative to
// the repo root. No per-component profile overlays -- pass nil profileNames
// to Load.
const DefaultLocalizationTOMLPath = "platform/shared/config/navigation/blind_nav/localization.toml"
