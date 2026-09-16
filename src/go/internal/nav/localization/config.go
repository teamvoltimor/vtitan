package localization

// Config is the localizer's tuning, matching LocalizationParams plus the
// three sensor-geometry values the search needs from robot.toml.
type Config struct {
	SearchRadiusM         float64
	Passes                int
	GridPoints            int
	ResidualClipM         float64
	MaxSpeedMPS           float64
	JumpConfirmToleranceM float64

	// LidarMountXOffsetM is how far forward of the chassis center the sensor
	// sits. The search predicts from where the SENSOR is, not the body
	// center -- see EstimatePosition.
	LidarMountXOffsetM float64
	LidarMinRangeM     float64
	LidarMaxRangeM     float64

	// RelocalizeCostThreshold/RelocalizeAfterScans/RelocalizeGridStepM/
	// RelocalizeAcceptRatio parameterize global relocalization -- see
	// relocalizeGlobally. See
	// adr:0084-localizer-divergence-and-relocalization.
	RelocalizeCostThreshold float64
	RelocalizeAfterScans    int
	RelocalizeGridStepM     float64
	RelocalizeAcceptRatio   float64
}

// Shipped defaults, matching
// src/config/navigation/blind_nav/localization.toml and
// src/config/robot.toml's [lidar] section.
const (
	// DefaultSearchRadiusM matches localization.toml's search_radius_m.
	DefaultSearchRadiusM = 0.15
	// DefaultPasses matches localization.toml's passes.
	DefaultPasses = 4
	// DefaultGridPoints matches localization.toml's grid_points.
	DefaultGridPoints = 5
	// DefaultResidualClipM matches localization.toml's residual_clip_m.
	DefaultResidualClipM = 0.25
	// DefaultMaxSpeedMPS matches localization.toml's max_speed_mps. Raised
	// from its earlier value because the old bound silently froze pose. See
	// adr:0084-localizer-divergence-and-relocalization.
	DefaultMaxSpeedMPS = 0.60
	// DefaultJumpConfirmToleranceM matches jump_confirm_tolerance_m.
	DefaultJumpConfirmToleranceM = 0.05

	// DefaultLidarMountXOffsetM matches robot.toml's [lidar] mount_x_offset:
	// how far forward of the chassis center the C1 sits.
	DefaultLidarMountXOffsetM = 0.1222
	// DefaultLidarMinRangeM matches robot.toml's [lidar] min_range. The C1
	// reports ~45 mm at closest; nearer is unmeasurable, not clear.
	DefaultLidarMinRangeM = 0.045
	// DefaultLidarMaxRangeM matches robot.toml's [lidar] max_range.
	DefaultLidarMaxRangeM = 12.0

	// DefaultRelocalizeCostThreshold matches localization.toml's
	// relocalize_cost_threshold.
	DefaultRelocalizeCostThreshold = 0.03
	// DefaultRelocalizeAfterScans matches localization.toml's
	// relocalize_after_scans.
	DefaultRelocalizeAfterScans = 15
	// DefaultRelocalizeGridStepM matches localization.toml's
	// relocalize_grid_step_m.
	DefaultRelocalizeGridStepM = 0.03
	// DefaultRelocalizeAcceptRatio matches localization.toml's
	// relocalize_accept_ratio.
	DefaultRelocalizeAcceptRatio = 0.5
)

// DefaultConfig returns the Config matching the shipped TOML defaults.
func DefaultConfig() Config {
	return Config{
		SearchRadiusM:         DefaultSearchRadiusM,
		Passes:                DefaultPasses,
		GridPoints:            DefaultGridPoints,
		ResidualClipM:         DefaultResidualClipM,
		MaxSpeedMPS:           DefaultMaxSpeedMPS,
		JumpConfirmToleranceM: DefaultJumpConfirmToleranceM,
		LidarMountXOffsetM:    DefaultLidarMountXOffsetM,
		LidarMinRangeM:        DefaultLidarMinRangeM,
		LidarMaxRangeM:        DefaultLidarMaxRangeM,

		RelocalizeCostThreshold: DefaultRelocalizeCostThreshold,
		RelocalizeAfterScans:    DefaultRelocalizeAfterScans,
		RelocalizeGridStepM:     DefaultRelocalizeGridStepM,
		RelocalizeAcceptRatio:   DefaultRelocalizeAcceptRatio,
	}
}
