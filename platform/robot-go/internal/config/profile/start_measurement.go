package profile

// StartMeasurementConfig mirrors
// platform/shared/config/navigation/sensors/start_measurement.toml
// (shared.config.navigation_tuning.StartMeasurementParams), the parameters
// for internal/nav/startmeasurement's scan-derived starting pose.
type StartMeasurementConfig struct {
	// RayHalfWidthDeg matches ray_half_width_deg: the half-angle of each
	// cardinal wedge the medians are taken over.
	RayHalfWidthDeg float64 `mapstructure:"ray_half_width_deg"`
	// ClosingToleranceM matches closing_tolerance_m: how far forward+back
	// may fall short of the mat before the reading is rejected.
	ClosingToleranceM float64 `mapstructure:"closing_tolerance_m"`
	// RetryWindowS and RetryAlignToleranceDeg are shipped in the TOML but
	// not consumed by internal/nav/startmeasurement itself, matching
	// start_measurement.py -- they belong to a retry loop around the
	// measurement, which is the caller's responsibility.
	RetryWindowS           float64 `mapstructure:"retry_window_s"`
	RetryAlignToleranceDeg float64 `mapstructure:"retry_align_tolerance_deg"`
}

// DefaultStartMeasurementTOMLPath is
// platform/shared/config/navigation/sensors/start_measurement.toml,
// relative to the repo root. No per-component profile overlays -- pass nil
// profileNames to Load.
const DefaultStartMeasurementTOMLPath = "platform/shared/config/navigation/sensors/start_measurement.toml"
