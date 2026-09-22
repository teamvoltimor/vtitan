package stateestimator

// Config is the heading-fusion tuning, matching
// src/config/navigation/blind_nav/state_estimator.toml and Python's
// EstimatorConstants.
type Config struct {
	// YawCorrectionGain is the fraction of the wall-heading discrepancy folded
	// into the estimate per update. Deliberately small: a hard assignment would
	// inject the wall estimate's per-scan noise straight into steering at the
	// scan rate and discard the IMU's short-term accuracy. See
	// adr:0054-absolute-heading-from-walls.
	YawCorrectionGain float64
}

// DefaultYawCorrectionGain matches state_estimator.toml's yaw_correction_gain.
const DefaultYawCorrectionGain = 0.05

// DefaultConfig returns the Config matching the shipped TOML.
func DefaultConfig() Config {
	return Config{YawCorrectionGain: DefaultYawCorrectionGain}
}
