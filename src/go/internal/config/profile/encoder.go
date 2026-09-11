package profile

import "fmt"

// EncoderConfig mirrors the subset of
// src/config/hardware/motors/encoder.toml
// (src/hardware/motors/encoder/config.py) that robot-go consumes: the A/B
// GPIO pins and the bench-calibrated counts per wheel revolution.
//
// The PID gains, max_rpm and feedforward_deadband_duty in that file are
// deliberately absent -- they configure Python's closed-loop speed
// controller, and the Go motor node drives open-loop (see
// internal/node/motor.SpeedToNormalized). Adding fields robot-go does not
// read would imply a loop that does not exist.
//
// wheel_diameter_m is absent for the same reason it is absent from the
// Python model: it derives from robot.toml's wheel radius (RobotWheel), and
// a second independent literal here could drift from it.
type EncoderConfig struct {
	PinA int `mapstructure:"pin_a"`
	PinB int `mapstructure:"pin_b"`
	// CountsPerRev is per-motor bench calibration with NO default, supplied
	// by a motor profile overlay. Zero means "no motor profile is active",
	// which LoadEncoderConfig rejects rather than dividing by -- see the
	// TOML's own note on why a stale shared default silently misconfigured
	// whichever motor was connected.
	CountsPerRev float64 `mapstructure:"counts_per_rev"`
}

// DefaultEncoderTOMLPath is
// src/config/hardware/motors/encoder.toml, relative to the repo
// root.
const DefaultEncoderTOMLPath = "src/config/hardware/motors/encoder.toml"

// LoadEncoderConfig loads EncoderConfig from basePath overlaid with
// profileNames (see Load), then rejects a missing counts_per_rev the way
// Python's required field does: an error naming the active profiles, not a
// zero that turns every distance into a division by zero.
func LoadEncoderConfig(basePath string, profileNames []string) (*EncoderConfig, error) {
	cfg, err := Load[EncoderConfig](basePath, profileNames)
	if err != nil {
		return nil, err
	}
	if cfg.CountsPerRev <= 0 {
		return nil, fmt.Errorf(
			"profile: encoder.toml requires counts_per_rev from an active motor profile (%s); active profiles: %v",
			EnvVar,
			profileNames,
		)
	}
	return cfg, nil
}
