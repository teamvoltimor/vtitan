package encoder

import (
	"fmt"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/config/profile"
)

// Config is the encoder's physical wiring plus the two calibrated numbers
// every derived quantity scales from. Every field is required: unlike the
// button or motor drivers there is no defensible fallback here, since a
// guessed CountsPerRev or WheelDiameterM produces confident, wrong
// distances rather than an obvious failure.
type Config struct {
	GPIOChip string
	// PinA/PinB are BCM line offsets for the encoder's A and B channels.
	PinA int
	PinB int
	// CountsPerRev is bench-calibrated for the specific motor, from
	// encoder.toml's active motor profile overlay. See Decoder.CountsPerEdge
	// for why it must be re-measured against THIS decoder.
	CountsPerRev float64
	// WheelDiameterM derives from robot.toml's wheel radius, matching
	// Python's calibration.DEFAULT_WHEEL_DIAMETER_M.
	WheelDiameterM float64
	// Invert flips the count's sign into the command frame. Independent of
	// a paired drive's own invert: the motor leads and the encoder's A/B
	// channels are separate connections, so swapping one does not swap the
	// other.
	Invert bool
}

// DefaultGPIOChip is the character device every GPIO consumer on this board
// requests lines against, matching button.DefaultGPIOChip and the motor
// driver's own chip. There are deliberately no default pins, CountsPerRev
// or WheelDiameterM -- see Config.
const DefaultGPIOChip = "gpiochip0"

// Validate rejects a Config that cannot produce meaningful odometry.
func (c Config) Validate() error {
	switch {
	case c.GPIOChip == "":
		return fmt.Errorf("encoder: GPIOChip is required")
	case c.PinA < 0 || c.PinB < 0:
		return fmt.Errorf("encoder: pins must be non-negative, got A=%d B=%d", c.PinA, c.PinB)
	case c.PinA == c.PinB:
		return fmt.Errorf("encoder: pins A and B must differ, both are %d", c.PinA)
	case c.CountsPerRev <= 0:
		return errCountsPerRevPositive
	case c.WheelDiameterM <= 0:
		return fmt.Errorf("encoder: WheelDiameterM must be positive, got %g", c.WheelDiameterM)
	}
	return nil
}

// ConfigFor resolves the Config to Connect with, from configRoot's
// encoder.toml (pins, counts_per_rev) and robot.toml (wheel radius), both
// overlaid with the profiles named in profile.ActiveNames().
//
// Unlike button.ConfigFor this returns an error rather than falling back to
// literals: counts_per_rev has no shared default anywhere in the stack by
// design, and inventing one here would reintroduce exactly the silent
// cross-motor misconfiguration encoder.toml's split was made to prevent.
func ConfigFor(configRoot string) (Config, error) {
	if configRoot == "" {
		return Config{}, fmt.Errorf("encoder: a config root is required to load encoder.toml")
	}

	names := profile.ActiveNames()
	encCfg, err := profile.LoadEncoderConfig(
		filepath.Join(configRoot, filepath.FromSlash(profile.DefaultEncoderTOMLPath)),
		names,
	)
	if err != nil {
		return Config{}, fmt.Errorf("encoder: loading encoder.toml: %w", err)
	}
	robotCfg, err := profile.LoadRobotConfig(
		filepath.Join(configRoot, filepath.FromSlash(profile.DefaultRobotTOMLPath)),
		names,
	)
	if err != nil {
		return Config{}, fmt.Errorf("encoder: loading robot.toml: %w", err)
	}

	cfg := Config{
		GPIOChip:       DefaultGPIOChip,
		PinA:           encCfg.PinA,
		PinB:           encCfg.PinB,
		CountsPerRev:   encCfg.CountsPerRev,
		WheelDiameterM: robotCfg.Wheel.Radius * 2,
	}
	if validateErr := cfg.Validate(); validateErr != nil {
		return Config{}, validateErr
	}
	return cfg, nil
}
