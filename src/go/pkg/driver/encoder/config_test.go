package encoder_test

import (
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/pkg/driver/encoder"
)

// shippedCountsPerRev is the live-verified calibration for the current motor
// (see adr:0076-drivetrain-and-steering-hardware).
const shippedCountsPerRev = 60.0

// shippedWheelDiameterM is robot.toml's wheel radius (0.035) doubled, the
// same derivation Python's calibration.DEFAULT_WHEEL_DIAMETER_M makes.
const shippedWheelDiameterM = 0.07

func TestConfig_ValidateRejectsIncompleteWiring(t *testing.T) {
	t.Parallel()

	valid := encoder.Config{
		GPIOChip:       encoder.DefaultGPIOChip,
		PinA:           16,
		PinB:           20,
		CountsPerRev:   shippedCountsPerRev,
		WheelDiameterM: shippedWheelDiameterM,
	}
	if err := valid.Validate(); err != nil {
		t.Fatalf("valid config: %v", err)
	}

	for _, tc := range []struct {
		name   string
		mutate func(*encoder.Config)
	}{
		{"no chip", func(c *encoder.Config) { c.GPIOChip = "" }},
		{"same pin twice", func(c *encoder.Config) { c.PinB = c.PinA }},
		{"negative pin", func(c *encoder.Config) { c.PinA = -1 }},
		{"no calibration", func(c *encoder.Config) { c.CountsPerRev = 0 }},
		{"no wheel diameter", func(c *encoder.Config) { c.WheelDiameterM = 0 }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()

			cfg := valid
			tc.mutate(&cfg)
			if err := cfg.Validate(); err == nil {
				t.Fatal("want error, got nil")
			}
		})
	}
}
