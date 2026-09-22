package encoder

import (
	"errors"
	"fmt"
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
	// encoder.toml's active motor profile overlay. See quadrature.CountsPerEdge
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

// errCountsPerRevPositive mirrors control.py's _CPR_POSITIVE guard for the
// wiring config; quadrature.ErrCountsPerRevPositive is the same rule inside
// the conversions themselves.
var errCountsPerRevPositive = errors.New("encoder: counts_per_rev must be positive")

// Validate rejects a Config that cannot produce meaningful odometry.
func (c Config) Validate() error {
	switch {
	case c.GPIOChip == "":
		return errors.New("encoder: GPIOChip is required")
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
