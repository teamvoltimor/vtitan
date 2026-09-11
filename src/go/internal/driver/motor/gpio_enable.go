//go:build linux

package motor

import (
	"context"
	"fmt"

	"github.com/warthog618/go-gpiocdev"
)

const (
	// gpioLow/gpioHigh are the raw line-value integers go-gpiocdev's
	// SetValue expects.
	gpioLow  = 0
	gpioHigh = 1
)

// gpioEnableLine is the real enableWriter (controller.go) backed by a
// go-gpiocdev output line. Used for both R_EN and L_EN — see
// docs/bts7960-ibt2-wiring.md for why both are held permanently HIGH once
// Controller.Connect enables them.
type gpioEnableLine struct {
	line *gpiocdev.Line
}

var _ enableWriter = (*gpioEnableLine)(nil)

// newGPIOEnableLine requests offset on chip as a digital output, initially
// driven LOW. The line is requested LOW deliberately — Controller.Connect
// is what raises it HIGH, once (and only once) both PWM channels are
// confirmed at 0 duty; see controller.go's Connect doc comment.
func newGPIOEnableLine(chip string, offset int) (*gpioEnableLine, error) {
	line, err := gpiocdev.RequestLine(chip, offset, gpiocdev.AsOutput(gpioLow))
	if err != nil {
		return nil, fmt.Errorf("motor: requesting GPIO line %s:%d: %w", chip, offset, err)
	}
	return &gpioEnableLine{line: line}, nil
}

// SetHigh drives the line HIGH or LOW.
func (g *gpioEnableLine) SetHigh(_ context.Context, high bool) error {
	value := gpioLow
	if high {
		value = gpioHigh
	}
	if err := g.line.SetValue(value); err != nil {
		return fmt.Errorf("motor: setting GPIO line value: %w", err)
	}
	return nil
}

// Close releases the underlying GPIO line. Note this does not by itself
// guarantee the physical pin stays LOW afterward — a released line-request
// floats, and this board's level shifter reads a float as HIGH (full-speed
// reverse). See driver.go's Close for the best-effort mitigation and
// project history for why the physical pull-down resistor on LPWM remains
// the only complete fix.
func (g *gpioEnableLine) Close() error {
	if g.line == nil {
		return nil
	}
	if err := g.line.Close(); err != nil {
		return fmt.Errorf("motor: closing GPIO line: %w", err)
	}
	return nil
}
