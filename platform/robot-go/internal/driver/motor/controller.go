package motor

import (
	"context"
	"errors"
	"fmt"
	"sync"
)

// dutyWriter is the narrow contract splitDuty's output is written through.
// Both the RPWM hardware-PWM sysfs channel (sysfs_pwm.go) and the LPWM
// software-PWM channel (soft_pwm.go) implement it identically from
// Controller's point of view — it never needs to know which physical
// channel it's talking to. Defined here, where it's consumed, per
// go-architect §4.
type dutyWriter interface {
	// SetDuty writes a duty fraction in [0, 1] to the channel.
	SetDuty(ctx context.Context, fraction float64) error
}

// enableWriter is the narrow contract for driving a single digital output
// line HIGH or LOW — implemented by the R_EN/L_EN GPIO lines
// (gpio_enable.go).
type enableWriter interface {
	SetHigh(ctx context.Context, high bool) error
}

// Actuator is the actuator-shaped counterpart to driver.Driver[T]
// (platform/robot-go/internal/driver): commanded via SetSpeed rather than
// sampled via Read. Driver (driver.go) is its only implementation today.
type Actuator interface {
	Connect(ctx context.Context) error
	SetSpeed(ctx context.Context, normalizedSpeed float64) error
	Close() error
}

// Controller holds the pure, hardware-independent BTS7960 drive logic: the
// safety-critical connect ordering and the signed-duty-to-channel-duty
// sequencing. It depends only on the small dutyWriter/enableWriter
// interfaces above, so it is fully unit-testable with fakes that record
// what was written and in what order — see controller_test.go — without any
// real GPIO or /sys/class/pwm access. driver.go wires it to real hardware
// adapters.
type Controller struct {
	mu sync.Mutex

	rpwm dutyWriter
	lpwm dutyWriter
	rEn  enableWriter
	lEn  enableWriter
	sign float64

	connected bool
}

// errNotConnected is returned by SetSpeed when called before Connect has
// completed successfully.
var errNotConnected = errors.New("motor: SetSpeed called before Connect")

// NewController builds a Controller from its four channel dependencies.
// invert flips the sign convention of SetSpeed's input, matching
// Bts7960PwmConfig's invert flag on the Python side.
func NewController(rpwm, lpwm dutyWriter, rEn, lEn enableWriter, invert bool) *Controller {
	sign := 1.0
	if invert {
		sign = -1.0
	}
	return &Controller{rpwm: rpwm, lpwm: lpwm, rEn: rEn, lEn: lEn, sign: sign}
}

// Connect opens the H-bridge: both PWM channels are driven to 0 duty and
// confirmed written before R_EN/L_EN are ever asserted HIGH.
//
// This exact ordering is not incidental — see commit f0fc617b ("assert
// BTS7960 R_EN/L_EN only after PWM channels confirm 0 duty"): a real
// hardware boot-time bug produced a brief full-speed-reverse motor kick
// because an earlier revision enabled R_EN/L_EN before the PWM channels'
// duty was confirmed at 0. Reordering these four writes (enabling before
// zeroing) reintroduces that exact bug. controller_test.go's
// TestController_Connect_ZerosBothChannelsBeforeEnabling asserts this order
// directly and was verified to fail against a deliberately-reordered
// implementation before being trusted (see that test's doc comment).
func (c *Controller) Connect(ctx context.Context) error {
	c.mu.Lock()
	defer c.mu.Unlock()

	if err := c.rpwm.SetDuty(ctx, dutyOff); err != nil {
		return fmt.Errorf("motor: zeroing RPWM before enable: %w", err)
	}
	if err := c.lpwm.SetDuty(ctx, dutyOff); err != nil {
		return fmt.Errorf("motor: zeroing LPWM before enable: %w", err)
	}

	// Only now, with both channels confirmed at 0 duty, enable the bridge.
	// R_EN/L_EN gate the module's overcurrent/thermal protection, not
	// direction — see docs/bts7960-ibt2-wiring.md — and are held HIGH for
	// the controller's lifetime.
	if err := c.rEn.SetHigh(ctx, true); err != nil {
		return fmt.Errorf("motor: enabling R_EN: %w", err)
	}
	if err := c.lEn.SetHigh(ctx, true); err != nil {
		return fmt.Errorf("motor: enabling L_EN: %w", err)
	}

	c.connected = true
	return nil
}

// SetSpeed drives the H-bridge at normalizedSpeed, a signed duty fraction in
// [-1, 1] (positive forward, negative reverse), clamped if out of range.
//
// The channel about to become inactive is always zeroed before the newly
// active channel's duty is raised — never the other way around — so there
// is no write-ordering window in which both RPWM and LPWM could read
// nonzero at once (Fast Brake, see splitDuty's doc comment).
func (c *Controller) SetSpeed(ctx context.Context, normalizedSpeed float64) error {
	c.mu.Lock()
	defer c.mu.Unlock()

	if !c.connected {
		return errNotConnected
	}

	rpwmDuty, lpwmDuty := splitDuty(c.sign * normalizedSpeed)

	if rpwmDuty == dutyOff {
		if err := c.rpwm.SetDuty(ctx, dutyOff); err != nil {
			return fmt.Errorf("motor: zeroing RPWM: %w", err)
		}
		if err := c.lpwm.SetDuty(ctx, lpwmDuty); err != nil {
			return fmt.Errorf("motor: setting LPWM duty: %w", err)
		}
		return nil
	}

	if err := c.lpwm.SetDuty(ctx, dutyOff); err != nil {
		return fmt.Errorf("motor: zeroing LPWM: %w", err)
	}
	if err := c.rpwm.SetDuty(ctx, rpwmDuty); err != nil {
		return fmt.Errorf("motor: setting RPWM duty: %w", err)
	}
	return nil
}

// Close stops the drive (both channels to 0 duty) and de-asserts R_EN/L_EN.
//
// This narrows but does not eliminate the disconnect-side floating-GPIO
// window documented in project history (a released GPIO line floats HIGH
// through the level shifter's onboard pull-up, which the BTS7960 reads as
// drive) — the only complete fix is the physical pull-down resistor on
// LPWM, not yet installed as of this port. driver.go's Close adds a
// best-effort re-assert of the physical pins after this returns, matching
// the Python driver's _force_gpio_low.
func (c *Controller) Close(ctx context.Context) error {
	c.mu.Lock()
	defer c.mu.Unlock()

	errs := make([]error, 0, 4) //nolint:mnd // four channel/line writes below, not a magic count
	if err := c.rpwm.SetDuty(ctx, dutyOff); err != nil {
		errs = append(errs, fmt.Errorf("motor: zeroing RPWM on close: %w", err))
	}
	if err := c.lpwm.SetDuty(ctx, dutyOff); err != nil {
		errs = append(errs, fmt.Errorf("motor: zeroing LPWM on close: %w", err))
	}
	if err := c.rEn.SetHigh(ctx, false); err != nil {
		errs = append(errs, fmt.Errorf("motor: disabling R_EN on close: %w", err))
	}
	if err := c.lEn.SetHigh(ctx, false); err != nil {
		errs = append(errs, fmt.Errorf("motor: disabling L_EN on close: %w", err))
	}

	c.connected = false
	return errors.Join(errs...)
}
