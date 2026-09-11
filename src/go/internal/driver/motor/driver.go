//go:build linux

package motor

import (
	"context"
	"errors"
	"fmt"
	"os/exec"
	"strconv"
	"strings"

	"github.com/go-playground/validator/v10"
	"github.com/warthog618/go-gpiocdev"
)

// Config configures a Driver's hardware wiring. Field defaults mirror
// Bts7960PwmConfig (platform/robot/src/hardware/motors/bts7960/config.py) --
// override via the New caller, same as that struct's env/TOML overrides.
type Config struct {
	GPIOChip       string `validate:"required"`
	PWMChip        int    `validate:"gte=0"`
	PWMChannel     int    `validate:"gte=0"`
	FrequencyHz    int    `validate:"required,gt=0"`
	ReversePWMLine int    `validate:"gte=0"`
	REnLine        int    `validate:"gte=0"`
	LEnLine        int    `validate:"gte=0"`
	Invert         bool
}

// Driver is the hardware BTS7960/IBT-2 drive-motor driver: RPWM on the Pi's
// hardware PWM engine via /sys/class/pwm, LPWM via software PWM, R_EN/L_EN
// as plain GPIO outputs. It implements Actuator (controller.go); it does
// not implement driver.Driver[T] -- see doc.go.
//
// All safety-critical sequencing (zero both PWM channels before enabling
// R_EN/L_EN, never drive both RPWM and LPWM nonzero at once) lives in
// Controller, not here -- Driver's job is only to wire real hardware
// adapters to it.
type Driver struct {
	cfg Config

	rpwm     *sysfsPWMChannel
	lpwm     *softPWM
	lpwmLine *gpiocdev.Line
	rEn      *gpioEnableLine
	lEn      *gpioEnableLine
	ctrl     *Controller
}

const (
	// DefaultGPIOChip is the character device the R_EN/L_EN/LPWM GPIO
	// lines are requested against.
	DefaultGPIOChip = "gpiochip0"
	// DefaultPWMChip/DefaultPWMChannel select /sys/class/pwm/pwmchip0/pwm1
	// -- the same channel the L298N predecessor's ENA used (GPIO13 under
	// the pwm-2chan overlay), now feeding RPWM only. See
	// docs/bts7960-ibt2-wiring.md.
	DefaultPWMChip    = 0
	DefaultPWMChannel = 1
	// DefaultFrequencyHz is the PWM carrier frequency shared by both the
	// hardware (RPWM) and software (LPWM) channels.
	DefaultFrequencyHz = 1000
	// DefaultReversePWMLine is the BCM GPIO offset driving LPWM (reverse)
	// via software PWM.
	DefaultReversePWMLine = 26
	// DefaultREnLine/DefaultLEnLine are the BCM GPIO offsets wired to the
	// module's R_EN/L_EN -- held permanently HIGH once Connect enables
	// them; they gate protection, not direction.
	DefaultREnLine = 6
	DefaultLEnLine = 5
)

var _ Actuator = (*Driver)(nil)

// DefaultConfig returns the Config matching this project's current wiring
// (see docs/bts7960-ibt2-wiring.md).
func DefaultConfig() Config {
	return Config{
		GPIOChip:       DefaultGPIOChip,
		PWMChip:        DefaultPWMChip,
		PWMChannel:     DefaultPWMChannel,
		FrequencyHz:    DefaultFrequencyHz,
		ReversePWMLine: DefaultReversePWMLine,
		REnLine:        DefaultREnLine,
		LEnLine:        DefaultLEnLine,
	}
}

// New validates cfg and returns a Driver. Call Connect before SetSpeed.
func New(cfg Config) (*Driver, error) {
	if err := validator.New().Struct(cfg); err != nil {
		return nil, fmt.Errorf("motor: invalid config: %w", err)
	}
	return &Driver{cfg: cfg}, nil
}

// Connect claims the RPWM hardware-PWM channel and the LPWM/R_EN/L_EN GPIO
// lines, then delegates the safety-critical enable sequencing to
// Controller.Connect. See Controller.Connect's doc comment for why that
// ordering is load-bearing, not stylistic.
func (d *Driver) Connect(ctx context.Context) error {
	rpwm := newSysfsPWMChannel(sysfsPWMRoot, d.cfg.PWMChip, d.cfg.PWMChannel, d.cfg.FrequencyHz)
	if err := rpwm.Export(ctx); err != nil {
		return err //nolint:wrapcheck // Export already wraps with "motor: ..." context
	}
	if err := rpwm.Init(ctx); err != nil {
		return err //nolint:wrapcheck // Init already wraps with "motor: ..." context
	}

	lpwmLine, err := gpiocdev.RequestLine(
		d.cfg.GPIOChip,
		d.cfg.ReversePWMLine,
		gpiocdev.AsOutput(gpioLow),
	)
	if err != nil {
		return fmt.Errorf("motor: requesting LPWM GPIO line: %w", err)
	}
	lpwm := newSoftPWM(lpwmLine, d.cfg.FrequencyHz)
	lpwm.start()

	rEn, err := newGPIOEnableLine(d.cfg.GPIOChip, d.cfg.REnLine)
	if err != nil {
		return errors.Join(err, lpwm.Stop(), lpwmLine.Close())
	}
	lEn, err := newGPIOEnableLine(d.cfg.GPIOChip, d.cfg.LEnLine)
	if err != nil {
		return errors.Join(err, rEn.Close(), lpwm.Stop(), lpwmLine.Close())
	}

	ctrl := NewController(rpwm, lpwm, rEn, lEn, d.cfg.Invert)
	if err := ctrl.Connect(ctx); err != nil {
		return errors.Join(err, rEn.Close(), lEn.Close(), lpwm.Stop(), lpwmLine.Close())
	}

	d.rpwm, d.lpwm, d.lpwmLine, d.rEn, d.lEn, d.ctrl = rpwm, lpwm, lpwmLine, rEn, lEn, ctrl
	return nil
}

// SetSpeed drives the motor at normalizedSpeed (positive forward, negative
// reverse, clamped to [-1, 1]).
func (d *Driver) SetSpeed(ctx context.Context, normalizedSpeed float64) error {
	if d.ctrl == nil {
		return errNotConnected
	}
	return d.ctrl.SetSpeed(ctx, normalizedSpeed)
}

// Close stops the drive, releases the RPWM/LPWM/R_EN/L_EN resources, and
// makes a best-effort attempt to re-drive the physical pins LOW afterward.
//
// That last step narrows but does not eliminate the disconnect-side
// floating-GPIO window: releasing a go-gpiocdev line request un-claims it
// entirely rather than leaving it driven at its last value, and this
// board's level shifter reads a released (floating) line as HIGH --
// full-speed reverse on the BTS7960, confirmed on hardware for the Python
// driver this ports. The physical pull-down resistor on LPWM, still not
// installed as of this port, remains the only complete fix -- see
// project history (gpio_boot_float_full_speed_motor,
// bts7960_connect_ordering_bug).
func (d *Driver) Close() error {
	ctx := context.Background()
	var errs []error

	if d.ctrl != nil {
		if err := d.ctrl.Close(ctx); err != nil {
			errs = append(errs, err)
		}
	}
	if d.rpwm != nil {
		if err := d.rpwm.Disable(); err != nil {
			errs = append(errs, err)
		}
	}
	if d.lpwm != nil {
		if err := d.lpwm.Stop(); err != nil {
			errs = append(errs, fmt.Errorf("motor: stopping LPWM software PWM: %w", err))
		}
	}
	if d.lpwmLine != nil {
		if err := d.lpwmLine.Close(); err != nil {
			errs = append(errs, fmt.Errorf("motor: closing LPWM GPIO line: %w", err))
		}
	}
	if d.rEn != nil {
		if err := d.rEn.Close(); err != nil {
			errs = append(errs, err)
		}
	}
	if d.lEn != nil {
		if err := d.lEn.Close(); err != nil {
			errs = append(errs, err)
		}
	}

	forceGPIOLow(d.cfg)
	return errors.Join(errs...)
}

// forceGPIOLow best-effort re-drives LPWM/R_EN/L_EN low via pinctrl after
// this process releases its GPIO line requests, mirroring the Python
// driver's _force_gpio_low: pinctrl programs the line's output state
// directly in the SoC's GPIO controller rather than through a held-open
// handle, so it persists after this process exits. Errors are deliberately
// swallowed -- this runs on dev machines without pinctrl too, and Close
// must not fail just because the mitigation binary is unavailable.
func forceGPIOLow(cfg Config) {
	pins := strings.Join([]string{
		strconv.Itoa(cfg.ReversePWMLine),
		strconv.Itoa(cfg.REnLine),
		strconv.Itoa(cfg.LEnLine),
	}, ",")
	//nolint:gosec // G204: args are config-derived integers, not user input; mirrors the Python driver's
	// subprocess.run(["pinctrl", ...], check=False) (noqa: S607).
	_ = exec.Command("pinctrl", "set", pins, "op", "dl").Run()
}
