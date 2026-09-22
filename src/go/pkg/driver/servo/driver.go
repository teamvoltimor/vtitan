package servo

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/go-playground/validator/v10"

	"github.com/teamvoltimor/vtitan/src/go/pkg/driver/internal/sysfspwm"
)

// Config is the servo's wiring and pulse calibration. Every field except
// Root mirrors a key of src/config/hardware/motors/servo.toml (and the
// Python ServoConfig built from it); there is deliberately no DefaultConfig,
// because the active servo profile overlays range_deg (a 270 degree servo
// read with the 180 degree base value would steer at two thirds of the
// commanded angle). Resolve it from the TOML tree instead.
type Config struct {
	// Root is the sysfs PWM class directory; empty means
	// sysfspwm.DefaultRoot (/sys/class/pwm). Tests point it at a temp dir.
	Root string

	// GPIOPin is only used to name the overlay in Export's error.
	GPIOPin    int `validate:"gte=0"`
	PWMChip    int `validate:"gte=0"`
	PWMChannel int `validate:"gte=0"`
	// FrequencyHz is the carrier; 50 Hz for a hobby servo.
	FrequencyHz int `validate:"required,gt=0"`

	MinPulseUS    float64 `validate:"gt=0"`
	MaxPulseUS    float64 `validate:"gtfield=MinPulseUS"`
	CenterPulseUS float64 `validate:"gtefield=MinPulseUS,ltefield=MaxPulseUS"`
	// RangeDeg is the servo's full mechanical travel (180 or 270): the angle
	// that spans MinPulseUS..MaxPulseUS.
	RangeDeg float64 `validate:"gt=0"`
	// Reversed flips the sign of every commanded angle.
	Reversed bool
}

// channel is the part of sysfspwm.Channel the driver uses once exported.
// It is an interface so the connect/center/epsilon/close sequencing can be
// tested with a recorder.
type channel interface {
	Init() error
	WriteDutyNS(dutyNS int64) error
	Disable() error
}

// Driver is a single RC servo on a hardware PWM channel. It has no position
// feedback: Angle is the last commanded value. Not safe for concurrent use;
// the motor loop owns it from one goroutine.
type Driver struct {
	cfg Config

	ch       channel
	angleDeg float64
	// pulseUS is the last pulse written; hasPulse is false before the first
	// write after Connect, so that write is never skipped as a no-op
	// (servo/driver.py:89 _pulse_us = None, reset on disconnect at :147).
	pulseUS  float64
	hasPulse bool
}

const (
	// CenterDeg is the servo angle for wheels-straight
	// (motors/base.py:40 STEERING_CENTER_DEG). Center writes it; the
	// steering offset trim is NOT applied, as in Python's center_steering.
	CenterDeg = 0.0

	// pulseEpsilonUS is the smallest pulse change worth writing
	// (servo/driver.py:63 _PULSE_EPSILON_US). A controller holding a
	// heading resends the same angle every tick; 1 us is well under a hobby
	// servo's own 2-10 us deadband, so this cannot swallow a real move.
	pulseEpsilonUS = 1.0

	// nsPerUS converts the pulse width to the duty_cycle attribute's unit.
	nsPerUS = 1000
)

var (
	// ErrNotConnected is returned by SetAngle and Center before Connect.
	ErrNotConnected = errors.New("servo: driver not connected")

	// ErrNonFiniteAngle is returned by SetAngle for NaN or +-Inf, and
	// nothing is written: the pulse stays where it was. Python has no such
	// guard - NaN falls through its min/max clamp to a full-lock pulse - so
	// this is a deliberate divergence, not a port.
	ErrNonFiniteAngle = errors.New("servo: non-finite angle")
)

// New validates cfg and returns a Driver. Call Connect before SetAngle.
func New(cfg Config) (*Driver, error) {
	if err := validator.New().Struct(cfg); err != nil {
		return nil, fmt.Errorf("servo: invalid config: %w", err)
	}
	if cfg.Root == "" {
		cfg.Root = sysfspwm.DefaultRoot
	}
	return &Driver{cfg: cfg, angleDeg: CenterDeg}, nil
}

// Connect exports the PWM channel, starts the carrier (duty 0, period,
// enable, in that order), and centers the servo - servo/driver.py:108-134.
// On a machine without the PWM overlay (any dev box) it fails with an error
// wrapping sysfspwm.ErrOverlayMissing.
func (d *Driver) Connect(ctx context.Context) error {
	ch := sysfspwm.New(
		d.cfg.Root,
		d.cfg.PWMChip,
		d.cfg.PWMChannel,
		d.cfg.FrequencyHz,
		// servo/driver.py:103's hint, verbatim.
		fmt.Sprintf("'dtoverlay=pwm,pin=%d,func=4'", d.cfg.GPIOPin),
	)
	if err := ch.Export(ctx); err != nil {
		return fmt.Errorf("servo: GPIO %d: %w", d.cfg.GPIOPin, err)
	}
	return d.connectChannel(ch)
}

// SetAngle moves the servo to an absolute angle in servo degrees, 0 =
// center, positive = the direction a positive pulse offset turns it (after
// Reversed). The pulse is clamped to [MinPulseUS, MaxPulseUS]; a change
// under 1 us is not written. servo/driver.py:157-181 move_steering_to.
func (d *Driver) SetAngle(angleDeg float64) error {
	if math.IsNaN(angleDeg) || math.IsInf(angleDeg, 0) {
		return fmt.Errorf("%w: %v", ErrNonFiniteAngle, angleDeg)
	}
	if d.ch == nil {
		return ErrNotConnected
	}

	pulseUS := PulseUS(d.cfg, angleDeg)
	if !d.hasPulse || math.Abs(pulseUS-d.pulseUS) >= pulseEpsilonUS {
		// int(pulse_us * NS_PER_US): truncation toward zero, as Python's int().
		if err := d.ch.WriteDutyNS(int64(pulseUS * nsPerUS)); err != nil {
			return fmt.Errorf("servo: PWM duty_cycle write failed: %w", err)
		}
		d.pulseUS = pulseUS
		d.hasPulse = true
	}
	d.angleDeg = angleDeg
	return nil
}

// Center moves the servo to CenterDeg (motors/base.py:125-127).
func (d *Driver) Center() error {
	return d.SetAngle(CenterDeg)
}

// Angle is the last commanded servo angle in degrees (no feedback).
func (d *Driver) Angle() float64 { return d.angleDeg }

// Close disables the carrier and forgets the cached pulse, so the first
// write after a reconnect is not skipped (servo/driver.py:136-147). It does
// not center first, as Python's disconnect does not: callers that want the
// wheels straight center before closing, as the motor loop does on its way
// out. With the carrier off the servo gets no pulses at all; whether it then
// goes limp or holds its last position depends on the servo and has not been
// checked on this one. Close on an unconnected Driver is a no-op.
func (d *Driver) Close() error {
	d.hasPulse = false
	if d.ch == nil {
		return nil
	}
	ch := d.ch
	d.ch = nil
	if err := ch.Disable(); err != nil {
		return fmt.Errorf("servo: disabling PWM on close: %w", err)
	}
	return nil
}

// connectChannel is Connect after export: init the carrier, then center.
func (d *Driver) connectChannel(ch channel) error {
	if err := ch.Init(); err != nil {
		return fmt.Errorf("servo: PWM init failed: %w", err)
	}
	d.ch = ch
	d.hasPulse = false
	return d.Center()
}

// PulseUS maps a servo angle (deg, 0 = center) to a pulse width in
// microseconds: center + (angle / range) * (max - min), sign flipped when
// Reversed, clamped to [min, max]. servo/driver.py:149-155
// _position_to_pulse_us. The caller keeps angleDeg finite.
func PulseUS(cfg Config, angleDeg float64) float64 {
	signed := angleDeg
	if cfg.Reversed {
		signed = -angleDeg
	}
	spanUS := cfg.MaxPulseUS - cfg.MinPulseUS
	pulseUS := cfg.CenterPulseUS + (signed/cfg.RangeDeg)*spanUS
	return max(cfg.MinPulseUS, min(cfg.MaxPulseUS, pulseUS))
}
