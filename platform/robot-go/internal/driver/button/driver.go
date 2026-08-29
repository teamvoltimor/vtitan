//go:build linux

package button

import (
	"context"
	"errors"
	"fmt"
	"time"

	"github.com/go-playground/validator/v10"
	"github.com/warthog618/go-gpiocdev"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver"
)

// gpioLineLow is the raw line-value integer go-gpiocdev's Value returns
// for a LOW reading -- matching motor.gpioLow's meaning in the sibling
// driver package (not reused directly since it belongs to a different
// package's private consts).
const gpioLineLow = 0

// errReadBeforeConnect is returned by Read when called before Connect has
// completed successfully.
var errReadBeforeConnect = errors.New("button: Read called before Connect")

// Config configures a Driver's GPIO wiring and poll rate, plus the embedded
// debounce/hold-threshold timings (Thresholds). GPIOChip/Line/PullUp are
// real hardware facts, matching
// platform/robot/src/hardware/button/gpio/driver.py's Config fields
// (gpio_pin, pull_up) -- PollInterval is the node-level tunable
// platform/robot/ros2_ws/.../button_node.py sources from
// button_node.toml's POLL_HZ (default 20Hz). It's ported here as a
// driver-level field rather than kept as a separate lifecycle-node timer,
// since this Go port folds the node's poll loop into Driver.Read itself --
// see Read's doc comment.
type Config struct {
	GPIOChip     string `validate:"required"`
	Line         int    `validate:"gte=0"`
	PullUp       bool
	PollInterval time.Duration `validate:"required,gt=0"`
	Thresholds   Thresholds    `validate:"required"`
}

// DefaultGPIOChip is the character device every GPIO consumer on this
// board requests lines against (matches motor.DefaultGPIOChip). There is
// deliberately no default Line: config.py's gpio_pin is required with no
// fallback (env BUTTON_GPIO_PIN) -- guessing a pin number risks driving
// the wrong physical line.
const DefaultGPIOChip = "gpiochip0"

// DefaultPollInterval matches button_node.toml's default POLL_HZ (20Hz).
const DefaultPollInterval = 50 * time.Millisecond

// Driver is the hardware GPIO-backed button driver: a single input line,
// sampled at Config.PollInterval and run through the pure evaluator
// (evaluator.go) to produce debounced press/hold/release events. It
// implements driver.Driver[Event] (platform/robot-go/internal/driver).
type Driver struct {
	cfg  Config
	line *gpiocdev.Line
	eval *Evaluator
}

var _ driver.Driver[Event] = (*Driver)(nil)

// New validates cfg and returns a Driver. Call Connect before Read.
func New(cfg Config) (*Driver, error) {
	if err := validator.New().Struct(cfg); err != nil {
		return nil, fmt.Errorf("button: invalid config: %w", err)
	}
	return &Driver{cfg: cfg, eval: NewEvaluator(cfg.Thresholds)}, nil
}

// Connect requests the configured GPIO line as an input, with internal
// pull-up or pull-down bias selected by Config.PullUp -- matching
// gpiozero's Button(pull_up=...) in the Python driver this ports.
func (d *Driver) Connect(_ context.Context) error {
	bias := gpiocdev.WithPullDown
	if d.cfg.PullUp {
		bias = gpiocdev.WithPullUp
	}
	line, err := gpiocdev.RequestLine(d.cfg.GPIOChip, d.cfg.Line, gpiocdev.AsInput, bias)
	if err != nil {
		return fmt.Errorf("button: requesting GPIO line %s:%d: %w", d.cfg.GPIOChip, d.cfg.Line, err)
	}
	d.line = line
	return nil
}

// Read polls the button line at Config.PollInterval and blocks until the
// poll loop's samples produce the next debounced event (press, a hold
// threshold crossing, short-press-on-release, or release-after-hold) --
// see Evaluator.Sample. This collapses button_node.py's separate 20Hz
// ROS2 timer plus the driver's own threading.Timer-based hold callbacks
// into one loop, since Go's Driver[T] contract is "block until the next
// value," not "poll on demand."
func (d *Driver) Read(ctx context.Context) (Event, error) {
	if d.line == nil {
		return Event{}, errReadBeforeConnect
	}

	ticker := time.NewTicker(d.cfg.PollInterval)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			return Event{}, fmt.Errorf("button: waiting for event: %w", ctx.Err())
		case now := <-ticker.C:
			value, err := d.line.Value()
			if err != nil {
				return Event{}, fmt.Errorf("button: reading GPIO line value: %w", err)
			}
			rawPressed := (value == gpioLineLow) == d.cfg.PullUp
			if event := d.eval.Sample(rawPressed, now); event != nil {
				return *event, nil
			}
		}
	}
}

// Close releases the underlying GPIO line. Safe to call even if Connect
// was never called.
func (d *Driver) Close() error {
	if d.line == nil {
		return nil
	}
	if err := d.line.Close(); err != nil {
		return fmt.Errorf("button: closing GPIO line: %w", err)
	}
	return nil
}
