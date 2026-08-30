package ssd1306

import (
	"context"
	"errors"
	"fmt"
	"strconv"
	"sync"

	"github.com/go-playground/validator/v10"
	"periph.io/x/conn/v3/i2c"
	"periph.io/x/conn/v3/i2c/i2creg"
	"periph.io/x/host/v3"
)

// Actuator is the actuator-shaped counterpart to driver.Driver[T]
// (platform/robot-go/internal/driver): commanded via WriteFramebuffer
// rather than sampled via Read — see doc.go. Kept to 3 methods
// (interfacebloat's cap, go-migration-plan.md's "Interfaces" guidance);
// Clear is exposed as an additional Driver method rather than folded into
// this interface, since callers needing an Actuator abstraction (a future
// page-orchestration consumer) can always clear by writing a blank
// Framebuffer through WriteFramebuffer. Driver is this interface's only
// implementation.
type Actuator interface {
	Connect(ctx context.Context) error
	WriteFramebuffer(ctx context.Context, fb *Framebuffer) error
	Close() error
}

// Driver is the SSD1306 OLED display driver: Connect claims a real
// periph.io I2C bus and wires it into a Controller, which owns all
// protocol logic (controller.go). It implements Actuator; it does not
// implement driver.Driver[T] — see doc.go.
type Driver struct {
	cfg Config

	mu   sync.Mutex
	bus  i2c.BusCloser // real periph.io bus handle, closed by Close
	ctrl *Controller
}

var (
	// ErrNotConnected is returned by WriteFramebuffer/Clear when called
	// before Connect has completed successfully.
	ErrNotConnected = errors.New("ssd1306: called before Connect")

	// ErrFramebufferSizeMismatch is returned by Controller.WriteFramebuffer
	// (and therefore Driver.WriteFramebuffer) when fb's dimensions don't
	// match the Config the display was configured with.
	ErrFramebufferSizeMismatch = errors.New(
		"ssd1306: framebuffer size does not match display config",
	)

	// Compile-time assertion that Driver satisfies Actuator.
	_ Actuator = (*Driver)(nil)
)

// New validates cfg and returns a Driver. Call Connect before
// WriteFramebuffer/Clear.
func New(cfg Config) (*Driver, error) {
	if err := validator.New().Struct(cfg); err != nil {
		return nil, fmt.Errorf("ssd1306: invalid config: %w", err)
	}
	return &Driver{cfg: cfg}, nil
}

// Connect opens the configured I2C bus and runs the SSD1306 init sequence
// via a Controller wired to it.
func (d *Driver) Connect(_ context.Context) error {
	d.mu.Lock()
	defer d.mu.Unlock()

	if _, err := host.Init(); err != nil {
		return fmt.Errorf("ssd1306: initializing periph.io host drivers: %w", err)
	}

	bus, err := i2creg.Open(strconv.Itoa(d.cfg.I2CBus))
	if err != nil {
		return fmt.Errorf("ssd1306: opening I2C bus %d: %w", d.cfg.I2CBus, err)
	}

	dev := &i2c.Dev{Bus: bus, Addr: d.cfg.I2CAddress}
	ctrl := NewController(dev, d.cfg)
	if initErr := ctrl.Init(); initErr != nil {
		return errors.Join(fmt.Errorf("ssd1306: running init sequence: %w", initErr), bus.Close())
	}

	d.bus = bus
	d.ctrl = ctrl
	return nil
}

// Clear blanks every pixel on the display.
func (d *Driver) Clear(_ context.Context) error {
	d.mu.Lock()
	defer d.mu.Unlock()

	if d.ctrl == nil {
		return ErrNotConnected
	}
	return d.ctrl.Clear()
}

// WriteFramebuffer writes fb's pixel contents to the display. fb's
// dimensions must match the Config Connect was called with.
func (d *Driver) WriteFramebuffer(_ context.Context, fb *Framebuffer) error {
	d.mu.Lock()
	defer d.mu.Unlock()

	if d.ctrl == nil {
		return ErrNotConnected
	}
	return d.ctrl.WriteFramebuffer(fb)
}

// Close turns the display off and closes the underlying I2C bus. Safe to
// call even if Connect was never called.
func (d *Driver) Close() error {
	d.mu.Lock()
	defer d.mu.Unlock()

	if d.ctrl == nil {
		return nil
	}

	var errs []error
	if err := d.ctrl.Off(); err != nil {
		errs = append(errs, fmt.Errorf("ssd1306: turning display off on close: %w", err))
	}
	if err := d.bus.Close(); err != nil {
		errs = append(errs, fmt.Errorf("ssd1306: closing I2C bus: %w", err))
	}

	d.ctrl = nil
	d.bus = nil
	return errors.Join(errs...)
}
