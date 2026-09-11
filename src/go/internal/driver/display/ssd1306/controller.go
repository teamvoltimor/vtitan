package ssd1306

import "fmt"

// i2cWriter is the narrow contract Controller's command/data writes go
// through — implemented by *i2c.Dev (periph.io/x/conn/v3/i2c) against real
// hardware (driver.go), and by a fake in controller_test.go that records
// what was written. Defined here, where it's consumed, per go-architect
// §4.
type i2cWriter interface {
	Write(b []byte) (int, error)
}

// Controller holds the pure, hardware-independent SSD1306 protocol logic:
// the init command sequence, page-addressing window, and command/data
// control-byte framing. It depends only on the small i2cWriter interface
// above, so it is fully unit-testable with a fake that records what was
// written and in what order — see controller_test.go — without any real
// I2C bus. driver.go wires it to a real periph.io I2C connection, the same
// split internal/driver/motor uses between Controller (logic) and Driver
// (hardware wiring).
type Controller struct {
	w   i2cWriter
	cfg Config
}

// NewController builds a Controller that writes through w, using cfg's
// panel geometry.
func NewController(w i2cWriter, cfg Config) *Controller {
	return &Controller{w: w, cfg: cfg}
}

// Init runs the SSD1306 init command sequence and clears the display,
// ported byte-for-byte (same commands, same order, same parameter values)
// from driver_raw_i2c.py's connect() body (after that method's fd/ioctl
// setup, which Driver.Connect's periph.io bus-open replaces).
func (c *Controller) Init() error {
	multiplex := byte(c.cfg.Height - 1)
	comPins := comPinsConfig32
	if c.cfg.Height == height128x64 {
		comPins = comPinsConfig64
	}

	if err := c.writeCommand(
		cmdDisplayOff,
		cmdSetDisplayClockDiv, displayClockDivValue,
		cmdSetMultiplex, multiplex,
		cmdSetDisplayOffset, displayOffsetValue,
		cmdSetStartLine,
		cmdChargePump, chargePumpEnableValue,
		cmdMemoryMode, memoryModeHorizontal,
		cmdSegRemap,
		cmdComScanDec,
		cmdSetComPins, comPins,
		cmdSetContrast, contrastValue,
		cmdSetPrecharge, prechargeValue,
		cmdSetVCOMDetect, vcomDetectValue,
		cmdDisplayAllOnResume,
		cmdNormalDisplay,
		cmdDisplayOn,
	); err != nil {
		return err
	}

	return c.Clear()
}

// Clear blanks every pixel on the display, matching driver_raw_i2c.py's
// clear().
func (c *Controller) Clear() error {
	numPages := c.cfg.Height / bitsPerPage
	blank := make([]byte, c.cfg.Width*numPages)

	if err := c.setAddressingWindow(); err != nil {
		return err
	}
	return c.writeData(blank)
}

// WriteFramebuffer writes fb's pixel contents to the display. fb's
// dimensions must match cfg — driver_raw_i2c.py's show_image silently
// resizes a mismatched PIL image instead, which isn't ported here (see
// doc.go's scope note): callers must build a Framebuffer matching cfg's
// Width/Height.
func (c *Controller) WriteFramebuffer(fb *Framebuffer) error {
	if fb.Width() != c.cfg.Width || fb.Height() != c.cfg.Height {
		return fmt.Errorf("%w: framebuffer %dx%d, display %dx%d",
			ErrFramebufferSizeMismatch, fb.Width(), fb.Height(), c.cfg.Width, c.cfg.Height)
	}

	if err := c.setAddressingWindow(); err != nil {
		return err
	}
	return c.writeData(fb.Bytes())
}

// Off sends the display-off command, matching driver_raw_i2c.py's close()
// (the part of it that's protocol, not fd teardown — see Driver.Close).
func (c *Controller) Off() error {
	return c.writeCommand(cmdDisplayOff)
}

// setAddressingWindow sets the column/page addressing window to the whole
// display, matching driver_raw_i2c.py's _set_addressing_window().
func (c *Controller) setAddressingWindow() error {
	numPages := c.cfg.Height / bitsPerPage
	return c.writeCommand(
		cmdColumnAddr, 0, byte(c.cfg.Width-1),
		cmdPageAddr, 0, byte(numPages-1),
	)
}

// writeCommand sends each byte in cmds as its own I2C write, prefixed with
// controlCommand — one write() call per command byte, matching
// driver_raw_i2c.py's _write_command loop exactly (this is load-bearing
// for controller_test.go's byte-sequence assertions, not an arbitrary
// choice).
func (c *Controller) writeCommand(cmds ...byte) error {
	for _, cmd := range cmds {
		if _, err := c.w.Write([]byte{controlCommand, cmd}); err != nil {
			return fmt.Errorf("ssd1306: writing command 0x%02X: %w", cmd, err)
		}
	}
	return nil
}

// writeData sends the full data buffer as a single I2C write, prefixed
// once with controlData — matching driver_raw_i2c.py's _write_data, which
// deliberately sends the whole framebuffer in one transaction rather than
// 32-byte SMBus-block-sized chunks (see that method's doc comment for the
// measured ~100ms/frame cost this avoids).
func (c *Controller) writeData(data []byte) error {
	buf := make([]byte, 0, len(data)+1)
	buf = append(buf, controlData)
	buf = append(buf, data...)
	if _, err := c.w.Write(buf); err != nil {
		return fmt.Errorf("ssd1306: writing data (%d bytes): %w", len(data), err)
	}
	return nil
}
