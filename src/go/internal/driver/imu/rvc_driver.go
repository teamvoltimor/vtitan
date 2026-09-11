package imu

import (
	"bufio"
	"context"
	"errors"
	"fmt"

	"github.com/go-playground/validator/v10"
	"go.bug.st/serial"

	"github.com/teamvoltimor/vtitan/src/go/internal/driver"
)

// Config configures an RVCDriver's serial connection. Matches the fields
// platform/robot/src/hardware/imu/bno08x/uart_rvc.py's Config exposes for
// the serial link itself (port, baudrate) — polling rate isn't configured
// here since RVCDriver.Read blocks on the next hardware-streamed frame
// rather than polling on a timer (see Read's doc comment).
type Config struct {
	Port     string `validate:"required"`
	BaudRate int    `validate:"required,gt=0"`
}

// RVCDriver reads BNO08x UART-RVC frames from a serial port. It implements
// driver.Driver[Reading] (platform/robot-go/internal/driver).
type RVCDriver struct {
	cfg    Config
	port   serial.Port
	reader *bufio.Reader
}

// readResult is the channel payload Read uses to hand a completed (or
// failed) frame back from the blocking goroutine.
type readResult struct {
	reading Reading
	err     error
}

// DefaultBaudRate is the BNO08x UART-RVC mode's documented serial rate,
// matching platform/robot's Config.baudrate default.
const DefaultBaudRate = 115200

var (
	errReadBeforeConnect = errors.New("imu: Read called before Connect")

	// Compile-time assertion that RVCDriver satisfies driver.Driver[Reading].
	_ driver.Driver[Reading] = (*RVCDriver)(nil)
)

// New validates cfg and returns an RVCDriver. Call Connect before Read.
func New(cfg Config) (*RVCDriver, error) {
	if err := validator.New().Struct(cfg); err != nil {
		return nil, fmt.Errorf("imu: invalid config: %w", err)
	}
	return &RVCDriver{cfg: cfg}, nil
}

// Connect opens the configured serial port. The BNO08x streams RVC frames
// continuously once powered, so no handshake beyond opening the port at the
// right baud rate is needed.
func (d *RVCDriver) Connect(_ context.Context) error {
	mode := &serial.Mode{BaudRate: d.cfg.BaudRate}
	port, err := serial.Open(d.cfg.Port, mode)
	if err != nil {
		return fmt.Errorf("imu: opening serial port %s: %w", d.cfg.Port, err)
	}
	d.port = port
	d.reader = bufio.NewReader(port)
	return nil
}

// Read blocks until the next valid RVC frame arrives (the sensor streams at
// ~100Hz) or until ctx is done. If ctx is canceled while a read is in
// flight, the underlying goroutine is left blocked on the serial read until
// Close is called — Close closing the port is what unblocks it, matching
// the standard Go pattern for wrapping a blocking syscall with a context.
func (d *RVCDriver) Read(ctx context.Context) (Reading, error) {
	if d.reader == nil {
		return Reading{}, errReadBeforeConnect
	}

	resultCh := make(chan readResult, 1)
	go func() {
		reading, err := readFrame(d.reader)
		resultCh <- readResult{reading: reading, err: err}
	}()

	select {
	case <-ctx.Done():
		return Reading{}, fmt.Errorf("imu: waiting for frame: %w", ctx.Err())
	case res := <-resultCh:
		return res.reading, res.err
	}
}

// Close closes the underlying serial port. Safe to call even if Connect was
// never called.
func (d *RVCDriver) Close() error {
	if d.port == nil {
		return nil
	}
	if err := d.port.Close(); err != nil {
		return fmt.Errorf("imu: closing serial port %s: %w", d.cfg.Port, err)
	}
	return nil
}
