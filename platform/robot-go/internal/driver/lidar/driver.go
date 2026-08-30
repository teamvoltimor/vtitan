package lidar

import (
	"bufio"
	"context"
	"errors"
	"fmt"
	"io"
	"time"

	"github.com/go-playground/validator/v10"
	"go.bug.st/serial"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver"
)

// Config configures a SerialDriver's serial connection to an RPLIDAR C1.
type Config struct {
	Port     string `validate:"required"`
	BaudRate int    `validate:"required,gt=0"`
}

// SerialDriver reads 360-degree Scans from an RPLIDAR C1 over its TTL UART
// interface using the classic SCAN command (see doc.go for scope). It
// implements driver.Driver[Scan] (platform/robot-go/internal/driver).
type SerialDriver struct {
	cfg    Config
	port   serial.Port
	reader *bufio.Reader
	// first holds one already-decoded measurement that arrived with S=1
	// (start-of-scan) while assembling the *previous* Scan — it belongs
	// to the *next* Scan and is carried over between Read calls instead
	// of being discarded.
	first *Point
}

// DefaultBaudRate is the RPLIDAR C1's documented UART baud rate ("Data
// Communication Interface", Communication Speed = 460800 bps — RPLIDAR C1
// datasheet rev 1.1, 2024-03-12).
const DefaultBaudRate = 460800

// MinRangeM and MaxRangeM are the RPLIDAR C1's documented distance range
// ("Measurement Performance", Distance Range: white object 0.05-12m —
// RPLIDAR C1 datasheet rev 1.1). Callers assembling a
// proto/vtitan/sensor/v1/scan.proto message from a Scan should use these
// for range_min/range_max.
const (
	MinRangeM = 0.05
	MaxRangeM = 12.0
)

// settleDelay durations: the protocol doc requires the host to wait after
// sending a no-response request before sending another, since RPLIDAR
// needs time to process it ("STOP Request" / "RPLIDAR Core Reset(RESET)
// Request").
const (
	stopSettleDelay  = 1 * time.Millisecond
	resetSettleDelay = 2 * time.Millisecond
)

var (
	errReadBeforeConnect = errors.New("lidar: Read called before Connect")

	// Compile-time assertion that SerialDriver satisfies driver.Driver[Scan].
	_ driver.Driver[Scan] = (*SerialDriver)(nil)
)

// New validates cfg and returns a SerialDriver. Call Connect before Read.
func New(cfg Config) (*SerialDriver, error) {
	if err := validator.New().Struct(cfg); err != nil {
		return nil, fmt.Errorf("lidar: invalid config: %w", err)
	}
	return &SerialDriver{cfg: cfg}, nil
}

// Connect opens the configured serial port, stops any scan already in
// progress (the device may still be scanning from a previous session that
// didn't clean up), and issues the classic SCAN request so measurement
// samples start streaming.
func (d *SerialDriver) Connect(ctx context.Context) error {
	mode := &serial.Mode{BaudRate: d.cfg.BaudRate}
	port, err := serial.Open(d.cfg.Port, mode)
	if err != nil {
		return fmt.Errorf("lidar: opening serial port %s: %w", d.cfg.Port, err)
	}
	d.port = port
	d.reader = bufio.NewReader(port)

	// Best-effort: if the device is already scanning from a prior session,
	// this stops it so the SCAN request below starts a clean session. A
	// fresh device that isn't scanning simply ignores it ("This request
	// will be ignored when RPLIDAR is in the Idle or Protection Stop
	// state.").
	if stopErr := d.Stop(ctx); stopErr != nil {
		return fmt.Errorf("lidar: stopping prior scan session: %w", stopErr)
	}

	if _, writeErr := d.port.Write(requestPacket(cmdScan)); writeErr != nil {
		return fmt.Errorf("lidar: sending SCAN request: %w", writeErr)
	}

	desc, descErr := d.readDescriptor()
	if descErr != nil {
		return fmt.Errorf("lidar: reading SCAN response descriptor: %w", descErr)
	}
	if desc.dataType != dataTypeMeasurement {
		return fmt.Errorf(
			"%w: got 0x%02X, want 0x%02X",
			ErrUnexpectedDataType,
			desc.dataType,
			dataTypeMeasurement,
		)
	}

	return nil
}

// Read blocks until a full Scan (all measurement samples between two
// start-of-scan flags) has been assembled, or until ctx is done. If ctx is
// canceled while a read is in flight, the underlying goroutine is left
// blocked on the serial read until Close is called — Close closing the
// port is what unblocks it, matching the standard Go pattern for wrapping
// a blocking syscall with a context (same pattern as imu.RVCDriver.Read).
func (d *SerialDriver) Read(ctx context.Context) (Scan, error) {
	if d.reader == nil {
		return nil, errReadBeforeConnect
	}

	type result struct {
		scan Scan
		err  error
	}
	resultCh := make(chan result, 1)
	go func() {
		scan, err := d.readScan()
		resultCh <- result{scan: scan, err: err}
	}()

	select {
	case <-ctx.Done():
		return nil, fmt.Errorf("lidar: waiting for scan: %w", ctx.Err())
	case res := <-resultCh:
		return res.scan, res.err
	}
}

// Close sends a STOP request (best-effort — the port may already be
// unusable) and closes the underlying serial port. Safe to call even if
// Connect was never called.
func (d *SerialDriver) Close() error {
	if d.port == nil {
		return nil
	}
	if _, writeErr := d.port.Write(requestPacket(cmdStop)); writeErr != nil {
		// Best-effort: a failed STOP shouldn't block Close from still
		// closing the port, so this is intentionally not returned.
		_ = writeErr
	}

	if err := d.port.Close(); err != nil {
		return fmt.Errorf("lidar: closing serial port %s: %w", d.cfg.Port, err)
	}
	return nil
}

// Stop sends the STOP request, exiting the scanning state. RPLIDAR sends
// no response to this request; Stop waits the protocol-mandated settle
// delay (or until ctx is done, whichever comes first) before returning so
// the caller doesn't immediately send another request the device isn't
// ready for yet.
func (d *SerialDriver) Stop(ctx context.Context) error {
	if _, err := d.port.Write(requestPacket(cmdStop)); err != nil {
		return fmt.Errorf("lidar: sending STOP request: %w", err)
	}
	return waitSettle(ctx, stopSettleDelay)
}

// Reset sends the RESET request, rebooting the RPLIDAR core back to the
// state it's in right after powering up — useful for recovering from the
// Protection Stop state. RPLIDAR sends no response to this request; Reset
// waits the protocol-mandated settle delay (or until ctx is done) before
// returning.
func (d *SerialDriver) Reset(ctx context.Context) error {
	if _, err := d.port.Write(requestPacket(cmdReset)); err != nil {
		return fmt.Errorf("lidar: sending RESET request: %w", err)
	}
	return waitSettle(ctx, resetSettleDelay)
}

// Health sends the GET_HEALTH request and returns the device's reported
// health state.
func (d *SerialDriver) Health(_ context.Context) (Health, error) {
	if _, err := d.port.Write(requestPacket(cmdGetHealth)); err != nil {
		return Health{}, fmt.Errorf("lidar: sending GET_HEALTH request: %w", err)
	}

	desc, err := d.readDescriptor()
	if err != nil {
		return Health{}, fmt.Errorf("lidar: reading GET_HEALTH response descriptor: %w", err)
	}
	if desc.dataType != dataTypeHealth {
		return Health{}, fmt.Errorf(
			"%w: got 0x%02X, want 0x%02X",
			ErrUnexpectedDataType,
			desc.dataType,
			dataTypeHealth,
		)
	}

	body := make([]byte, healthRespLen)
	if _, readErr := io.ReadFull(d.reader, body); readErr != nil {
		return Health{}, fmt.Errorf("lidar: reading GET_HEALTH response body: %w", readErr)
	}

	health, err := decodeHealth(body)
	if err != nil {
		return Health{}, fmt.Errorf("lidar: decoding GET_HEALTH response: %w", err)
	}
	return health, nil
}

// readDescriptor reads and parses the fixed 7-byte response descriptor
// that precedes every data response.
func (d *SerialDriver) readDescriptor() (descriptor, error) {
	raw := make([]byte, descLen)
	if _, err := io.ReadFull(d.reader, raw); err != nil {
		return descriptor{}, fmt.Errorf("lidar: reading response descriptor: %w", err)
	}
	return parseDescriptor(raw)
}

// readScan reads measurement samples until a full Scan (all samples
// between two S=1 start-of-scan flags) has been assembled. Samples that
// arrive before the first S=1 flag of a fresh connection are discarded —
// they belong to whatever partial scan was already in flight when
// streaming started.
func (d *SerialDriver) readScan() (Scan, error) {
	var points []Point
	started := false
	if d.first != nil {
		points = append(points, *d.first)
		d.first = nil
		started = true
	}

	for {
		raw := make([]byte, measurementLen)
		if _, err := io.ReadFull(d.reader, raw); err != nil {
			return nil, fmt.Errorf("lidar: reading measurement: %w", err)
		}

		pt, startOfScan, err := decodeMeasurement(raw)
		if err != nil {
			return nil, fmt.Errorf("lidar: decoding measurement: %w", err)
		}

		if startOfScan {
			if started {
				d.first = &pt
				return points, nil
			}
			started = true
		}
		if started {
			points = append(points, pt)
		}
	}
}

// waitSettle blocks for delay or until ctx is done, whichever comes first.
func waitSettle(ctx context.Context, delay time.Duration) error {
	timer := time.NewTimer(delay)
	defer timer.Stop()

	select {
	case <-ctx.Done():
		return fmt.Errorf("lidar: waiting for settle delay: %w", ctx.Err())
	case <-timer.C:
		return nil
	}
}
