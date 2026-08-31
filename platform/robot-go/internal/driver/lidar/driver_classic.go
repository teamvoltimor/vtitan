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

// Config configures a ClassicSerialDriver's serial connection to an
// RPLIDAR C1.
type Config struct {
	Port     string `validate:"required"`
	BaudRate int    `validate:"required,gt=0"`
	// YawOffsetDeg rotates every decoded measurement angle by a fixed
	// mounting offset so that decoded angle 0 (the sensor's own 0 reference)
	// maps to the robot's forward axis. The physical lidar is mounted rotated
	// relative to the chassis; without this, "front" reads at the mounting
	// angle instead of 0. Mirrors the Python launch's inverted/inverted-yaw
	// handling. Applied in decodeClassicMeasurement (frame_classic.go)
	// before AngleRad is stored. Zero means "sensor 0 == robot forward".
	//
	// The single source of truth is robot.toml's [lidar] section: the total
	// correction is LidarYawOffsetRad() = 180deg when inverted=true plus
	// mount_yaw_offset_deg (0 on this robot). Only the interactive hardware
	// tests set this so decoded angles read in the robot frame directly;
	// production lidar-node keeps it 0 and publishes raw C1 bearings, with
	// consumers (telemetry diag, the nav gateway) applying the same offset
	// via profile.RobotConfig.LidarYawOffsetRad().
	YawOffsetDeg float64
}

// ClassicSerialDriver reads 360-degree Scans from an RPLIDAR C1 over its
// TTL UART interface using the classic SCAN command (see doc.go for scope,
// and frame_classic.go's package comment for why this is kept alongside
// the Dense/Express mode implementation rather than as the sole driver).
// It implements driver.Driver[Scan] (platform/robot-go/internal/driver).
type ClassicSerialDriver struct {
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
	// resetSettleDelay is how long Connect waits after a RESET before issuing
	// the next request. The RPLIDAR C1 reboots its core on RESET, which takes
	// far longer than the protocol's typical few-ms stop settle — sending the
	// scan request during the reboot is silently ignored and the descriptor
	// never arrives (verified on hardware 2026-08-31: a 2ms delay hung/failed,
	// ~1s succeeds). 1s is conservative but well within the serial read
	// timeout, so Connect still fails fast if the device is truly unresponsive.
	resetSettleDelay = 1 * time.Second
	// motorSpinupDelay is how long Connect waits after starting the motor
	// before requesting the scan. The C1 ignores the Express Scan request
	// (and so never returns its descriptor) until the motor is actually
	// spinning; an ~800ms spin-up window matches the validated sllidar/Python
	// flow and the hardware probe that first got the C1 streaming
	// (2026-08-31).
	motorSpinupDelay = 800 * time.Millisecond
	// connectReadTimeout bounds every Read after Connect opens the port, so a
	// device that never answers the SCAN request fails fast instead of hanging.
	connectReadTimeout = 2 * time.Second
	// scanReadTimeout is the per-read silence tolerance used while streaming
	// scan data. The RPLIDAR C1 Express/Dense stream emits one scan then
	// pauses ~2.1s before the next (measured on hardware 2026-08-31); this
	// must exceed that gap so a healthy scan assembles, while still bounding a
	// truly stalled device.
	scanReadTimeout = 4 * time.Second
)

var (
	errReadBeforeConnect = errors.New("lidar: Read called before Connect")

	// Compile-time assertion that ClassicSerialDriver satisfies
	// driver.Driver[Scan].
	_ driver.Driver[Scan] = (*ClassicSerialDriver)(nil)
)

// NewClassic validates cfg and returns a ClassicSerialDriver. Call Connect
// before Read.
func NewClassic(cfg Config) (*ClassicSerialDriver, error) {
	if err := validator.New().Struct(cfg); err != nil {
		return nil, fmt.Errorf("lidar: invalid config: %w", err)
	}
	yawOffsetDeg = cfg.YawOffsetDeg
	return &ClassicSerialDriver{cfg: cfg}, nil
}

// Connect opens the configured serial port, stops any scan already in
// progress (the device may still be scanning from a previous session that
// didn't clean up), and issues the classic SCAN request so measurement
// samples start streaming.
func (d *ClassicSerialDriver) Connect(ctx context.Context) error {
	mode := &serial.Mode{BaudRate: d.cfg.BaudRate}
	port, err := serial.Open(d.cfg.Port, mode)
	if err != nil {
		return fmt.Errorf("lidar: opening serial port %s: %w", d.cfg.Port, err)
	}
	d.port = port
	d.reader = bufio.NewReader(newTimeoutReader(port, scanReadTimeout))
	// The serial port's per-call read timeout is kept short; the
	// timeoutReader's maxSilence (connectReadTimeout) bounds a stalled read.
	if err := d.port.SetReadTimeout(serialPollTimeout); err != nil {
		return fmt.Errorf("lidar: setting read timeout: %w", err)
	}

	// Best-effort: if the device is already scanning from a prior session,
	// this stops it so the SCAN request below starts a clean session. A
	// fresh device that isn't scanning simply ignores it ("This request
	// will be ignored when RPLIDAR is in the Idle or Protection Stop
	// state.").
	if stopErr := d.Stop(ctx); stopErr != nil {
		return fmt.Errorf("lidar: stopping prior scan session: %w", stopErr)
	}

	// Purge any measurement bytes the device already had queued on the wire
	// before we sent STOP -- otherwise the SCAN descriptor read below picks up
	// a stale sample instead of the real response descriptor (seen live: a
	// previously-running sllidar node left the RX buffer full of scan data).
	if err := d.port.ResetInputBuffer(); err != nil {
		return fmt.Errorf("lidar: purging stale input: %w", err)
	}

	if _, writeErr := d.port.Write(requestPacket(cmdClassicScan)); writeErr != nil {
		return fmt.Errorf("lidar: sending SCAN request: %w", writeErr)
	}

	desc, descErr := d.readDescriptor()
	if descErr != nil {
		return fmt.Errorf("lidar: reading SCAN response descriptor: %w", descErr)
	}
	if desc.dataType != dataTypeClassicMeasurement {
		return fmt.Errorf(
			"%w: got 0x%02X, want 0x%02X",
			ErrUnexpectedDataType,
			desc.dataType,
			dataTypeClassicMeasurement,
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
func (d *ClassicSerialDriver) Read(ctx context.Context) (Scan, error) {
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
func (d *ClassicSerialDriver) Close() error {
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
func (d *ClassicSerialDriver) Stop(ctx context.Context) error {
	if _, err := d.port.Write(requestPacket(cmdStop)); err != nil {
		return fmt.Errorf("lidar: sending STOP request: %w", err)
	}
	return waitSettle(ctx, stopSettleDelay)
}

// drain is unused; stale input is purged via ResetInputBuffer in Connect.

// Reset sends the RESET request, rebooting the RPLIDAR core back to the
// state it's in right after powering up — useful for recovering from the
// Protection Stop state. RPLIDAR sends no response to this request; Reset
// waits the protocol-mandated settle delay (or until ctx is done) before
// returning.
func (d *ClassicSerialDriver) Reset(ctx context.Context) error {
	if _, err := d.port.Write(requestPacket(cmdReset)); err != nil {
		return fmt.Errorf("lidar: sending RESET request: %w", err)
	}
	return waitSettle(ctx, resetSettleDelay)
}

// Health sends the GET_HEALTH request and returns the device's reported
// health state.
func (d *ClassicSerialDriver) Health(_ context.Context) (Health, error) {
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
// that precedes every data response. It resyncs on the 0xA5 0x5A sync pair
// (scanning byte-by-byte) before reading the remaining 5 descriptor bytes,
// rather than assuming the stream is already aligned at the descriptor
// start. This mirrors imu.readFrame's robustness: after a STOP (or a
// leftover scan still streaming from a prior session) there can be one or
// more stray bytes queued ahead of the real SCAN response descriptor, and a
// fixed 7-byte read would otherwise start mid-descriptor and fail with a
// "response descriptor sync mismatch" even though the device answered
// correctly. See f404ce99 / the live "got 0x5A 0x4B" failure this guards
// against.
func (d *ClassicSerialDriver) readDescriptor() (descriptor, error) {
	var prev byte
	for {
		b, err := d.reader.ReadByte()
		if err != nil {
			return descriptor{}, fmt.Errorf("lidar: reading descriptor sync: %w", err)
		}
		if prev == descStartFlag1 && b == descStartFlag2 {
			break
		}
		prev = b
	}

	rest := make([]byte, descLen-2)
	if _, err := io.ReadFull(d.reader, rest); err != nil {
		return descriptor{}, fmt.Errorf("lidar: reading response descriptor: %w", err)
	}

	raw := make([]byte, 0, descLen)
	raw = append(raw, descStartFlag1, descStartFlag2)
	raw = append(raw, rest...)
	return parseDescriptor(raw)
}

// readScan reads measurement samples until a full Scan (all samples
// between two S=1 start-of-scan flags) has been assembled. Samples that
// arrive before the first S=1 flag of a fresh connection are discarded —
// they belong to whatever partial scan was already in flight when
// streaming started.
func (d *ClassicSerialDriver) readScan() (Scan, error) {
	var points []Point
	started := false
	if d.first != nil {
		points = append(points, *d.first)
		d.first = nil
		started = true
	}

	for {
		raw := make([]byte, classicMeasurementLen)
		if _, err := io.ReadFull(d.reader, raw); err != nil {
			return nil, fmt.Errorf("lidar: reading measurement: %w", err)
		}

		pt, startOfScan, err := decodeClassicMeasurement(raw)
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
