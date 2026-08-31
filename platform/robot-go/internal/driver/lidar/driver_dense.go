package lidar

import (
	"bufio"
	"context"
	"fmt"
	"io"
	"time"

	"github.com/go-playground/validator/v10"
	"go.bug.st/serial"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver"
)

// DenseSerialDriver reads 360-degree Scans from an RPLIDAR C1 over its TTL
// UART interface using the Express Scan "Dense Mode" command (see doc.go
// for scope, and frame_dense.go's package comment for why this is the
// preferred mode over ClassicSerialDriver). It implements
// driver.Driver[Scan] (platform/robot-go/internal/driver). Shares Config,
// DefaultBaudRate, MinRangeM/MaxRangeM, and the settle-delay/timeout
// constants with ClassicSerialDriver (driver_classic.go) — those are
// protocol- and hardware-generic, not scan-mode-specific.
type DenseSerialDriver struct {
	cfg    Config
	port   serial.Port
	reader *bufio.Reader
	// packetLen is the exact byte size of one Dense Mode response packet
	// for this session, taken from the response descriptor's declared
	// Data Response Length (Figure 2-7) in Connect -- not assumed to be
	// the documented worked example's 84 bytes, in case this device's
	// actual legacy-mode packet size differs. Set once by Connect, then
	// read-only for the life of the connection.
	packetLen int
	// prev holds the most recently decoded Dense Mode packet, whose
	// samples can't be angle-resolved yet — that requires the *next*
	// packet's start_angle_q6 (see frame_dense.go's package comment).
	// Carried across Read calls the same way ClassicSerialDriver.first
	// carries one already-decoded Point.
	prev *densePacket
	// pending holds prev's already-resolved Points once the boundary
	// packet (prev.startOfScan) has been reached but the completed Scan
	// has already been returned — they belong to the *next* Scan and are
	// carried over between Read calls instead of being discarded.
	pending []Point
}

var (
	// Compile-time assertion that DenseSerialDriver satisfies
	// driver.Driver[Scan].
	_ driver.Driver[Scan] = (*DenseSerialDriver)(nil)
)

// NewDense validates cfg and returns a DenseSerialDriver. Call Connect
// before Read.
func NewDense(cfg Config) (*DenseSerialDriver, error) {
	if err := validator.New().Struct(cfg); err != nil {
		return nil, fmt.Errorf("lidar: invalid config: %w", err)
	}
	yawOffsetDeg = cfg.YawOffsetDeg
	return &DenseSerialDriver{cfg: cfg}, nil
}

// Connect opens the configured serial port, stops any scan already in
// progress (the device may still be scanning from a previous session that
// didn't clean up), and issues the legacy Dense Mode Express Scan request
// so measurement samples start streaming.
func (d *DenseSerialDriver) Connect(ctx context.Context) error {
	mode := &serial.Mode{BaudRate: d.cfg.BaudRate}
	port, err := serial.Open(d.cfg.Port, mode)
	if err != nil {
		return fmt.Errorf("lidar: opening serial port %s: %w", d.cfg.Port, err)
	}
	d.port = port
	d.reader = bufio.NewReader(newTimeoutReader(port, scanReadTimeout))
	// The serial port's per-call read timeout is kept short; the
	// timeoutReader's maxSilence (scanReadTimeout) bounds a stalled read so a
	// device that never answers the Express Scan request fails fast instead of
	// hanging (go.bug.st/serial returns (0, nil) on timeout, which otherwise
	// loops forever in bufio). scanReadTimeout is set above the C1's observed
	// ~2.1s inter-scan gap (measured on hardware 2026-08-31) so a healthy
	// scan assembles across bursts, while a truly dead device still errors.
	if err := d.port.SetReadTimeout(serialPollTimeout); err != nil {
		return fmt.Errorf("lidar: setting read timeout: %w", err)
	}

	// Best-effort: if the device is already scanning from a prior session,
	// this stops it so the Express Scan request below starts a clean
	// session. A fresh device that isn't scanning simply ignores it ("This
	// request will be ignored when RPLIDAR is in the Idle or Protection
	// Stop state.").
	if stopErr := d.Stop(ctx); stopErr != nil {
		return fmt.Errorf("lidar: stopping prior scan session: %w", stopErr)
	}

	// Reboot the RPLIDAR core to a clean idle state. The sllidar SDK's
	// connect sequence issues a RESET before starting a scan, and on the C1
	// the Express Scan request is otherwise sometimes silently ignored
	// (verified on hardware 2026-08-31: streaming only began reliably after
	// a RESET + motor-start, matching the SDK's flow).
	if resetErr := d.Reset(ctx); resetErr != nil {
		return fmt.Errorf("lidar: resetting device: %w", resetErr)
	}

	// The C1 will not stream scan data unless its motor is spinning, so start
	// it before requesting the scan (mirrors the sllidar SDK's startMotor()
	// call inside startScanExpress). Without this the Express Scan request is
	// silently ignored and the device returns no data (verified on hardware
	// 2026-08-31).
	if _, writeErr := d.port.Write(startMotorPacket()); writeErr != nil {
		return fmt.Errorf("lidar: starting motor: %w", writeErr)
	}
	// The C1 ignores the Express Scan request until the motor is actually
	// spinning, so wait for spin-up before issuing it (see motorSpinupDelay).
	time.Sleep(motorSpinupDelay)

	// Purge any measurement bytes the device already had queued on the wire
	// before we sent STOP -- otherwise the descriptor read below picks up a
	// stale sample instead of the real response descriptor (same rationale
	// as ClassicSerialDriver.Connect).
	if err := d.port.ResetInputBuffer(); err != nil {
		return fmt.Errorf("lidar: purging stale input: %w", err)
	}

	if _, writeErr := d.port.Write(denseRequestPacket()); writeErr != nil {
		return fmt.Errorf("lidar: sending Express Scan request: %w", writeErr)
	}

	desc, descErr := d.readDescriptor()
	if descErr != nil {
		return fmt.Errorf("lidar: reading Express Scan response descriptor: %w", descErr)
	}
	if desc.dataType != dataTypeDenseMeasurement {
		return fmt.Errorf(
			"%w: got 0x%02X, want 0x%02X",
			ErrUnexpectedDataType,
			desc.dataType,
			dataTypeDenseMeasurement,
		)
	}
	if desc.length < denseHeaderLen {
		return fmt.Errorf(
			"lidar: Express Scan response descriptor declared %d byte packets, too short for a %d byte header",
			desc.length, denseHeaderLen,
		)
	}
	d.packetLen = int(desc.length)

	return nil
}

// Read blocks until a full Scan (all samples between two S=1 start-of-scan
// flags) has been assembled, or until ctx is done. If ctx is canceled
// while a read is in flight, the underlying goroutine is left blocked on
// the serial read until Close is called — Close closing the port is what
// unblocks it, same pattern as ClassicSerialDriver.Read.
func (d *DenseSerialDriver) Read(ctx context.Context) (Scan, error) {
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
func (d *DenseSerialDriver) Close() error {
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

// Stop sends the STOP request, exiting the scanning state. Shared
// STOP/RESET/GET_HEALTH command bytes and settle-delay handling with
// ClassicSerialDriver — those requests aren't scan-mode-specific.
func (d *DenseSerialDriver) Stop(ctx context.Context) error {
	if _, err := d.port.Write(requestPacket(cmdStop)); err != nil {
		return fmt.Errorf("lidar: sending STOP request: %w", err)
	}
	return waitSettle(ctx, stopSettleDelay)
}

// Reset sends the RESET request, rebooting the RPLIDAR core back to the
// state it's in right after powering up — useful for recovering from the
// Protection Stop state.
func (d *DenseSerialDriver) Reset(ctx context.Context) error {
	if _, err := d.port.Write(requestPacket(cmdReset)); err != nil {
		return fmt.Errorf("lidar: sending RESET request: %w", err)
	}
	return waitSettle(ctx, resetSettleDelay)
}

// Health sends the GET_HEALTH request and returns the device's reported
// health state.
func (d *DenseSerialDriver) Health(_ context.Context) (Health, error) {
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
// that precedes every data response. Same resync-on-sync-pair robustness
// as ClassicSerialDriver.readDescriptor, and for the same reason (a
// leftover scan or STOP response can leave stray bytes ahead of the real
// descriptor).
func (d *DenseSerialDriver) readDescriptor() (descriptor, error) {
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

// readScan reads Dense Mode packets until a full Scan (all samples between
// two S=1 start-of-scan flags) has been assembled. Because a packet's own
// samples can only be angle-resolved once the *next* packet's start angle
// is known (frame_dense.go's package comment), this carries both an
// unresolved packet (d.prev) and an already-resolved-but-not-yet-returned
// point batch (d.pending) across Read calls — the two-field generalization
// of ClassicSerialDriver.readScan's single d.first carryover.
func (d *DenseSerialDriver) readScan() (Scan, error) {
	var points []Point
	started := false
	if d.pending != nil {
		points = append(points, d.pending...)
		d.pending = nil
		started = true
	}

	prev := d.prev
	d.prev = nil

	for {
		raw := make([]byte, d.packetLen)
		if _, err := io.ReadFull(d.reader, raw); err != nil {
			return nil, fmt.Errorf("lidar: reading dense packet: %w", err)
		}

		cur, err := decodeDensePacket(raw)
		if err != nil {
			return nil, fmt.Errorf("lidar: decoding dense packet: %w", err)
		}

		if prev != nil {
			resolved := resolveDenseCabins(*prev, cur.startAngleDeg)

			if prev.startOfScan {
				started = true
			}
			if started {
				points = append(points, resolved...)

				// The C1 Express/Dense stream only flags the start of a scan
				// (S=1) on its first packet; subsequent packets never re-set
				// S, so a scan can't be closed on a second S flag (verified on
				// hardware 2026-08-31: S was true exactly once per stream).
				// Instead the scan ends when the per-packet start angle wraps
				// back toward 0 (i.e. drops below the previous packet's angle
				// after having increased monotonically through 360deg). Close
				// the scan on that wrap.
				if prev.startAngleDeg > cur.startAngleDeg+scanWrapAngleDeg {
					d.prev = &cur
					d.pending = resolved
					return points, nil
				}
			}
		}

		prev = &cur
	}
}
