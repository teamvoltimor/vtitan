package lidar

// Byte-level framing for the RPLIDAR classic protocol (request packets,
// response descriptors, and the 5-byte SCAN measurement structure). Zero
// I/O — everything here operates on already-read []byte, so it's testable
// without a serial port. Grounded in Slamtec's own "RPLIDAR 360 Degree
// Laser Range Scanner — Interface Protocol and Application Notes" (rev
// 2.2, 2021-02-25, applied to RPLIDAR A and S series):
// https://cdn.robotshop.com/media/r/rpk/rb-rpk-02/pdf/lr001_slamtec_rplidar_protocol_v2.2_en.pdf
// — "Request Packets' Format" / "Response Packets' Format" (pp. 6-9),
// "Start Scan (SCAN) Request and Response" (pp. 15-17), and "Get Device
// Health Status (GET_HEALTH) Request and Response" (p. 36). Cross-checked
// against the RPLIDAR C1 datasheet (rev 1.2, 2026-03-12) confirming the C1
// "is compatible with all communication protocols used by previous RPLIDAR
// S Series products and the traditional sampling protocol (standard) of
// the A Series products" (Protocol Compatibility, p.7).
//
// This is the CLASSIC scan mode (cmdClassicScan = 0x20) — the legacy,
// lower-sample-rate path every RPLIDAR model supports. It is kept as a
// standalone option alongside the Dense/Express mode implementation (see
// frame_dense.go, ClassicSerialDriver's sibling), not because it's the
// preferred mode: hardware validation 2026-08-31 found classic-mode range
// decode reading 2-4x too large versus known physical distances (angle
// decode is correct), while the same hardware's Python stack — which uses
// Dense mode via sllidar_ros2's default scan_mode="Standard" (routed
// through startScanExpress, never classic startScan) — reads correctly.
// That means classic mode's distanceQ2Scale formula, though it matches the
// protocol doc's generic Figure 4-5 text, has not actually been validated
// as correct on this specific C1 unit. Prefer the Dense implementation;
// this one remains available for comparison/fallback.
import (
	"encoding/binary"
	"errors"
	"fmt"
	"math"
	"time"

	"go.bug.st/serial"
)

// yawOffsetDeg is the mounting rotation applied to every decoded
// measurement angle (see Config.YawOffsetDeg). It is a package-level value
// rather than a decodeClassicMeasurement parameter because
// decodeClassicMeasurement is on the documented wire-format signature
// (Figure 4-4) and is also exercised by the frame-unit tests with no
// offset; NewClassic/Connect install it from Config.
var yawOffsetDeg float64

type (
	// descriptor is a parsed RPLIDAR response descriptor: the fixed 7-byte
	// header (0xA5 0x5A + 30-bit length + 2-bit send mode + 1-byte data
	// type) that precedes every data response. Shared framing, used by
	// both classic and Dense/Express modes.
	descriptor struct {
		length   uint32
		sendMode byte
		dataType byte
	}

	// Point is one decoded RPLIDAR measurement sample: angle and range in
	// the units sensor_msgs/LaserScan (and this repo's
	// proto/vtitan/sensor/v1/scan.proto, which mirrors it) use — radians
	// and meters, not the wire format's degrees/mm. Shared output type
	// across scan modes.
	Point struct {
		AngleRad float64
		RangeM   float64
		Quality  uint8
	}

	// Health is a decoded GET_HEALTH response.
	Health struct {
		Status    HealthStatus
		ErrorCode uint16
	}

	// HealthStatus is RPLIDAR's 3-value device health state.
	HealthStatus uint8

	// Scan is one full 360-degree revolution's worth of measurement
	// samples, assembled from consecutive SCAN measurements between two
	// S=1 (start-of-new-scan) flags. This is this package's Driver[T]
	// reading type — analogous to imu.Reading, but a LIDAR's natural unit
	// of data is a full scan, not a single sample. Shared output type
	// across scan modes.
	Scan []Point
)

// HealthStatus string representations, returned by HealthStatus.String().
const (
	healthGoodString    = "good"
	healthWarningString = "warning"
	healthErrorString   = "error"
	healthUnknownString = "unknown"
)

// HealthStatus values, matching the wire encoding in Figure 4-29 of the
// protocol doc exactly (0=Good, 1=Warning, 2=Error) — do not reorder.
const (
	HealthGood HealthStatus = iota
	HealthWarning
	HealthError
)

const (
	// reqStartFlag is the fixed first byte of every request packet (§
	// "Request Packets' Format"). Shared across scan modes.
	reqStartFlag = 0xA5

	// cmdStop, cmdReset, cmdClassicScan, cmdGetHealth are the command
	// bytes this package implements. Values from Figure 4-1 "The
	// Available Requests of RPLIDAR". cmdStop/cmdReset/cmdGetHealth are
	// shared across scan modes; cmdClassicScan starts classic mode
	// specifically (Dense/Express mode uses its own command, see
	// frame_dense.go).
	cmdStop        = 0x25
	cmdReset       = 0x40
	cmdClassicScan = 0x20
	cmdGetHealth   = 0x52
	// cmdSetMotorPwm starts/spins the C1's motor (SL_LIDAR_CMD_SET_MOTOR_PWM,
	// SDK sl_lidar_cmd.h). The RPLIDAR C1 will not stream scan data unless its
	// motor is spinning, so both scan modes must issue this before requesting
	// a scan (the sllidar SDK's startScanExpress/startScanNormal both call
	// startMotor() first — see sl_lidar_driver.cpp startScanExpress line 59).
	cmdSetMotorPwm = 0xF0
	// cmdHQMatorSpeedCtrl (SL_LIDAR_CMD_HQ_MOTOR_SPEED_CTRL) is the
	// alternative motor-start command the sllidar SDK uses for RPM-mode motor
	// control. On the C1 this is the variant that actually spins the motor up
	// and lets the Express Scan stream (verified on hardware 2026-08-31: the
	// 0xF0 PWM command left the device unresponsive to the scan request,
	// while 0xA8 with an RPM payload produced a live capsule stream).
	cmdHQMatorSpeedCtrl = 0xA8

	// descStartFlag1/2 are the fixed two bytes identifying the start of a
	// response descriptor (§ "Response Packets' Format", Figure 2-7).
	// Shared across scan modes.
	descStartFlag1 = 0xA5
	descStartFlag2 = 0x5A
	// descLen is the total byte length of a response descriptor: 2 sync
	// bytes + 4 bytes packing {30-bit length, 2-bit send mode} + 1 data
	// type byte. Shared across scan modes.
	descLen = 7
	// descLengthMask isolates the 30-bit Data Response Length field from
	// the combined length/send-mode 32-bit word (bits 31-30 are send
	// mode). Shared across scan modes.
	descLengthMask = 0x3FFFFFFF
	descModeShift  = 30

	// dataTypeClassicMeasurement is the classic SCAN response descriptor's
	// Data Type byte (Figure 4-4/4-8: response descriptor
	// "A5 5A 05 00 00 40 81" — the trailing 0x81 is this field).
	dataTypeClassicMeasurement = 0x81
	// dataTypeHealth is the GET_HEALTH response descriptor's Data Type
	// byte (§ "Get Device Health Status", response descriptor
	// "A5 5A 3 00 00 00 06"). Shared across scan modes.
	dataTypeHealth = 0x06

	// classicMeasurementLen is the fixed size in bytes of one classic
	// SCAN measurement sample (Figure 4-4 "Format of a RPLIDAR
	// Measurement Result Data Response Packet").
	classicMeasurementLen = 5
	// healthRespLen is the fixed size in bytes of a GET_HEALTH data
	// response (Figure 4-28). Shared across scan modes.
	healthRespLen = 3

	// angleQ6Scale converts a packed 15-bit angle_q6 fixed-point value to
	// degrees: "Actual heading = angle_q6/64.0 Degree" (Figure 4-5).
	// Shared: both classic mode's per-sample angle_q6 and Dense mode's
	// per-packet start_angle_q6 (Figure 4-13) use this same conversion.
	angleQ6Scale = 64.0
	// classicDistanceQ2Scale converts classic mode's packed 16-bit
	// distance_q2 fixed-point value to millimeters: "Actual Distance =
	// distance_q2/4.0 mm" (Figure 4-5). NOTE: on the RPLIDAR C1 this
	// over-scales real distances by 2-4x versus known physical distances
	// (hardware test 2026-08-31, angle decode confirmed correct
	// separately) — see this file's package comment. Dense mode
	// (frame_dense.go) does not use this constant; its cabin distance
	// field is plain millimeters, no Q2 scaling.
	classicDistanceQ2Scale = 4.0
	// mmPerMeter converts the wire format's millimeter distance unit to
	// the meter unit this package's Point.RangeM and
	// proto/vtitan/sensor/v1/scan.proto both use. Shared across scan
	// modes.
	mmPerMeter = 1000.0
	// degToRad converts the wire format's degree angle unit to the radian
	// unit this package's Point.AngleRad and
	// proto/vtitan/sensor/v1/scan.proto both use. Shared across scan
	// modes.
	degToRad = 3.141592653589793 / 180.0

	// classicAngleHighByteShift is how far byte 2 (angle_q6[14:7]) is
	// shifted left to combine with byte 1's low 7 bits (angle_q6[6:0]).
	classicAngleHighByteShift = 7
	// highByteShift is how far a value's high byte is shifted left to
	// combine with its low byte in the wire format's little-endian 16-bit
	// fields: classic mode's distance_q2[15:8] (Figure 4-4) and
	// GET_HEALTH's error_code[15:8] (Figure 4-28). Shared across scan
	// modes.
	highByteShift = 8

	// classicMeasurementSBit and classicMeasurementSInvBit are byte 0's
	// start-of-scan flag and its inverted self-check pair (Figure 4-4:
	// "Quality | ~S | S", bit-position markers 8..2, 1, 0 — S is bit 0,
	// ~S is bit 1).
	classicMeasurementSBit         = 0x01
	classicMeasurementSInvBit      = 0x02
	classicMeasurementQualityShift = 2

	// classicMeasurementCheckBit is byte 1's bit 0 (C), which the
	// protocol always sets to 1 as a second self-check alongside S/~S.
	classicMeasurementCheckBit = 0x01
)

var (
	// ErrShortBuffer means fewer bytes were supplied than the structure
	// being decoded requires. Shared across scan modes.
	ErrShortBuffer = errors.New("lidar: buffer shorter than expected")
	// ErrBadDescriptorSync means a supposed response descriptor didn't
	// start with the fixed 0xA5 0x5A sync bytes. Shared across scan
	// modes.
	ErrBadDescriptorSync = errors.New("lidar: response descriptor sync mismatch")
	// ErrUnexpectedDataType means a response descriptor's data type byte
	// didn't match what the caller requested (e.g. a SCAN request got a
	// non-measurement descriptor back). Shared across scan modes.
	ErrUnexpectedDataType = errors.New("lidar: unexpected response descriptor data type")
	// ErrReadTimeout means a serial read returned no data before the port's
	// configured read timeout elapsed (surfaced by timeoutReader wrapping
	// go.bug.st/serial's (0, nil) timeout quirk). Shared across scan modes.
	ErrReadTimeout = errors.New("lidar: serial read timed out")
	// ErrSyncBitMismatch means a classic measurement sample's S and ~S
	// bits (byte 0, bits 0-1) weren't complementary — the sample is
	// corrupt.
	ErrSyncBitMismatch = errors.New("lidar: measurement S/~S sync bits not complementary")
	// ErrCheckBitUnset means a classic measurement sample's C check bit
	// (byte 1, bit 0), which the protocol defines as "constantly set to
	// 1", was 0 — the sample is corrupt or misaligned.
	ErrCheckBitUnset = errors.New("lidar: measurement check bit C not set")
	// ErrUnknownHealthStatus means a GET_HEALTH response's status byte was
	// outside the documented 0-2 range. Shared across scan modes.
	ErrUnknownHealthStatus = errors.New("lidar: unknown health status value")
)

// requestPacket builds a no-payload request packet for cmd. STOP, RESET,
// classic SCAN, and GET_HEALTH all carry no payload, so a request is
// always exactly the 2-byte {reqStartFlag, cmd} pair — the
// payload-size/payload/checksum fields "Request Packets' Format" describes
// are only present when a request carries a payload, which none of these
// do (verified against each request's own worked example in the protocol
// doc, e.g. STOP is literally "A5 25", nothing more). Shared across scan
// modes; Dense/Express mode's own 5-byte-payload request is built
// separately (see frame_dense.go).
func requestPacket(cmd byte) []byte {
	return []byte{reqStartFlag, cmd}
}

// payloadRequestPacket builds a request that carries a payload: {reqStartFlag,
// cmd, payloadSize, payload..., checksum}, where checksum is the XOR of every
// byte from reqStartFlag through the last payload byte (per the SDK's
// RPLidarProtocolCodec::onEncodeData — the C1 rejects the request, returning
// nothing, if the checksum omits any of these). Used by the motor-start
// command and the Dense/Express scan request.
func payloadRequestPacket(cmd byte, payload []byte) []byte {
	checksum := byte(reqStartFlag) ^ cmd ^ byte(len(payload))
	for _, p := range payload {
		checksum ^= p
	}
	pkt := make([]byte, 0, 3+len(payload)+1)
	pkt = append(pkt, reqStartFlag, cmd, byte(len(payload)))
	pkt = append(pkt, payload...)
	pkt = append(pkt, checksum)
	return pkt
}

// startMotorPacket builds the SET_MOTOR_PWM request that spins the C1 motor up
// to its default speed. Must be sent before any scan request, or the device
// answers the scan request with no data stream (verified on hardware
// 2026-08-31: Express Scan returned zero bytes until the motor was started).
// motorDefaultRpm is the 16-bit RPM value sent to start the motor via the
// HQ motor-speed command. The sllidar SDK's startMotor() drives the motor to a
// nominal speed; 600 RPM matches the spin-up observed streaming correctly on
// the C1 (verified on hardware 2026-08-31). Declared as a var (not a const)
// so it can be split into bytes at runtime without a constant-width overflow.
var motorDefaultRpm uint16 = 600

func startMotorPacket() []byte {
	return payloadRequestPacket(cmdHQMatorSpeedCtrl, []byte{byte(motorDefaultRpm), byte(motorDefaultRpm >> 8)})
}

// timeoutReader wraps a serial.Port so that the underlying driver's
// "timed-out read returns (0, nil)" quirk (go.bug.st/serial v1.8.0,
// serial_unix.go:93) doesn't cause an endless retry. bufio.Reader.fill (and
// io.ReadFull) treat a (0, nil) read as "no data available yet, try again"
// and loop forever, which otherwise hangs Connect and Read indefinitely
// whenever the device is slow or silent — confirmed on the C1 (2026-08-31: a
// non-streaming lidar hung the dense driver for the full test timeout).
//
// Instead of erroring on the very first empty read (which would also kill a
// healthy scan that has a brief inter-packet gap), timeoutReader tolerates
// short silences up to maxSilence, then returns ErrReadTimeout so a truly
// stalled device still fails fast.
type timeoutReader struct {
	port       serial.Port
	maxSilence time.Duration
}

func (t *timeoutReader) Read(p []byte) (int, error) {
	var silence time.Duration
	for {
		n, err := t.port.Read(p)
		if n > 0 || err != nil {
			return n, err
		}
		// n == 0 && err == nil: the serial lib's timeout quirk.
		silence += t.portReadTimeout()
		if silence >= t.maxSilence {
			return 0, fmt.Errorf("%w: serial read returned no data within %s", ErrReadTimeout, t.maxSilence)
		}
	}
}

// portReadTimeout reports the underlying port's configured read timeout (the
// duration of a single (0, nil) empty read). It defaults to a small value if
// the port doesn't expose one.
func (t *timeoutReader) portReadTimeout() time.Duration {
	if p, ok := t.port.(interface{ ReadTimeout() time.Duration }); ok {
		return p.ReadTimeout()
	}
	return 250 * time.Millisecond
}

// newTimeoutReader wraps port in a timeoutReader. SetReadTimeout on the
// underlying port controls the per-call wait; maxSilence bounds how long a
// total silence is tolerated before erroring.
func newTimeoutReader(port serial.Port, maxSilence time.Duration) *timeoutReader {
	return &timeoutReader{port: port, maxSilence: maxSilence}
}

// parseDescriptor decodes a 7-byte response descriptor. Shared across scan
// modes.
func parseDescriptor(b []byte) (descriptor, error) {
	if len(b) < descLen {
		return descriptor{}, fmt.Errorf(
			"%w: descriptor needs %d bytes, got %d",
			ErrShortBuffer,
			descLen,
			len(b),
		)
	}
	if b[0] != descStartFlag1 || b[1] != descStartFlag2 {
		return descriptor{}, fmt.Errorf("%w: got 0x%02X 0x%02X", ErrBadDescriptorSync, b[0], b[1])
	}

	packed := binary.LittleEndian.Uint32(b[2:6])
	return descriptor{
		length:   packed & descLengthMask,
		sendMode: byte(packed >> descModeShift),
		dataType: b[6],
	}, nil
}

// decodeClassicMeasurement decodes one 5-byte classic SCAN measurement
// sample (Figure 4-4 field layout). It returns the decoded Point, whether
// S (byte 0 bit 0) marks the start of a new 360-degree scan, and an error
// if the sample fails either of the protocol's two self-check bits (S/~S
// complementary, C set).
func decodeClassicMeasurement(b []byte) (pt Point, startOfScan bool, err error) {
	if len(b) < classicMeasurementLen {
		return Point{}, false, fmt.Errorf(
			"%w: measurement needs %d bytes, got %d", ErrShortBuffer, classicMeasurementLen, len(b),
		)
	}

	// Byte 0: bits [7:2] quality, bit 1 = ~S (inverted start flag), bit 0
	// = S (start flag) — Figure 4-4 draws the byte MSB-first as
	// "Quality | ~S | S" with bit-position markers 8..2, 1, 0, so S is
	// bit 0 and ~S is bit 1. (Careful: this is the opposite pairing of a
	// naive "S is the higher bit" reading of the two adjacent single-bit
	// boxes — verified against the figure's explicit bit-position
	// numbers, not just box order.)
	startOfScan = b[0]&classicMeasurementSBit != 0
	inverted := b[0]&classicMeasurementSInvBit != 0
	if startOfScan == inverted {
		return Point{}, false, fmt.Errorf("%w: byte0=0x%02X", ErrSyncBitMismatch, b[0])
	}
	quality := b[0] >> classicMeasurementQualityShift

	// Byte 1: bits [7:1] = angle_q6[6:0], bit 0 = C (check bit, always 1).
	// Byte 2: bits [7:0] = angle_q6[14:7].
	if b[1]&classicMeasurementCheckBit == 0 {
		return Point{}, false, fmt.Errorf("%w: byte1=0x%02X", ErrCheckBitUnset, b[1])
	}
	angleQ6 := uint16(b[1]>>1) | uint16(b[2])<<classicAngleHighByteShift
	angleDeg := float64(angleQ6) / angleQ6Scale

	// Byte 3: distance_q2[7:0]. Byte 4: distance_q2[15:8].
	distanceQ2 := uint16(b[3]) | uint16(b[4])<<highByteShift
	rangeMM := float64(distanceQ2) / classicDistanceQ2Scale

	// Apply the fixed mounting yaw offset so decoded 0 == robot forward
	// (see Config.YawOffsetDeg). yawOffsetDeg is the package-level value
	// installed from Config by NewClassic/Connect.
	angleDeg += yawOffsetDeg

	// Normalize to [0, 360) so a large positive/negative offset can't push
	// angles outside the conventional lidar range (e.g. offset 180 landing
	// at 412 deg). Keeps the reported angle well-formed for consumers.
	angleDeg = math.Mod(angleDeg, 360)
	if angleDeg < 0 {
		angleDeg += 360
	}

	return Point{
		AngleRad: angleDeg * degToRad,
		RangeM:   rangeMM / mmPerMeter,
		Quality:  quality,
	}, startOfScan, nil
}

// decodeHealth decodes a 3-byte GET_HEALTH data response (Figure 4-28).
// Shared across scan modes.
func decodeHealth(b []byte) (Health, error) {
	if len(b) < healthRespLen {
		return Health{}, fmt.Errorf(
			"%w: health response needs %d bytes, got %d", ErrShortBuffer, healthRespLen, len(b),
		)
	}

	status := b[0]
	if status > byte(HealthError) {
		return Health{}, fmt.Errorf("%w: got %d", ErrUnknownHealthStatus, status)
	}

	return Health{
		Status:    HealthStatus(status),
		ErrorCode: uint16(b[1]) | uint16(b[2])<<highByteShift,
	}, nil
}

// String returns s's lowercase name ("good", "warning", "error"), or
// "unknown" for any value outside the documented 0-2 range.
func (s HealthStatus) String() string {
	switch s {
	case HealthGood:
		return healthGoodString
	case HealthWarning:
		return healthWarningString
	case HealthError:
		return healthErrorString
	default:
		return healthUnknownString
	}
}
