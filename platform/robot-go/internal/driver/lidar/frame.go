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
// against the RPLIDAR C1 datasheet (rev 1.1, 2024-03-12) confirming the C1
// "is compatible with all communication protocols used by previous RPLIDAR
// S Series products and the traditional sampling protocol (standard) of
// the A Series products" (Protocol Compatibility, p.7) — i.e. the classic
// SCAN command this package implements is the correct, documented mode for
// this exact sensor, not a guess ported from a different RPLIDAR model.

import (
	"encoding/binary"
	"errors"
	"fmt"
)

type (
	// descriptor is a parsed RPLIDAR response descriptor: the fixed 7-byte
	// header (0xA5 0x5A + 30-bit length + 2-bit send mode + 1-byte data
	// type) that precedes every data response.
	descriptor struct {
		length   uint32
		sendMode byte
		dataType byte
	}

	// Point is one decoded RPLIDAR measurement sample: angle and range in
	// the units sensor_msgs/LaserScan (and this repo's
	// proto/vtitan/sensor/v1/scan.proto, which mirrors it) use — radians
	// and meters, not the wire format's degrees/mm.
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
	// of data is a full scan, not a single sample.
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
	// "Request Packets' Format").
	reqStartFlag = 0xA5

	// cmdStop, cmdReset, cmdScan, cmdGetHealth are the command bytes this
	// package implements (classic scan mode only — see package doc.go).
	// Values from Figure 4-1 "The Available Requests of RPLIDAR".
	cmdStop      = 0x25
	cmdReset     = 0x40
	cmdScan      = 0x20
	cmdGetHealth = 0x52

	// descStartFlag1/2 are the fixed two bytes identifying the start of a
	// response descriptor (§ "Response Packets' Format", Figure 2-7).
	descStartFlag1 = 0xA5
	descStartFlag2 = 0x5A
	// descLen is the total byte length of a response descriptor: 2 sync
	// bytes + 4 bytes packing {30-bit length, 2-bit send mode} + 1 data
	// type byte.
	descLen = 7
	// descLengthMask isolates the 30-bit Data Response Length field from
	// the combined length/send-mode 32-bit word (bits 31-30 are send
	// mode).
	descLengthMask = 0x3FFFFFFF
	descModeShift  = 30

	// dataTypeMeasurement is the SCAN response descriptor's Data Type byte
	// (Figure 4-4/4-8: response descriptor "A5 5A 05 00 00 40 81" — the
	// trailing 0x81 is this field).
	dataTypeMeasurement = 0x81
	// dataTypeHealth is the GET_HEALTH response descriptor's Data Type
	// byte (§ "Get Device Health Status", response descriptor
	// "A5 5A 3 00 00 00 06").
	dataTypeHealth = 0x06

	// measurementLen is the fixed size in bytes of one SCAN measurement
	// sample (Figure 4-4 "Format of a RPLIDAR Measurement Result Data
	// Response Packet").
	measurementLen = 5
	// healthRespLen is the fixed size in bytes of a GET_HEALTH data
	// response (Figure 4-28).
	healthRespLen = 3

	// angleQ6Scale converts the packed 15-bit angle_q6 fixed-point value
	// to degrees: "Actual heading = angle_q6/64.0 Degree" (Figure 4-5).
	angleQ6Scale = 64.0
	// distanceQ2Scale converts the packed 16-bit distance_q2 fixed-point
	// value to millimeters: "Actual Distance = distance_q2/4.0 mm"
	// (Figure 4-5).
	distanceQ2Scale = 4.0
	// mmPerMeter converts the wire format's millimeter distance unit to
	// the meter unit this package's Point.RangeM and
	// proto/vtitan/sensor/v1/scan.proto both use.
	mmPerMeter = 1000.0
	// degToRad converts the wire format's degree angle unit to the radian
	// unit this package's Point.AngleRad and
	// proto/vtitan/sensor/v1/scan.proto both use.
	degToRad = 3.141592653589793 / 180.0

	// angleHighByteShift is how far byte 2 (angle_q6[14:7]) is shifted
	// left to combine with byte 1's low 7 bits (angle_q6[6:0]).
	angleHighByteShift = 7
	// highByteShift is how far a value's high byte is shifted left to
	// combine with its low byte in the wire format's little-endian 16-bit
	// fields: distance_q2[15:8] (Figure 4-4) and GET_HEALTH's
	// error_code[15:8] (Figure 4-28).
	highByteShift = 8
)

var (
	// ErrShortBuffer means fewer bytes were supplied than the structure
	// being decoded requires.
	ErrShortBuffer = errors.New("lidar: buffer shorter than expected")
	// ErrBadDescriptorSync means a supposed response descriptor didn't
	// start with the fixed 0xA5 0x5A sync bytes.
	ErrBadDescriptorSync = errors.New("lidar: response descriptor sync mismatch")
	// ErrUnexpectedDataType means a response descriptor's data type byte
	// didn't match what the caller requested (e.g. a SCAN request got a
	// non-measurement descriptor back).
	ErrUnexpectedDataType = errors.New("lidar: unexpected response descriptor data type")
	// ErrSyncBitMismatch means a measurement sample's S and ~S bits (byte
	// 0, bits 0-1) weren't complementary — the sample is corrupt.
	ErrSyncBitMismatch = errors.New("lidar: measurement S/~S sync bits not complementary")
	// ErrCheckBitUnset means a measurement sample's C check bit (byte 1,
	// bit 0), which the protocol defines as "constantly set to 1", was 0
	// — the sample is corrupt or misaligned.
	ErrCheckBitUnset = errors.New("lidar: measurement check bit C not set")
	// ErrUnknownHealthStatus means a GET_HEALTH response's status byte was
	// outside the documented 0-2 range.
	ErrUnknownHealthStatus = errors.New("lidar: unknown health status value")
)

// requestPacket builds a no-payload request packet for cmd. All four
// commands this package implements (STOP, RESET, SCAN, GET_HEALTH) carry
// no payload, so a request is always exactly the 2-byte
// {reqStartFlag, cmd} pair — the payload-size/payload/checksum fields
// "Request Packets' Format" describes are only present when a request
// carries a payload, which none of these do (verified against each
// request's own worked example in the protocol doc, e.g. STOP is
// literally "A5 25", nothing more).
func requestPacket(cmd byte) []byte {
	return []byte{reqStartFlag, cmd}
}

// parseDescriptor decodes a 7-byte response descriptor.
func parseDescriptor(b []byte) (descriptor, error) {
	if len(b) < descLen {
		return descriptor{}, fmt.Errorf("%w: descriptor needs %d bytes, got %d", ErrShortBuffer, descLen, len(b))
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

// decodeMeasurement decodes one 5-byte SCAN measurement sample (Figure 4-4
// field layout). It returns the decoded Point, whether S (byte 0 bit 0)
// marks the start of a new 360-degree scan, and an error if the sample
// fails either of the protocol's two self-check bits (S/~S complementary,
// C set).
func decodeMeasurement(b []byte) (pt Point, startOfScan bool, err error) {
	if len(b) < measurementLen {
		return Point{}, false, fmt.Errorf(
			"%w: measurement needs %d bytes, got %d", ErrShortBuffer, measurementLen, len(b),
		)
	}

	// Byte 0: bits [7:2] quality, bit 1 = ~S (inverted start flag), bit 0
	// = S (start flag) — Figure 4-4 draws the byte MSB-first as
	// "Quality | ~S | S" with bit-position markers 8..2, 1, 0, so S is
	// bit 0 and ~S is bit 1. (Careful: this is the opposite pairing of a
	// naive "S is the higher bit" reading of the two adjacent single-bit
	// boxes — verified against the figure's explicit bit-position
	// numbers, not just box order.)
	const (
		sBit         = 0x01
		sInvBit      = 0x02
		qualityShift = 2
	)
	startOfScan = b[0]&sBit != 0
	inverted := b[0]&sInvBit != 0
	if startOfScan == inverted {
		return Point{}, false, fmt.Errorf("%w: byte0=0x%02X", ErrSyncBitMismatch, b[0])
	}
	quality := b[0] >> qualityShift

	// Byte 1: bits [7:1] = angle_q6[6:0], bit 0 = C (check bit, always 1).
	// Byte 2: bits [7:0] = angle_q6[14:7].
	const checkBit = 0x01
	if b[1]&checkBit == 0 {
		return Point{}, false, fmt.Errorf("%w: byte1=0x%02X", ErrCheckBitUnset, b[1])
	}
	angleQ6 := uint16(b[1]>>1) | uint16(b[2])<<angleHighByteShift
	angleDeg := float64(angleQ6) / angleQ6Scale

	// Byte 3: distance_q2[7:0]. Byte 4: distance_q2[15:8].
	distanceQ2 := uint16(b[3]) | uint16(b[4])<<highByteShift
	rangeMM := float64(distanceQ2) / distanceQ2Scale

	return Point{
		AngleRad: angleDeg * degToRad,
		RangeM:   rangeMM / mmPerMeter,
		Quality:  quality,
	}, startOfScan, nil
}

// decodeHealth decodes a 3-byte GET_HEALTH data response (Figure 4-28).
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
