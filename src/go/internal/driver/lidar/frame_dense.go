package lidar

// Byte-level framing for the RPLIDAR Express Scan "Dense Mode" protocol —
// the legacy/default Express capsule format ("Express Scan (EXPRESS_SCAN)
// Request and Response" / "Dense Mode", protocol doc rev 2.8, pp. 18-23).
// Zero I/O, same split as frame_classic.go.
//
// This is the mode this robot's Python stack actually validated on
// hardware: sllidar_ros2's default scan_mode="Standard" always routes
// through drv->startScanExpress (see sllidar_node.cpp's work_loop — the
// scan_mode.empty() branch, the only one that calls the classic
// drv->startScan, is never taken when scan_mode is set), never the
// classic 5-byte SCAN packets frame_classic.go decodes. Hardware
// validation 2026-08-31 found classic mode's range decode reading 2-4x
// too large versus known distances on this exact C1 unit, while the
// Python/Dense path reads correctly — so this is the mode to prefer.
//
// Scope: only the "legacy" Dense capsule request (working_mode=0, Figure
// 4-11's note: "When set to 0, this command is a legacy version express
// scan request") is implemented, not GET_LIDAR_CONF-selected named scan
// modes ("Standard", "Boost", ...) or the newer Ultra Capsuled format —
// GET_LIDAR_CONF is a separate, unimplemented request/response pair, and
// the legacy Dense request is the simplest mode that exercises the same
// capsuled wire format Python's "Standard" mode resolves to on this
// hardware family.
//
// Unlike classic mode, a Dense response packet carries only its *first*
// sample's angle (start_angle_q6); the other 39 samples' angles are
// linearly interpolated between this packet's start angle and the
// following packet's (Figure 4-17), so decoding one packet's samples
// requires the next packet to already be read. See DenseSerialDriver's
// readScan (driver_dense.go) for how that one-packet lookback is carried
// across Read calls, mirroring ClassicSerialDriver's first-Point
// carryover in frame_classic.go/driver_classic.go.

import (
	"encoding/binary"
	"errors"
	"fmt"
	"math"
)

// densePacket is one decoded Dense Mode response packet (Figure 4-13),
// before its samples' individual angles are known — resolving them
// requires the *next* packet's start_angle_q6 (see resolveDenseCabins).
type densePacket struct {
	startAngleDeg float64
	startOfScan   bool
	// cabinDistancesMM holds each of the packet's samples' raw distance in
	// millimeters (Figure 4-15: plain mm, no Q2 scaling, unlike classic
	// mode's distance_q2). 0 means the sample is invalid. Its length is
	// derived from the decoded packet's actual size (see
	// decodeDensePacket), not hardcoded to denseCabinsPerPacket — the
	// response descriptor's declared Data Response Length (Figure 2-7) is
	// the authoritative packet size for a given session, and this package
	// doesn't assume the legacy request always gets exactly the documented
	// 84-byte/40-cabin worked example back on every device revision.
	cabinDistancesMM []float64
}

const (
	// cmdExpressScan is the Express Scan (EXPRESS_SCAN) request command
	// byte (Figure 4-10 "Dense Mode" request example: "A5 82 05 M 00 00
	// 00 22").
	cmdExpressScan = 0x82
	// denseWorkingModeLegacy is the working_mode payload byte value that
	// requests the legacy Dense capsule format (Figure 4-12: "When set to
	// 0, this command is a legacy version express scan request").
	denseWorkingModeLegacy = 0x00
	// denseRequestPayloadLen is the Express Scan request's fixed 5-byte
	// payload: working_mode + 4 reserved-zero bytes (Figure 4-11, byte
	// offsets +0 through +4).
	denseRequestPayloadLen = 5

	// dataTypeDenseMeasurement is the Dense Mode response descriptor's
	// Data Type byte (p.19/21 worked example: "A5 5A 54 00 00 40 85" —
	// the trailing 0x85 is this field).
	dataTypeDenseMeasurement = 0x85
	// denseResponseLen is the *documented* size in bytes of one Dense Mode
	// response packet (Figure 4-13: 4-byte header + 40 cabins x 2 bytes =
	// 84 bytes) per the worked example. It is only a fallback/test
	// default -- the real, authoritative packet size for a given session
	// is the response descriptor's declared Data Response Length (Figure
	// 2-7), which DenseSerialDriver.Connect reads and uses instead (see
	// driver_dense.go), in case a given device's actual legacy-mode
	// packet size differs from this worked example.
	denseResponseLen = 84
	// denseHeaderLen is the number of header bytes (sync + checksum +
	// start_angle_q6 + S) preceding the first cabin in a Dense Mode
	// response packet (Figure 4-13, byte offsets +0 through +3). This one
	// is not a fallback -- the header layout itself doesn't vary with
	// packet size.
	denseHeaderLen = 4
	// denseCabinsPerPacket is the *documented* number of 2-byte cabins
	// (samples) per Dense Mode response packet (Figure 4-14: "A data
	// response packet contains 40 groups of cabin data"), used only by
	// this package's tests -- decodeDensePacket derives the real cabin
	// count from the actual packet length instead of assuming this.
	denseCabinsPerPacket = 40
	// denseCabinLen is the fixed size in bytes of one cabin (Figure 4-13:
	// "distance[15:0]").
	denseCabinLen = 2

	// denseSync1Nibble and denseSync2Nibble are the fixed upper nibbles
	// of a Dense Mode response packet's first two bytes, identifying the
	// start of a new packet (Figure 4-14: sync1 always 0xA, sync2 always
	// 0x5 — distinct from the response *descriptor*'s full-byte
	// descStartFlag1/2 sync pattern).
	denseSync1Nibble = 0xA
	denseSync2Nibble = 0x5
	// denseNibbleMask isolates a byte's low nibble.
	denseNibbleMask = 0x0F
	// denseNibbleShift is how far a high nibble is shifted to combine
	// with a low nibble into one byte.
	denseNibbleShift = 4

	// denseSFlagBit is byte 3's bit 7 (S, start-of-scan flag; Figure
	// 4-13's "8 7 0 | S | start_angle_q6[14:8]" layout).
	denseSFlagBit = 0x80
	// denseStartAngleHighMask isolates byte 3's bits [6:0]
	// (start_angle_q6[14:8]) after the S flag bit.
	denseStartAngleHighMask = 0x7F
	// denseStartAngleHighShift is how far byte 3's start_angle_q6[14:8]
	// bits are shifted left to combine with byte 2's start_angle_q6[7:0].
	denseStartAngleHighShift = 8
)

// ErrDenseSyncMismatch means a Dense Mode response packet's first two
// bytes' upper nibbles weren't the fixed 0xA/0x5 sync pattern (Figure
// 4-14) — the packet is corrupt or the stream is misaligned.
var ErrDenseSyncMismatch = errors.New("lidar: dense packet sync nibble mismatch")

// ErrDenseChecksumMismatch means a Dense Mode response packet's XOR
// checksum (Figure 4-14) didn't match its computed value — the packet is
// corrupt.
var ErrDenseChecksumMismatch = errors.New("lidar: dense packet checksum mismatch")

// denseRequestPacket builds the legacy Dense Mode Express Scan request:
// {reqStartFlag, cmdExpressScan, payloadSize, payload..., checksum}
// (Figure 2-4's general payload-carrying request format, applied to
// Figure 4-11's 5-byte Express Scan payload). The checksum follows the
// documented equation exactly (p.7, using XOR for the equation's circled-
// plus operator): "checksum = 0 XOR 0xA5 XOR CmdType XOR PayloadSize XOR
// Payload[0] XOR ... XOR Payload[n]" — verified against the worked
// example ("A5 82 05 M 00 00 00 22" with M=0 legacy mode: 0xA5 ^ 0x82 ^
// 0x05 ^ 0x00 == 0x22).
func denseRequestPacket() []byte {
	payload := [denseRequestPayloadLen]byte{denseWorkingModeLegacy, 0, 0, 0, 0}

	checksum := byte(reqStartFlag) ^ byte(cmdExpressScan) ^ byte(denseRequestPayloadLen)
	for _, p := range payload {
		checksum ^= p
	}

	pkt := make([]byte, 0, 3+denseRequestPayloadLen+1)
	pkt = append(pkt, reqStartFlag, cmdExpressScan, denseRequestPayloadLen)
	pkt = append(pkt, payload[:]...)
	pkt = append(pkt, checksum)
	return pkt
}

// decodeDensePacket decodes one Dense Mode response packet (Figure 4-13
// field layout), validating its sync nibbles and XOR checksum. The number
// of cabins (samples) is derived from len(b) rather than assumed to be
// denseCabinsPerPacket — callers should size b to the response
// descriptor's declared Data Response Length (see DenseSerialDriver.Connect
// in driver_dense.go), not hardcode the documented worked example's 84
// bytes, in case this device's actual legacy-mode packet size differs.
func decodeDensePacket(b []byte) (densePacket, error) {
	if len(b) < denseHeaderLen {
		return densePacket{}, fmt.Errorf(
			"%w: dense packet needs at least %d header bytes, got %d",
			ErrShortBuffer, denseHeaderLen, len(b),
		)
	}
	if b[0]>>denseNibbleShift != denseSync1Nibble || b[1]>>denseNibbleShift != denseSync2Nibble {
		return densePacket{}, fmt.Errorf(
			"%w: got 0x%02X 0x%02X", ErrDenseSyncMismatch, b[0], b[1],
		)
	}

	// ChkSum is split across the two sync bytes' low nibbles (Figure
	// 4-13): byte0 low nibble = ChkSum[3:0], byte1 low nibble =
	// ChkSum[7:4]. It's computed over every byte from start_angle_q6[7:0]
	// (byte 2) onward, i.e. the whole packet except the two sync/checksum
	// bytes themselves (Figure 4-14: "starting from start_angle_q6[7:0]").
	wantChecksum := b[0]&denseNibbleMask | (b[1]&denseNibbleMask)<<denseNibbleShift
	var gotChecksum byte
	for _, x := range b[2:] {
		gotChecksum ^= x
	}
	if gotChecksum != wantChecksum {
		return densePacket{}, fmt.Errorf(
			"%w: got 0x%02X, want 0x%02X", ErrDenseChecksumMismatch, gotChecksum, wantChecksum,
		)
	}

	startOfScan := b[3]&denseSFlagBit != 0
	startAngleQ6 := uint16(b[2]) | uint16(b[3]&denseStartAngleHighMask)<<denseStartAngleHighShift

	cabinCount := (len(b) - denseHeaderLen) / denseCabinLen
	pkt := densePacket{
		startAngleDeg:    float64(startAngleQ6) / angleQ6Scale,
		startOfScan:      startOfScan,
		cabinDistancesMM: make([]float64, cabinCount),
	}
	for k := range pkt.cabinDistancesMM {
		off := denseHeaderLen + k*denseCabinLen
		pkt.cabinDistancesMM[k] = float64(binary.LittleEndian.Uint16(b[off : off+2]))
	}
	return pkt, nil
}

// resolveDenseCabins converts prev's raw cabin distances into Points,
// interpolating each sample's angle between prev's own start angle and
// nextStartAngleDeg (the *following* packet's start angle) per the
// documented formula (Figure 4-17, generalized from the documented 40 to
// prev's actual cabin count -- see decodeDensePacket):
//
//	θ_k = ω_i + AngleDiff(ω_i, ω_i+1)/N * k    (N = len(prev.cabinDistancesMM))
//	AngleDiff(ω_i, ω_i+1) = ω_i+1 - ω_i          if ω_i <= ω_i+1
//	                      = 360 + ω_i+1 - ω_i    otherwise
//
// Samples with a zero raw distance are invalid (Figure 4-15) and skipped,
// mirroring classic mode's implicit RangeM==0 no-return convention.
func resolveDenseCabins(prev densePacket, nextStartAngleDeg float64) []Point {
	angleDiff := nextStartAngleDeg - prev.startAngleDeg
	if prev.startAngleDeg > nextStartAngleDeg {
		angleDiff += 360
	}
	step := angleDiff / float64(len(prev.cabinDistancesMM))

	pts := make([]Point, 0, len(prev.cabinDistancesMM))
	for k, distMM := range prev.cabinDistancesMM {
		if distMM == 0 {
			continue
		}

		angleDeg := correctAngleDeg(prev.startAngleDeg+step*float64(k), mountInverted, yawOffsetDeg)
		angleDeg = math.Mod(angleDeg, 360)
		if angleDeg < 0 {
			angleDeg += 360
		}

		pts = append(pts, Point{
			AngleRad: angleDeg * degToRad,
			RangeM:   distMM / mmPerMeter,
		})
	}
	return pts
}
