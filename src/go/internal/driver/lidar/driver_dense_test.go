package lidar

// White-box (package lidar, not lidar_test): readScan's scan-close behavior
// depends on d.reader/d.packetLen, which the black-box tests can't set up.
// These hand-encode dense packet streams (same layout + checksum as
// frame_dense_test.go's vectors) and drive readScan directly.

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"testing"
)

// encodeDensePacketForTest builds one valid Dense Mode response packet
// (Figure 4-13/4-14): start_angle_q6 = startAngleDeg*64, S flag per
// startOfScan, every cabin carrying distMM (nonzero = valid). Checksum is
// XOR over bytes 2..end (Figure 4-14), packed into the two sync bytes' low
// nibbles, exactly as decodeDensePacket expects.
func encodeDensePacketForTest(startAngleDeg float64, startOfScan bool, distMM uint16) []byte {
	raw := make([]byte, denseResponseLen)
	startAngleQ6 := uint16(startAngleDeg * angleQ6Scale)
	raw[2] = byte(startAngleQ6 & 0xFF)
	b3 := byte(startAngleQ6 >> 8)
	if startOfScan {
		b3 |= denseSFlagBit
	}
	raw[3] = b3
	for k := range denseCabinsPerPacket {
		off := denseHeaderLen + k*denseCabinLen
		binary.LittleEndian.PutUint16(raw[off:off+2], distMM)
	}
	var chk byte
	for _, b := range raw[2:] {
		chk ^= b
	}
	raw[0] = denseSync1Nibble<<denseNibbleShift | chk&denseNibbleMask
	raw[1] = denseSync2Nibble<<denseNibbleShift | (chk>>denseNibbleShift)&denseNibbleMask
	return raw
}

// TestDenseSerialDriverReadScan_DiscardsPartialFirstScan reproduces the
// hardware finding of 2026-08-31: the C1 sets S=true only on the stream's
// very first packet, wherever the motor happens to be. When that lands near
// the sweep end (359deg here), a scan closed on the first wrap drop would
// return only the tail of the rotation (~40 points). readScan must discard
// that partial first scan and keep collecting until a full revolution is
// covered.
func TestDenseSerialDriverReadScan_DiscardsPartialFirstScan(t *testing.T) {
	t.Parallel()

	// First packet S=true at 359deg (mid-revolution), then a full sweep to
	// 359deg again, then the wrap packet back to 32deg.
	angles := []float64{359, 32, 68, 104, 140, 176, 212, 248, 284, 320, 359, 32}
	var buf bytes.Buffer
	for i, a := range angles {
		buf.Write(encodeDensePacketForTest(a, i == 0, 1000))
	}

	d := &DenseSerialDriver{
		reader:    bufio.NewReader(&buf),
		packetLen: denseResponseLen,
	}
	scan, err := d.readScan()
	if err != nil {
		t.Fatalf("readScan() error = %v, want nil", err)
	}

	// The partial 359deg packet and the closing wrap packet are excluded;
	// the scan covers the full revolution from 32deg back to 32deg = 10
	// packets x 40 cabins, all valid (distMM=1000).
	if want := 10 * denseCabinsPerPacket; len(scan) != want {
		t.Errorf("readScan() returned %d points, want %d (full revolution, not a partial first scan)", len(scan), want)
	}
}

// TestDenseSerialDriverReadScan_ResyncsAcrossCorruptPacket verifies
// readScan survives a corrupted packet in the middle of the stream: the C1
// streams at 460800 baud, so a single dropped/corrupted byte both fails its
// own packet and (without resync) misaligns every later fixed-size read.
// readDensePacket must scan forward to the next 0xA? 0x5? sync pair and
// keep assembling the scan.
func TestDenseSerialDriverReadScan_ResyncsAcrossCorruptPacket(t *testing.T) {
	t.Parallel()

	angles := []float64{0, 36, 72, 108, 144, 180, 216, 252, 288, 324, 0}

	var buf bytes.Buffer
	for i, a := range angles {
		pkt := encodeDensePacketForTest(a, i == 0, 1000)
		if i == 3 {
			// Corrupt a cabin byte in the 108deg packet.
			pkt[denseHeaderLen+2] ^= 0xFF
		}
		buf.Write(pkt)
	}

	d := &DenseSerialDriver{
		reader:    bufio.NewReader(&buf),
		packetLen: denseResponseLen,
	}
	scan, err := d.readScan()
	if err != nil {
		t.Fatalf("readScan() error = %v, want nil (should resync past the corrupt packet)", err)
	}
	// The corrupt 108deg packet's 40 cabins are dropped (resync skips it),
	// leaving 9 valid packets x 40 cabins.
	if want := 9 * denseCabinsPerPacket; len(scan) != want {
		t.Errorf(
			"readScan() returned %d points, want %d (corrupt packet dropped, rest of revolution kept)",
			len(scan), want,
		)
	}
}

// TestDenseSerialDriverReadScan_FullScanWhenStreamStartsAtZero is the
// control: when the stream's S=true packet lands at the true sweep start
// (0deg), the FIRST scan is already a full revolution and must be returned
// without discarding anything.
func TestDenseSerialDriverReadScan_FullScanWhenStreamStartsAtZero(t *testing.T) {
	t.Parallel()

	// First packet S=true at 0deg, then a full sweep to 324deg, then the
	// wrap packet back to 0deg.
	angles := []float64{0, 36, 72, 108, 144, 180, 216, 252, 288, 324, 0}
	var buf bytes.Buffer
	for i, a := range angles {
		buf.Write(encodeDensePacketForTest(a, i == 0, 1000))
	}

	d := &DenseSerialDriver{
		reader:    bufio.NewReader(&buf),
		packetLen: denseResponseLen,
	}
	scan, err := d.readScan()
	if err != nil {
		t.Fatalf("readScan() error = %v, want nil", err)
	}

	// 10 packets x 40 cabins = the full 0deg..324deg sweep.
	if want := 10 * denseCabinsPerPacket; len(scan) != want {
		t.Errorf("readScan() returned %d points, want %d (full revolution from sweep start)", len(scan), want)
	}
}
