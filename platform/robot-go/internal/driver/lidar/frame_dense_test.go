package lidar

// White-box (package lidar, not lidar_test), same rationale as
// frame_classic_test.go. TestDenseRequestPacket is a sourced, vendor-
// verified vector (protocol doc p.19's worked example, cross-checked
// against the documented checksum equation, p.7). decodeDensePacket and
// resolveDenseCabins have no equivalent published worked example, so their
// tests hand-encode a chosen angle/distance/checksum per the documented
// field layout and formula (shown in the comment above each vector) and
// assert the decoder round-trips it exactly — same "weaker than a vendor
// vector, but not tautological" caveat as frame_classic_test.go.

import (
	"bytes"
	"encoding/binary"
	"errors"
	"testing"
)

func TestDenseRequestPacket(t *testing.T) {
	t.Parallel()

	// Worked example (protocol doc p.19, "Dense Mode" request): "A5 82 05
	// M 00 00 00 22" with M=0 (legacy mode). Figure 4-11 declares 5
	// distinct payload byte offsets (+0..+4); the doc's inline example
	// appears to elide one trailing reserved zero, but since XOR with
	// 0x00 is a no-op the checksum (0x22) is identical either way -- this
	// asserts the full 9-byte wire form per Figure 4-11's byte-offset
	// table: 0xA5 ^ 0x82 ^ 0x05 ^ 0x00 ^ 0x00 ^ 0x00 ^ 0x00 ^ 0x00 = 0x22.
	want := []byte{0xA5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22}
	if got := denseRequestPacket(); !bytes.Equal(got, want) {
		t.Errorf("denseRequestPacket() = % X, want % X", got, want)
	}
}

// TestDecodeDensePacket_HandComputed hand-encodes start_angle=45.0deg,
// S=1, and 40 cabins each carrying distance (k+1)*10 mm except cabin 5
// (set to 0, the documented invalid-sample value; Figure 4-15), per the
// documented field layout (Figure 4-13/4-14). Work shown:
//
//	start_angle_q6 = 45.0 * 64 = 2880 (0x0B40)
//	  byte2 = 2880 & 0xFF          = 0x40
//	  byte3 = S(1)<<7 | 2880>>8    = 0x80 | 0x0B = 0x8B
//	ChkSum = XOR of every byte from offset 2 onward (Figure 4-14)
//	  byte0 = 0xA0 | ChkSum[3:0], byte1 = 0x50 | ChkSum[7:4]
func TestDecodeDensePacket_HandComputed(t *testing.T) {
	t.Parallel()

	const wantStartAngleDeg = 45.0

	raw := make([]byte, denseResponseLen)
	raw[2] = 0x40
	raw[3] = 0x8B

	wantDistancesMM := make([]float64, denseCabinsPerPacket)
	for k := range wantDistancesMM {
		distMM := uint16((k + 1) * 10) //nolint:gosec // k bounded by denseCabinsPerPacket=40
		if k == 5 {
			distMM = 0
		}
		wantDistancesMM[k] = float64(distMM)
		off := denseHeaderLen + k*denseCabinLen
		binary.LittleEndian.PutUint16(raw[off:off+2], distMM)
	}

	var chk byte
	for _, b := range raw[2:] {
		chk ^= b
	}
	raw[0] = denseSync1Nibble<<denseNibbleShift | chk&denseNibbleMask
	raw[1] = denseSync2Nibble<<denseNibbleShift | (chk>>denseNibbleShift)&denseNibbleMask

	got, err := decodeDensePacket(raw)
	if err != nil {
		t.Fatalf("decodeDensePacket() error = %v, want nil", err)
	}
	if !got.startOfScan {
		t.Error("decodeDensePacket() startOfScan = false, want true")
	}
	if diff := got.startAngleDeg - wantStartAngleDeg; diff > measurementTolerance ||
		diff < -measurementTolerance {
		t.Errorf("startAngleDeg = %v, want %v", got.startAngleDeg, wantStartAngleDeg)
	}
	for k, want := range wantDistancesMM {
		if got.cabinDistancesMM[k] != want {
			t.Errorf("cabinDistancesMM[%d] = %v, want %v", k, got.cabinDistancesMM[k], want)
		}
	}
}

func TestDecodeDensePacket_ShortBuffer(t *testing.T) {
	t.Parallel()

	if _, err := decodeDensePacket(make([]byte, denseHeaderLen-1)); !errors.Is(err, ErrShortBuffer) {
		t.Fatalf("decodeDensePacket() with short buffer: err = %v, want ErrShortBuffer", err)
	}
}

func TestDecodeDensePacket_BadSync(t *testing.T) {
	t.Parallel()

	raw := make([]byte, denseResponseLen)
	raw[0] = 0x00 // wrong upper nibble; should be denseSync1Nibble (0xA)
	raw[1] = denseSync2Nibble << denseNibbleShift
	if _, err := decodeDensePacket(raw); !errors.Is(err, ErrDenseSyncMismatch) {
		t.Fatalf("decodeDensePacket() with bad sync: err = %v, want ErrDenseSyncMismatch", err)
	}
}

func TestDecodeDensePacket_BadChecksum(t *testing.T) {
	t.Parallel()

	// All-zero payload -> real checksum is 0x00; flip a low bit in byte0
	// so the encoded ChkSum[3:0] no longer matches.
	raw := make([]byte, denseResponseLen)
	raw[0] = denseSync1Nibble<<denseNibbleShift | 0x01
	raw[1] = denseSync2Nibble << denseNibbleShift
	if _, err := decodeDensePacket(raw); !errors.Is(err, ErrDenseChecksumMismatch) {
		t.Fatalf("decodeDensePacket() with bad checksum: err = %v, want ErrDenseChecksumMismatch", err)
	}
}

// TestResolveDenseCabins checks the angle-interpolation formula (Figure
// 4-17), including the wraparound branch of AngleDiff, and that
// zero-distance (invalid) cabins are skipped.
func TestResolveDenseCabins(t *testing.T) {
	t.Parallel()

	var prev densePacket
	prev.startAngleDeg = 350.0
	prev.cabinDistancesMM = make([]float64, denseCabinsPerPacket)
	prev.cabinDistancesMM[0] = 1000 // k=0 -> angle == prev.startAngleDeg
	prev.cabinDistancesMM[20] = 2000
	// every other cabin (incl. index 1) is left at its zero value ->
	// invalid, must be skipped.

	// nextStartAngleDeg=10.0 wraps past 360: AngleDiff(350, 10) since
	// 350 > 10 -> 360 + 10 - 350 = 20.
	got := resolveDenseCabins(prev, 10.0)

	if len(got) != 2 {
		t.Fatalf("resolveDenseCabins() returned %d points, want 2 (only cabins 0 and 20 valid)", len(got))
	}

	// k=0: angle = 350 + (20/40)*0 = 350 deg.
	wantAngle0Rad := 350.0 * degToRad
	if diff := got[0].AngleRad - wantAngle0Rad; diff > measurementTolerance ||
		diff < -measurementTolerance {
		t.Errorf("point 0 AngleRad = %v, want %v", got[0].AngleRad, wantAngle0Rad)
	}
	if got[0].RangeM != 1.0 {
		t.Errorf("point 0 RangeM = %v, want 1.0", got[0].RangeM)
	}

	// k=20: angle = 350 + (20/40)*20 = 360 deg -> normalized to 0.
	if diff := got[1].AngleRad - 0.0; diff > measurementTolerance || diff < -measurementTolerance {
		t.Errorf("point 1 (cabin 20) AngleRad = %v, want 0", got[1].AngleRad)
	}
	if got[1].RangeM != 2.0 {
		t.Errorf("point 1 (cabin 20) RangeM = %v, want 2.0", got[1].RangeM)
	}
}
