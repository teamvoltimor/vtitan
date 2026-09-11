package bagreplay_test

import (
	"encoding/binary"
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/test/bagreplay"
)

// encodeLaserScan builds a sensor_msgs/msg/LaserScan exactly as rmw emits it:
// the four-byte encapsulation header, a Humble+ std_msgs/Header (time stamp +
// string frame_id, no legacy seq field), the seven float32 sweep parameters,
// then the ranges[] sequence and an (unconsumed here) intensities[] sequence.
func encodeLaserScan(order binary.ByteOrder, headerFrameID string,
	angleMin, angleMax, angleInc float32,
	ranges []float32, intensities []float32) []byte {

	out := []byte{0x00, 0x01, 0x00, 0x00} // little-endian CDR header

	putF32 := func(v float32) {
		b := make([]byte, 4)
		order.PutUint32(b, math.Float32bits(v))
		out = append(out, b...)
	}

	// Header: time stamp (sec, nanosec) then string frame_id.
	out = append(out, 0, 0, 0, 0) // sec
	out = append(out, 0, 0, 0, 0) // nanosec
	// frame_id string: uint32 length INCLUDING NUL, then bytes + NUL.
	fid := append([]byte(headerFrameID), 0x00)
	lenBuf := make([]byte, 4)
	order.PutUint32(lenBuf, uint32(len(fid)))
	out = append(out, lenBuf...)
	out = append(out, fid...)
	// CDR aligns the next member to 4; real rmw emits padding bytes to get
	// there, so replicate that gap in the buffer.
	for len(out)%4 != 0 {
		out = append(out, 0x00)
	}

	// Sweep parameters.
	putF32(angleMin)
	putF32(angleMax)
	putF32(angleInc)
	putF32(0.0) // time_increment
	putF32(0.0) // scan_time
	putF32(0.0) // range_min
	putF32(0.0) // range_max

	// ranges[] sequence.
	rc := make([]byte, 4)
	order.PutUint32(rc, uint32(len(ranges)))
	out = append(out, rc...)
	for _, r := range ranges {
		putF32(r)
	}

	// intensities[] sequence (present on the wire; the decoder skips it).
	ic := make([]byte, 4)
	order.PutUint32(ic, uint32(len(intensities)))
	out = append(out, ic...)
	for _, v := range intensities {
		putF32(v)
	}
	return out
}

func TestDecodeLaserScan(t *testing.T) {
	t.Parallel()

	ranges := []float32{0.1, 0.2, 0.3, 0.4}
	data := encodeLaserScan(binary.LittleEndian, "laser", -3.14159, 3.14159, 0.0087, ranges, []float32{1, 2})
	got, err := bagreplay.DecodeLaserScan(data)
	if err != nil {
		t.Fatalf("DecodeLaserScan() error = %v, want nil", err)
	}
	if got.AngleMin != -3.14159 {
		t.Fatalf("AngleMin = %v, want -3.14159", got.AngleMin)
	}
	if got.AngleMax != 3.14159 {
		t.Fatalf("AngleMax = %v, want 3.14159", got.AngleMax)
	}
	if got.AngleIncrement != 0.0087 {
		t.Fatalf("AngleIncrement = %v, want 0.0087", got.AngleIncrement)
	}
	if len(got.RangesM) != len(ranges) {
		t.Fatalf("len(RangesM) = %d, want %d", len(got.RangesM), len(ranges))
	}
	for i, r := range ranges {
		if got.RangesM[i] != r {
			t.Fatalf("RangesM[%d] = %v, want %v", i, got.RangesM[i], r)
		}
	}
}

// TestDecodeLaserScan_RejectsTruncated covers the malformed cases: a buffer
// that ends inside the sweep parameters or ranges[] must fail rather than
// silently produce a short, wrong scan -- a bad /scan decode would corrupt
// every downstream parity comparison.
func TestDecodeLaserScan_RejectsTruncated(t *testing.T) {
	t.Parallel()

	good := encodeLaserScan(binary.LittleEndian, "laser", -3.14, 3.14, 0.01,
		[]float32{1, 2, 3}, []float32{0, 0, 0})

	// Absolute offsets into the valid buffer above (little-endian, "laser"
	// frame_id pads to a 4-byte boundary at offset 24):
	//   0   encapsulation header
	//   4   header sec
	//   8   header nanosec
	//   12  frame_id length (4 bytes)
	//   16  frame_id "laser\0" (6 bytes) + 2 pad -> ends at 24
	//   24  angle_min..range_max (7 float32 = 28 bytes) -> ends at 52
	//   52  ranges length (4 bytes) -> ends at 56
	//   56  ranges[0..2] (12 bytes) -> ends at 68
	tests := []struct {
		name  string
		trunc int
	}{
		{"header only", 4},
		{"inside frame_id length", 14},
		{"before ranges count", 52},
		{"inside ranges payload", 64},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()
			if tt.trunc > len(good) {
				t.Skip("truncation longer than buffer")
			}
			if _, err := bagreplay.DecodeLaserScan(good[:tt.trunc]); err == nil {
				t.Fatalf("DecodeLaserScan() error = nil on %d-byte truncated buffer, want error", tt.trunc)
			}
		})
	}
}
