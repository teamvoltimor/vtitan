package lidar

// White-box (package lidar, not lidar_test): these tests exercise the
// unexported wire-format functions directly, deliberately — exporting them
// solely so a black-box test could reach them would be worse than testing
// the real internal parsing boundary directly (same rationale as
// internal/driver/imu/frame_test.go).
//
// Test-vector provenance, by strength:
//
//   - requestPacket and the descriptor test vectors below are copied
//     byte-for-byte from Slamtec's own worked examples in the protocol PDF
//     (STOP "A5 25", RESET "A5 40", SCAN request "A5 20" / response
//     descriptor "A5 5A 05 00 00 40 81", GET_HEALTH request "A5 52" /
//     response descriptor "A5 5A 3 00 00 00 06") — these are sourced,
//     vendor-verified vectors, the strongest coverage in this file.
//   - decodeMeasurement and decodeHealth have no equivalent sourced
//     byte-for-byte worked example in the protocol doc (it defines the
//     field layout and scale factors but never publishes a concrete
//     sample's raw bytes the way it does for descriptors). Their tests
//     instead hand-encode a chosen angle/distance/status per the
//     documented formula (shown in the comment above each vector) and
//     assert the decoder round-trips it exactly. This is weaker than a
//     vendor-sourced vector — it can't catch an error present in both the
//     encoding comment's arithmetic and the decoder simultaneously — but
//     it is not a tautological "assert against whatever the code
//     produces" test: the expected values are computed independently, by
//     hand, from the documented bit layout and scale factors before the
//     decoder ever runs.

import (
	"bytes"
	"testing"
)

// measurementTolerance is the float64 comparison tolerance for decoded
// angle/range values in this file.
const measurementTolerance = 1e-9

func TestRequestPacket(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name string
		cmd  byte
		want []byte
	}{
		{name: "stop", cmd: cmdStop, want: []byte{0xA5, 0x25}},
		{name: "reset", cmd: cmdReset, want: []byte{0xA5, 0x40}},
		{name: "scan", cmd: cmdScan, want: []byte{0xA5, 0x20}},
		{name: "get_health", cmd: cmdGetHealth, want: []byte{0xA5, 0x52}},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			got := requestPacket(tt.cmd)
			if !bytes.Equal(got, tt.want) {
				t.Errorf("requestPacket(0x%02X) = % X, want % X", tt.cmd, got, tt.want)
			}
		})
	}
}

func TestParseDescriptor(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name string
		raw  []byte
		want descriptor
	}{
		// Sourced from "Start Scan (SCAN) Request and Response", Figure
		// 4-8: response descriptor "A5 5A 05 00 00 40 81". Packed 32-bit
		// word (bytes 2-5, little-endian) = 0x40000005; low 30 bits
		// (length) = 5, top 2 bits (send mode) = 1 (Multiple).
		{
			name: "scan",
			raw:  []byte{0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81},
			want: descriptor{length: 5, sendMode: 1, dataType: dataTypeMeasurement},
		},
		// Sourced from "Get Device Health Status (GET_HEALTH)": response
		// descriptor "A5 5A 3 00 00 00 06". Packed word = 0x00000003;
		// length = 3, send mode = 0 (Single).
		{
			name: "get_health",
			raw:  []byte{0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06},
			want: descriptor{length: 3, sendMode: 0, dataType: dataTypeHealth},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			got, err := parseDescriptor(tt.raw)
			if err != nil {
				t.Fatalf("parseDescriptor() error = %v, want nil", err)
			}
			if got != tt.want {
				t.Errorf("parseDescriptor() = %+v, want %+v", got, tt.want)
			}
		})
	}
}

func TestParseDescriptor_BadSync(t *testing.T) {
	t.Parallel()

	raw := []byte{0xA5, 0x5B, 0x05, 0x00, 0x00, 0x40, 0x81} // second sync byte corrupted
	if _, err := parseDescriptor(raw); err == nil {
		t.Fatal("parseDescriptor() with bad sync: got nil error, want ErrBadDescriptorSync")
	}
}

func TestParseDescriptor_ShortBuffer(t *testing.T) {
	t.Parallel()

	if _, err := parseDescriptor([]byte{0xA5, 0x5A, 0x05}); err == nil {
		t.Fatal("parseDescriptor() with short buffer: got nil error, want ErrShortBuffer")
	}
}

// TestDecodeMeasurement_HandComputed hand-encodes angle=45.5deg,
// distance=1234.75mm, quality=10, S=1 per the documented field layout
// (Figure 4-4/4-5) and checks decodeMeasurement recovers the exact same
// values. Work shown:
//
//	angle_q6 = 45.5 * 64 = 2912 (0xB60, 12 significant bits)
//	  low7  = 2912 & 0x7F = 96  (0x60)
//	  high8 = 2912 >> 7   = 22  (0x16)
//	  byte1 = (low7 << 1) | C(1) = (96<<1)|1 = 193 (0xC1)
//	  byte2 = high8 = 22 (0x16)
//	distance_q2 = 1234.75 * 4 = 4939 (0x134B)
//	  byte3 = 4939 & 0xFF = 75  (0x4B)
//	  byte4 = 4939 >> 8   = 19  (0x13)
//	byte0 = (quality=10 << 2) | (~S=0 << 1) | (S=1) = 0x29
func TestDecodeMeasurement_HandComputed(t *testing.T) {
	t.Parallel()

	raw := []byte{0x29, 0xC1, 0x16, 0x4B, 0x13}
	got, startOfScan, err := decodeMeasurement(raw)
	if err != nil {
		t.Fatalf("decodeMeasurement() error = %v, want nil", err)
	}
	if !startOfScan {
		t.Error("decodeMeasurement() startOfScan = false, want true")
	}

	const (
		wantAngleDeg = 45.5
		wantRangeMM  = 1234.75
		wantQuality  = 10
	)
	wantAngleRad := wantAngleDeg * degToRad
	wantRangeM := wantRangeMM / mmPerMeter

	if diff := got.AngleRad - wantAngleRad; diff > measurementTolerance || diff < -measurementTolerance {
		t.Errorf("AngleRad = %v, want %v", got.AngleRad, wantAngleRad)
	}
	if diff := got.RangeM - wantRangeM; diff > measurementTolerance || diff < -measurementTolerance {
		t.Errorf("RangeM = %v, want %v", got.RangeM, wantRangeM)
	}
	if got.Quality != wantQuality {
		t.Errorf("Quality = %d, want %d", got.Quality, wantQuality)
	}
}

// TestDecodeMeasurement_NotStartOfScan is the same hand-computed vector as
// above but with S=0, ~S=1 (byte0 = (10<<2)|(1<<1)|0 = 0x2A) — confirms the
// start-of-scan flag round-trips both ways, not just the S=1 case.
func TestDecodeMeasurement_NotStartOfScan(t *testing.T) {
	t.Parallel()

	raw := []byte{0x2A, 0xC1, 0x16, 0x4B, 0x13}
	_, startOfScan, err := decodeMeasurement(raw)
	if err != nil {
		t.Fatalf("decodeMeasurement() error = %v, want nil", err)
	}
	if startOfScan {
		t.Error("decodeMeasurement() startOfScan = true, want false")
	}
}

func TestDecodeMeasurement_SyncBitMismatch(t *testing.T) {
	t.Parallel()

	// Both S and ~S set to 1 (0x03) — must differ per the protocol.
	raw := []byte{0x03, 0xC1, 0x16, 0x4B, 0x13}
	if _, _, err := decodeMeasurement(raw); err == nil {
		t.Fatal("decodeMeasurement() with S==~S: got nil error, want ErrSyncBitMismatch")
	}
}

func TestDecodeMeasurement_CheckBitUnset(t *testing.T) {
	t.Parallel()

	// byte1's low bit (C) cleared: 0xC1 -> 0xC0.
	raw := []byte{0x29, 0xC0, 0x16, 0x4B, 0x13}
	if _, _, err := decodeMeasurement(raw); err == nil {
		t.Fatal("decodeMeasurement() with C=0: got nil error, want ErrCheckBitUnset")
	}
}

func TestDecodeMeasurement_ShortBuffer(t *testing.T) {
	t.Parallel()

	if _, _, err := decodeMeasurement([]byte{0x29, 0xC1}); err == nil {
		t.Fatal("decodeMeasurement() with short buffer: got nil error, want ErrShortBuffer")
	}
}

func TestDecodeHealth(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name string
		raw  []byte
		want Health
	}{
		{name: "good", raw: []byte{0x00, 0x00, 0x00}, want: Health{Status: HealthGood, ErrorCode: 0}},
		{name: "warning", raw: []byte{0x01, 0x00, 0x00}, want: Health{Status: HealthWarning, ErrorCode: 0}},
		// error_code = 0x1234 = 4660, little-endian bytes 0x34 0x12.
		{name: "error_with_code", raw: []byte{0x02, 0x34, 0x12}, want: Health{Status: HealthError, ErrorCode: 4660}},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			got, err := decodeHealth(tt.raw)
			if err != nil {
				t.Fatalf("decodeHealth() error = %v, want nil", err)
			}
			if got != tt.want {
				t.Errorf("decodeHealth() = %+v, want %+v", got, tt.want)
			}
		})
	}
}

func TestDecodeHealth_UnknownStatus(t *testing.T) {
	t.Parallel()

	if _, err := decodeHealth([]byte{0x03, 0x00, 0x00}); err == nil {
		t.Fatal("decodeHealth() with status=3: got nil error, want ErrUnknownHealthStatus")
	}
}

func TestDecodeHealth_ShortBuffer(t *testing.T) {
	t.Parallel()

	if _, err := decodeHealth([]byte{0x00, 0x00}); err == nil {
		t.Fatal("decodeHealth() with short buffer: got nil error, want ErrShortBuffer")
	}
}

func TestHealthStatus_String(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name   string
		status HealthStatus
		want   string
	}{
		{name: "good", status: HealthGood, want: "good"},
		{name: "warning", status: HealthWarning, want: "warning"},
		{name: "error", status: HealthError, want: "error"},
		{name: "unknown", status: HealthStatus(99), want: "unknown"},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			if got := tt.status.String(); got != tt.want {
				t.Errorf("String() = %q, want %q", got, tt.want)
			}
		})
	}
}

// FuzzParseDescriptor proves the descriptor parser fails cleanly rather
// than panicking on arbitrary, malformed, or truncated byte streams — the
// serial port is a trust boundary, real hardware glitches and stream
// desync produce exactly this kind of garbage.
func FuzzParseDescriptor(f *testing.F) {
	f.Add([]byte{0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81})
	f.Add([]byte{0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06})
	f.Add([]byte{})
	f.Add([]byte{0xA5})
	f.Add([]byte{0xA5, 0x5A})

	f.Fuzz(func(t *testing.T, data []byte) {
		_, _ = parseDescriptor(data) // must not panic; error is fine
	})
}

// FuzzDecodeMeasurement proves the measurement decoder fails cleanly on
// arbitrary/malformed/truncated data rather than panicking.
func FuzzDecodeMeasurement(f *testing.F) {
	f.Add([]byte{0x29, 0xC1, 0x16, 0x4B, 0x13})
	f.Add([]byte{})
	f.Add([]byte{0x00})
	f.Add([]byte{0x00, 0x00, 0x00, 0x00, 0x00})
	f.Add([]byte{0xFF, 0xFF, 0xFF, 0xFF, 0xFF})

	f.Fuzz(func(t *testing.T, data []byte) {
		_, _, _ = decodeMeasurement(data) // must not panic; error is fine
	})
}

// FuzzDecodeHealth proves the health decoder fails cleanly on
// arbitrary/malformed/truncated data rather than panicking.
func FuzzDecodeHealth(f *testing.F) {
	f.Add([]byte{0x00, 0x00, 0x00})
	f.Add([]byte{0x02, 0x34, 0x12})
	f.Add([]byte{})
	f.Add([]byte{0xFF, 0xFF, 0xFF})

	f.Fuzz(func(t *testing.T, data []byte) {
		_, _ = decodeHealth(data) // must not panic; error is fine
	})
}
