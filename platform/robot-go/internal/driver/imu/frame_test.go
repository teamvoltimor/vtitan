package imu

// White-box (package imu, not imu_test): these tests exercise readFrame and
// parseRVCFrame directly by their unexported names, deliberately — exporting
// them solely so a black-box test could reach them would be worse than
// testing the real internal parsing boundary directly.

import (
	"bufio"
	"bytes"
	"math"
	"testing"
)

// readingTolerance is the float64 comparison tolerance for decoded Reading
// values in this file.
const readingTolerance = 1e-9

// adafruitExampleFrame is the complete example message from Adafruit's
// UART-RVC documentation (learn.adafruit.com/.../uart-rvc-for-arduino):
// 0xAA AA DE 01 00 92 FF 25 08 8D FE EC FF D1 03 00 00 00 E7 — sourced, not
// invented, so a passing test here means the parser matches the vendor's
// own documented example, not just internal self-consistency.
var adafruitExampleFrame = []byte{
	0xAA, 0xAA, 0xDE, 0x01, 0x00, 0x92, 0xFF, 0x25,
	0x08, 0x8D, 0xFE, 0xEC, 0xFF, 0xD1, 0x03, 0x00,
	0x00, 0x00, 0xE7,
}

func TestReadFrame_AdafruitExample(t *testing.T) {
	t.Parallel()

	r := bufio.NewReader(bytes.NewReader(adafruitExampleFrame))
	got, err := readFrame(r)
	if err != nil {
		t.Fatalf("readFrame() error = %v, want nil", err)
	}

	// index=0xDE, yaw raw=0x0001=1 -> 0.01 deg, pitch raw=0xFF92=-110 ->
	// -1.10 deg, roll raw=0x0825=2085 -> 20.85 deg, x_accel raw=0xFE8D=-371
	// -> -371*0.0098067 m/s^2, y_accel raw=0xFFEC=-20, z_accel raw=0x03D1=977
	// — decoded from the same little-endian int16 layout parseRVCFrame
	// implements.
	want := Reading{
		Yaw:    0.01,
		Pitch:  -1.10,
		Roll:   20.85,
		XAccel: -371 * rvcAccelScale,
		YAccel: -20 * rvcAccelScale,
		ZAccel: 977 * rvcAccelScale,
	}

	if !closeReading(got, want, readingTolerance) {
		t.Errorf("readFrame() = %+v, want %+v", got, want)
	}
}

func TestReadFrame_ResyncsOnLeadingGarbage(t *testing.T) {
	t.Parallel()

	garbage := append([]byte{0x00, 0xFF, 0xAA, 0x00}, adafruitExampleFrame...)
	r := bufio.NewReader(bytes.NewReader(garbage))

	if _, err := readFrame(r); err != nil {
		t.Fatalf("readFrame() with leading garbage error = %v, want nil", err)
	}
}

func TestReadFrame_ChecksumMismatch(t *testing.T) {
	t.Parallel()

	corrupt := make([]byte, len(adafruitExampleFrame))
	copy(corrupt, adafruitExampleFrame)
	corrupt[len(corrupt)-1] ^= 0xFF // flip the checksum byte

	r := bufio.NewReader(bytes.NewReader(corrupt))
	if _, err := readFrame(r); err == nil {
		t.Fatal("readFrame() with corrupted checksum: got nil error, want ErrChecksumMismatch")
	}
}

func TestReadFrame_TruncatedStreamDoesNotHang(t *testing.T) {
	t.Parallel()

	truncated := adafruitExampleFrame[:10]
	r := bufio.NewReader(bytes.NewReader(truncated))

	if _, err := readFrame(r); err == nil {
		t.Fatal("readFrame() on truncated stream: got nil error, want an error")
	}
}

// FuzzReadFrame proves the parser fails cleanly (returns an error) rather
// than panicking on arbitrary, malformed, or truncated byte streams — the
// serial port is a trust boundary, real hardware glitches produce exactly
// this kind of garbage.
func FuzzReadFrame(f *testing.F) {
	f.Add(adafruitExampleFrame)
	f.Add([]byte{})
	f.Add([]byte{0xAA})
	f.Add([]byte{0xAA, 0xAA})
	f.Add(bytes.Repeat([]byte{0xAA}, 40))

	f.Fuzz(func(t *testing.T, data []byte) {
		r := bufio.NewReader(bytes.NewReader(data))
		_, _ = readFrame(r) // must not panic; error is fine
	})
}

func closeReading(a, b Reading, tol float64) bool {
	return closeFloat(a.Yaw, b.Yaw, tol) &&
		closeFloat(a.Pitch, b.Pitch, tol) &&
		closeFloat(a.Roll, b.Roll, tol) &&
		closeFloat(a.XAccel, b.XAccel, tol) &&
		closeFloat(a.YAccel, b.YAccel, tol) &&
		closeFloat(a.ZAccel, b.ZAccel, tol)
}

func closeFloat(a, b, tol float64) bool {
	return math.Abs(a-b) <= tol
}
