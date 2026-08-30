package imu

import (
	"bufio"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
)

// Reading is one decoded BNO08x UART-RVC heading sample. Yaw/Pitch/Roll are
// in degrees, XAccel/YAccel/ZAccel in m/s^2 — the same units and scaling as
// Adafruit's reference implementation (adafruit_bno08x_rvc.py), which this
// parser was verified against byte-for-byte using its own documented
// example frame (see frame_test.go).
type Reading struct {
	Yaw, Pitch, Roll       float64
	XAccel, YAccel, ZAccel float64
}

const (
	rvcSyncByte = 0xAA
	// rvcFrameBodyLen is the byte count following the two 0xAA sync bytes:
	// 1-byte index, 6 little-endian int16 fields, 3 reserved bytes, 1
	// checksum byte (1 + 12 + 3 + 1 = 17).
	rvcFrameBodyLen = 17
	// rvcChecksumLen is how many leading bytes of the body the checksum
	// covers — everything except the checksum byte itself.
	rvcChecksumLen = 16

	rvcYawOffset    = 1
	rvcPitchOffset  = 3
	rvcRollOffset   = 5
	rvcXAccelOffset = 7
	rvcYAccelOffset = 9
	rvcZAccelOffset = 11

	rvcDegreeScale = 0.01
	rvcAccelScale  = 0.0098067 // m/s^2 per LSB, matches Adafruit's constant exactly
)

var (
	// ErrChecksumMismatch means a frame's checksum byte didn't match the
	// sum of the preceding bytes — the frame is corrupt and must be
	// discarded, not trusted.
	ErrChecksumMismatch = errors.New("imu: RVC frame checksum mismatch")
	// ErrShortFrame means fewer bytes were available than a full RVC frame
	// requires (e.g. the serial stream ended mid-frame).
	ErrShortFrame = errors.New("imu: RVC frame shorter than expected")
)

// readFrame scans r for the next valid RVC frame, resyncing on the 0xAA 0xAA
// sync bytes if the stream is misaligned. Unlike the reference Adafruit
// implementation (which blindly reads 2 bytes at a time and can fail to
// resync if the stream is ever off by one byte), this scans byte-by-byte
// for the sync pattern — a deliberate robustness improvement, not a
// behavioral drift in the decoded values themselves.
func readFrame(r *bufio.Reader) (Reading, error) {
	var prev byte
	for {
		b, err := r.ReadByte()
		if err != nil {
			return Reading{}, fmt.Errorf("imu: reading sync bytes: %w", err)
		}
		if prev == rvcSyncByte && b == rvcSyncByte {
			break
		}
		prev = b
	}

	body := make([]byte, rvcFrameBodyLen)
	if _, err := io.ReadFull(r, body); err != nil {
		return Reading{}, fmt.Errorf("imu: reading frame body: %w", err)
	}
	return parseRVCFrame(body)
}

// parseRVCFrame decodes the 17 bytes following an RVC frame's two 0xAA sync
// bytes.
func parseRVCFrame(body []byte) (Reading, error) {
	if len(body) != rvcFrameBodyLen {
		return Reading{}, fmt.Errorf(
			"%w: got %d bytes, want %d",
			ErrShortFrame,
			len(body),
			rvcFrameBodyLen,
		)
	}

	var checksum byte
	for _, b := range body[:rvcChecksumLen] {
		checksum += b
	}
	if got := body[rvcChecksumLen]; checksum != got {
		return Reading{}, fmt.Errorf(
			"%w: got 0x%02X, want 0x%02X",
			ErrChecksumMismatch,
			got,
			checksum,
		)
	}

	return Reading{
		Yaw:    float64(rvcInt16(body, rvcYawOffset)) * rvcDegreeScale,
		Pitch:  float64(rvcInt16(body, rvcPitchOffset)) * rvcDegreeScale,
		Roll:   float64(rvcInt16(body, rvcRollOffset)) * rvcDegreeScale,
		XAccel: float64(rvcInt16(body, rvcXAccelOffset)) * rvcAccelScale,
		YAccel: float64(rvcInt16(body, rvcYAccelOffset)) * rvcAccelScale,
		ZAccel: float64(rvcInt16(body, rvcZAccelOffset)) * rvcAccelScale,
	}, nil
}

func rvcInt16(body []byte, offset int) int16 {
	return int16(binary.LittleEndian.Uint16(body[offset : offset+2]))
}
