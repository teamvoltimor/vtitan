package recording

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"image/jpeg"
	"os"
)

// mjpegAVISink is the pure-Go video backend: it muxes JPEG frames into a minimal
// AVI 1.0 container (MJPEG). No C dependencies, so it builds under
// CGO_ENABLED=0 for bench/sim. The gocv backend (video_cgo.go) produces true
// mp4v/H.264 on hardware; this is the portable fallback.
type mjpegAVISink struct {
	f       *os.File
	frames  [][]byte
	width   int
	height  int
	written bool
}

// newMJPEGAVISink opens path for MJPEG-in-AVI writing. The first frame fixes the
// dimensions.
func newMJPEGAVISink(path string) (*mjpegAVISink, error) {
	f, err := os.Create(path)
	if err != nil {
		return nil, fmt.Errorf("recording: creating video %s: %w", path, err)
	}
	return &mjpegAVISink{f: f}, nil
}

// VideoSink adapter: mjpegAVISink implements VideoSink, so this is identity.
func (s *mjpegAVISink) asSink() VideoSink { return s }

// Write encodes f as JPEG and buffers it. The AVI header is written on Close
// once the frame count and size are known (AVI requires the index up front).
func (s *mjpegAVISink) Write(f *Frame, _ HudOverlay) error {
	if s.written {
		return fmt.Errorf("recording: write after close")
	}
	img, err := f.image()
	if err != nil {
		return err
	}
	if s.width == 0 {
		b := img.Bounds()
		s.width, s.height = b.Dx(), b.Dy()
	}
	var buf bytes.Buffer
	if err = jpeg.Encode(&buf, img, &jpeg.Options{Quality: 80}); err != nil {
		return fmt.Errorf("recording: encoding video frame: %w", err)
	}
	s.frames = append(s.frames, buf.Bytes())
	return nil
}

// Close writes the AVI header + movi list + index, then closes the file.
func (s *mjpegAVISink) Close() error {
	if s.written {
		return nil
	}
	s.written = true
	defer s.f.Close()

	if len(s.frames) == 0 {
		// Nothing recorded; leave an empty file rather than a broken AVI.
		return nil
	}
	return s.writeAVI()
}

// writeAVI emits a minimal but valid AVI 1.0 MJPEG file.
func (s *mjpegAVISink) writeAVI() error {
	const (
		streamHeaderBytes = 56
		streamFormatBytes = 40 // BITMAPINFOHEADER (we write a minimal one)
	)
	// Pre-compute chunk sizes.
	type chunk struct{ data []byte }
	chunks := make([]chunk, len(s.frames))
	totalMovi := 4 // "movi" fourcc
	for i, fr := range s.frames {
		// Each frame chunk: "00dc" tag (4) + size (4) + padded data.
		size := len(fr)
		pad := (4 - size%4) % 4
		c := make([]byte, 8+size+pad)
		copy(c[0:], "00dc")
		binary.LittleEndian.PutUint32(c[4:], uint32(size))
		copy(c[8:], fr)
		chunks[i] = chunk{data: c}
		totalMovi += len(c)
	}

	// We build: RIFF AVI / LIST hdrl (avih + strl(strh+strf)) / LIST movi / idx1.
	var b bytes.Buffer

	// RIFF header.
	writeFourCC(&b, "RIFF")
	// RIFF size filled later.
	riffSizePos := b.Len()
	b.Write([]byte{0, 0, 0, 0})
	writeFourCC(&b, "AVI ")

	// LIST hdrl
	writeFourCC(&b, "LIST")
	hdrlListSizePos := b.Len()
	b.Write([]byte{0, 0, 0, 0})
	writeFourCC(&b, "hdrl")

	// avih chunk.
	writeFourCC(&b, "avih")
	binary.Write(&b, binary.LittleEndian, uint32(56))
	// microSecPerFrame (assume 30fps), maxBytesPerSec=0, padding=0,
	// flags=0x10 (has index), totalFrames, initialFrames=0, streams=1,
	// suggestedBufferSize=0, width, height, reserved[4]=0.
	binary.Write(&b, binary.LittleEndian, uint32(1_000_000/30))
	binary.Write(&b, binary.LittleEndian, uint32(0))
	binary.Write(&b, binary.LittleEndian, uint32(0))
	binary.Write(&b, binary.LittleEndian, uint32(0x00000010)) // AVIF_HASINDEX
	binary.Write(&b, binary.LittleEndian, uint32(len(s.frames)))
	binary.Write(&b, binary.LittleEndian, uint32(0))
	binary.Write(&b, binary.LittleEndian, uint32(1)) // streams
	binary.Write(&b, binary.LittleEndian, uint32(0))
	binary.Write(&b, binary.LittleEndian, uint32(s.width))
	binary.Write(&b, binary.LittleEndian, uint32(s.height))
	b.Write(make([]byte, 16)) // reserved

	// strl list.
	writeFourCC(&b, "strl")
	// strl contents size = (8+56) + (8+40)
	binary.Write(&b, binary.LittleEndian, uint32((8+streamHeaderBytes)+(8+streamFormatBytes)))

	// strh chunk.
	writeFourCC(&b, "strh")
	binary.Write(&b, binary.LittleEndian, uint32(streamHeaderBytes))
	writeFourCC(&b, "vids")
	writeFourCC(&b, "MJPG")
	binary.Write(&b, binary.LittleEndian, uint32(0))             // flags
	binary.Write(&b, binary.LittleEndian, uint32(0))             // priority + language
	binary.Write(&b, binary.LittleEndian, uint32(0))             // initialFrames
	binary.Write(&b, binary.LittleEndian, uint32(1))             // scale (1)
	binary.Write(&b, binary.LittleEndian, uint32(30))            // rate (30)
	binary.Write(&b, binary.LittleEndian, uint32(0))             // start
	binary.Write(&b, binary.LittleEndian, uint32(len(s.frames))) // length
	binary.Write(&b, binary.LittleEndian, uint32(0))             // suggestedBufferSize
	binary.Write(&b, binary.LittleEndian, uint32(0))             // quality
	binary.Write(&b, binary.LittleEndian, uint32(0))             // sampleSize
	b.Write(make([]byte, 8))                                     // rcFrame (16 bytes? actually 8 used here; remainder below)
	// strh is 56 bytes total; we've written 4+4+ (8+4+4+4+4+4+4+4+4+4+4+4 +8) = ensure 56.
	// The rcFrame is RECT (4 int16 = 8 bytes); we already wrote 8 zeros above,
	// so strh is complete at 56.

	// strf chunk (BITMAPINFOHEADER, minimal).
	writeFourCC(&b, "strf")
	binary.Write(&b, binary.LittleEndian, uint32(streamFormatBytes))
	binary.Write(&b, binary.LittleEndian, uint32(streamFormatBytes)) // biSize
	binary.Write(&b, binary.LittleEndian, uint32(s.width))
	binary.Write(&b, binary.LittleEndian, uint32(s.height))
	binary.Write(&b, binary.LittleEndian, uint16(1))                  // planes
	binary.Write(&b, binary.LittleEndian, uint16(24))                 // bitCount
	writeFourCC(&b, "MJPG")                                           // biCompression
	binary.Write(&b, binary.LittleEndian, uint32(s.width*s.height*3)) // biSizeImage
	binary.Write(&b, binary.LittleEndian, uint32(0))                  // xPelsPerMeter
	binary.Write(&b, binary.LittleEndian, uint32(0))                  // yPelsPerMeter
	binary.Write(&b, binary.LittleEndian, uint32(0))                  // biClrUsed
	binary.Write(&b, binary.LittleEndian, uint32(0))                  // biClrImportant

	// Patch hdrl list size.
	hdrlTotal := b.Len() - hdrlListSizePos - 4
	binary.LittleEndian.PutUint32(b.Bytes()[hdrlListSizePos:], uint32(hdrlTotal))

	// LIST movi.
	writeFourCC(&b, "LIST")
	moviListSizePos := b.Len()
	b.Write([]byte{0, 0, 0, 0})
	writeFourCC(&b, "movi")
	moviStart := b.Len()
	for _, c := range chunks {
		b.Write(c.data)
	}
	moviTotal := b.Len() - moviStart
	binary.LittleEndian.PutUint32(b.Bytes()[moviListSizePos:], uint32(moviTotal+4))

	// idx1.
	writeFourCC(&b, "idx1")
	idxSizePos := b.Len()
	b.Write([]byte{0, 0, 0, 0})
	idxStart := b.Len()
	offset := 4 // offset to first chunk relative to movi list payload (after "movi" tag)
	for _, c := range chunks {
		writeFourCC(&b, "00dc")
		binary.Write(&b, binary.LittleEndian, uint32(0x00000010))    // AVIIF_KEYFRAME
		binary.Write(&b, binary.LittleEndian, uint32(offset))        // offset from movi payload start
		binary.Write(&b, binary.LittleEndian, uint32(len(c.data)-8)) // chunk data size
		offset += len(c.data)
	}
	idxTotal := b.Len() - idxStart
	binary.LittleEndian.PutUint32(b.Bytes()[idxSizePos:], uint32(idxTotal))

	// Patch RIFF size.
	riffTotal := b.Len() - riffSizePos - 4
	binary.LittleEndian.PutUint32(b.Bytes()[riffSizePos:], uint32(riffTotal))

	if _, err := s.f.Write(b.Bytes()); err != nil {
		return fmt.Errorf("recording: writing AVI: %w", err)
	}
	return nil
}

func writeFourCC(b *bytes.Buffer, s string) {
	b.WriteString(s)
}
