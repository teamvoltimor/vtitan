package recording

import (
	"bytes"
	"encoding/binary"
	"errors"
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

// aviChunk is one "00dc" frame chunk buffered until the AVI index sizes are
// known.
type aviChunk struct {
	data []byte
}

const (
	// aviStreamHeaderBytes is the fixed size of an AVI strh stream-header chunk.
	aviStreamHeaderBytes = 56
	// aviStreamFormatBytes is the size of the strf chunk we write: a minimal
	// BITMAPINFOHEADER.
	aviStreamFormatBytes = 40
	// aviWordAlign is the DWORD alignment AVI chunks pad their payloads to.
	aviWordAlign = 4
	// aviChunkHeaderBytes is a "00dc" chunk's tag plus size field.
	aviChunkHeaderBytes = 8
	// strhRCFrameBytes is the strh rcFrame RECT (four int16).
	strhRCFrameBytes = 8
	// avihReservedBytes is avih's reserved[4] array, four DWORDs.
	avihReservedBytes = 16
	// aviFrameRate is the nominal AVI frame rate recorded in avih and strh.
	aviFrameRate = 30
	// aviMicroSecPerFrame is avih's per-frame period at aviFrameRate.
	aviMicroSecPerFrame = 1_000_000 / aviFrameRate
	// aviBitsPerPixel is the BITMAPINFOHEADER bit depth for the RGB frames.
	aviBitsPerPixel = 24
	// aviRGBBytesPerPixel is the unpacked RGB stride multiplier.
	aviRGBBytesPerPixel = 3
	// avihHasIndex flags the AVI file as carrying an idx1 index.
	avihHasIndex = 0x00000010
	// aviifKeyframe flags an idx1 entry as a key frame.
	aviifKeyframe = 0x00000010
	// videoJPEGQuality is the JPEG quality the portable backend encodes at.
	videoJPEGQuality = 80
	// sizeFieldBytes is the DWORD a chunk or list size field occupies.
	sizeFieldBytes = 4
	// fourCCLen is the byte length of a four-character code tag.
	fourCCLen = 4
)

// newMJPEGAVISink opens path for MJPEG-in-AVI writing. The first frame fixes the
// dimensions.
func newMJPEGAVISink(path string) (*mjpegAVISink, error) {
	f, err := os.Create(path)
	if err != nil {
		return nil, fmt.Errorf("recording: creating video %s: %w", path, err)
	}
	return &mjpegAVISink{f: f}, nil
}

// Write encodes f as JPEG and buffers it. The AVI header is written on Close
// once the frame count and size are known (AVI requires the index up front).
func (s *mjpegAVISink) Write(f *Frame, _ HudOverlay) error {
	if s.written {
		return errors.New("recording: write after close")
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
	if err = jpeg.Encode(&buf, img, &jpeg.Options{Quality: videoJPEGQuality}); err != nil {
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
	// Pre-compute chunk sizes.
	chunks := make([]aviChunk, len(s.frames))
	totalMovi := fourCCLen // "movi" fourcc
	for i, fr := range s.frames {
		// Each frame chunk: "00dc" tag (4) + size (4) + padded data.
		size := len(fr)
		pad := (aviWordAlign - size%aviWordAlign) % aviWordAlign
		c := make([]byte, aviChunkHeaderBytes+size+pad)
		copy(c[0:], "00dc")
		binary.LittleEndian.PutUint32(c[fourCCLen:], uint32(size))
		copy(c[aviChunkHeaderBytes:], fr)
		chunks[i] = aviChunk{data: c}
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
	writeU32LE(&b, aviStreamHeaderBytes)
	// microSecPerFrame, maxBytesPerSec=0, padding=0, flags=has-index,
	// totalFrames, initialFrames=0, streams=1, suggestedBufferSize=0,
	// width, height, reserved[4]=0.
	writeU32LE(&b, aviMicroSecPerFrame)
	writeU32LE(&b, 0)
	writeU32LE(&b, 0)
	writeU32LE(&b, avihHasIndex)
	writeU32LE(&b, uint32(len(s.frames)))
	writeU32LE(&b, 0)
	writeU32LE(&b, 1) // streams
	writeU32LE(&b, 0)
	writeU32LE(&b, uint32(s.width))
	writeU32LE(&b, uint32(s.height))
	b.Write(make([]byte, avihReservedBytes)) // reserved

	// strl list.
	writeFourCC(&b, "strl")
	// strl contents size = (chunk header + strh) + (chunk header + strf)
	writeU32LE(&b, (aviChunkHeaderBytes+aviStreamHeaderBytes)+(aviChunkHeaderBytes+aviStreamFormatBytes))

	// strh chunk.
	writeFourCC(&b, "strh")
	writeU32LE(&b, aviStreamHeaderBytes)
	writeFourCC(&b, "vids")
	writeFourCC(&b, "MJPG")
	writeU32LE(&b, 0)                       // flags
	writeU32LE(&b, 0)                       // priority + language
	writeU32LE(&b, 0)                       // initialFrames
	writeU32LE(&b, 1)                       // scale (1)
	writeU32LE(&b, aviFrameRate)            // rate
	writeU32LE(&b, 0)                       // start
	writeU32LE(&b, uint32(len(s.frames)))   // length
	writeU32LE(&b, 0)                       // suggestedBufferSize
	writeU32LE(&b, 0)                       // quality
	writeU32LE(&b, 0)                       // sampleSize
	b.Write(make([]byte, strhRCFrameBytes)) // rcFrame RECT
	// strh is 56 bytes total: the fixed fields above plus the rcFrame RECT.

	// strf chunk (BITMAPINFOHEADER, minimal).
	writeFourCC(&b, "strf")
	writeU32LE(&b, aviStreamFormatBytes)
	writeU32LE(&b, aviStreamFormatBytes) // biSize
	writeU32LE(&b, uint32(s.width))
	writeU32LE(&b, uint32(s.height))
	writeU16LE(&b, 1)                                            // planes
	writeU16LE(&b, aviBitsPerPixel)                              // bitCount
	writeFourCC(&b, "MJPG")                                      // biCompression
	writeU32LE(&b, uint32(s.width*s.height*aviRGBBytesPerPixel)) // biSizeImage
	writeU32LE(&b, 0)                                            // xPelsPerMeter
	writeU32LE(&b, 0)                                            // yPelsPerMeter
	writeU32LE(&b, 0)                                            // biClrUsed
	writeU32LE(&b, 0)                                            // biClrImportant

	// Patch hdrl list size.
	hdrlTotal := b.Len() - hdrlListSizePos - sizeFieldBytes
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
	binary.LittleEndian.PutUint32(b.Bytes()[moviListSizePos:], uint32(moviTotal+fourCCLen))

	// idx1.
	writeFourCC(&b, "idx1")
	idxSizePos := b.Len()
	b.Write([]byte{0, 0, 0, 0})
	idxStart := b.Len()
	offset := fourCCLen // offset to first chunk relative to movi list payload
	for _, c := range chunks {
		writeFourCC(&b, "00dc")
		writeU32LE(&b, aviifKeyframe)                           // AVIIF_KEYFRAME
		writeU32LE(&b, uint32(offset))                          // offset from movi payload start
		writeU32LE(&b, uint32(len(c.data)-aviChunkHeaderBytes)) // chunk data size
		offset += len(c.data)
	}
	idxTotal := b.Len() - idxStart
	binary.LittleEndian.PutUint32(b.Bytes()[idxSizePos:], uint32(idxTotal))

	// Patch RIFF size.
	riffTotal := b.Len() - riffSizePos - sizeFieldBytes
	binary.LittleEndian.PutUint32(b.Bytes()[riffSizePos:], uint32(riffTotal))

	if _, err := s.f.Write(b.Bytes()); err != nil {
		return fmt.Errorf("recording: writing AVI: %w", err)
	}
	return nil
}

// writeU32LE appends v as a little-endian uint32. A bytes.Buffer write cannot
// fail, so binary.Write's error is intentionally not surfaced.
func writeU32LE(b *bytes.Buffer, v uint32) {
	var buf [4]byte
	binary.LittleEndian.PutUint32(buf[:], v)
	b.Write(buf[:])
}

// writeU16LE appends v as a little-endian uint16. See writeU32LE.
func writeU16LE(b *bytes.Buffer, v uint16) {
	var buf [2]byte
	binary.LittleEndian.PutUint16(buf[:], v)
	b.Write(buf[:])
}

func writeFourCC(b *bytes.Buffer, s string) {
	b.WriteString(s)
}
