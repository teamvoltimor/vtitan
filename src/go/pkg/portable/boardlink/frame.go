package boardlink

import (
	"encoding/binary"
	"errors"
)

// Decoder turns a byte stream into packets. The zero value is ready to use.
// It holds one frame's worth of fixed buffer and allocates nothing.
//
// Not safe for concurrent use.
type Decoder struct {
	buf      [MaxEncodedLen]byte
	n        int
	overflow bool
}

// Version is the protocol version every frame carries. Bump it on any
// incompatible change to framing or to an existing message's body; adding a
// new Type does not need a bump.
const Version uint8 = 1

// Frame layout: version:u8 | type:u8 | seq:u16 | body | crc16:u16. The
// header offsets are stated once here; decodeFrame reads through them, and
// Append writes the fields in the same order.
const (
	versionLen = 1
	typeLen    = 1
	seqLen     = 2

	versionOffset = 0
	typeOffset    = versionOffset + versionLen
	seqOffset     = typeOffset + typeLen
	headerLen     = seqOffset + seqLen

	crcLen = 2
	// maxRawLen is the largest frame before COBS.
	maxRawLen = headerLen + maxBodyLen + crcLen
	// MaxEncodedLen is the largest frame on the wire, delimiter included:
	// COBS adds at most one byte per 254 plus one, and the frame is far
	// shorter than 254.
	MaxEncodedLen = maxRawLen + 2 + 1
)

// ReadChunkSize is the buffer size a link reader asks for. A frame is well
// under it, so one read normally holds a whole frame; the per-byte decoder
// handles one split across reads either way.
const ReadChunkSize = 64

// CRC-16/CCITT-FALSE parameters, and COBS's longest run marker.
const (
	crcPoly     = 0x1021
	crcInit     = uint16(0xFFFF)
	bitsPerByte = 8
	cobsMaxCode = 0xFF
)

// Errors a decoder reports. Each drops exactly one frame; the decoder is
// ready for the next one.
var (
	ErrCRC         = errors.New("boardlink: CRC mismatch")
	ErrVersion     = errors.New("boardlink: unsupported protocol version")
	ErrUnknownType = errors.New("boardlink: unknown message type")
	ErrLength      = errors.New("boardlink: body length does not match its type")
	ErrOverflow    = errors.New("boardlink: frame longer than any valid frame")
	ErrCOBS        = errors.New("boardlink: malformed COBS encoding")
)

// Append encodes p as one delimited frame appended to dst. p.Seq and p.Type
// are taken as given; the caller owns sequence numbering.
func Append(dst []byte, p *Packet) ([]byte, error) {
	if _, ok := bodyLen(p.Type); !ok {
		return dst, ErrUnknownType
	}
	var raw [maxRawLen]byte
	w := writer{b: raw[:0]}
	w.u8(Version)
	w.u8(uint8(p.Type))
	w.u16(p.Seq)
	appendBody(&w, p)
	w.u16(crc16(w.b))
	dst = appendCOBS(dst, w.b)
	return append(dst, 0), nil
}

// Feed consumes one byte. When b completes a frame, Feed decodes it into p
// and returns true with a nil error, or false with the reason the frame was
// dropped. For every other byte it returns false, nil.
//
// Zero-length frames (consecutive delimiters) are ignored, which lets a
// sender prefix a delimiter to flush a receiver that joined mid-frame.
func (d *Decoder) Feed(b byte, p *Packet) (bool, error) {
	if b != 0 {
		if d.n == len(d.buf) {
			d.overflow = true
			return false, nil
		}
		d.buf[d.n] = b
		d.n++
		return false, nil
	}

	n, overflow := d.n, d.overflow
	d.n, d.overflow = 0, false
	if overflow {
		return false, ErrOverflow
	}
	if n == 0 {
		return false, nil
	}
	return decodeFrame(d.buf[:n], p)
}

func decodeFrame(encoded []byte, p *Packet) (bool, error) {
	var raw [MaxEncodedLen]byte
	n, ok := decodeCOBS(raw[:], encoded)
	if !ok {
		return false, ErrCOBS
	}
	frame := raw[:n]
	if len(frame) < headerLen+crcLen {
		return false, ErrLength
	}
	body, sum := frame[:len(frame)-crcLen], binary.LittleEndian.Uint16(frame[len(frame)-crcLen:])
	if crc16(body) != sum {
		return false, ErrCRC
	}
	if body[versionOffset] != Version {
		return false, ErrVersion
	}
	t := Type(body[typeOffset])
	want, known := bodyLen(t)
	if !known {
		return false, ErrUnknownType
	}
	if len(body)-headerLen != want {
		return false, ErrLength
	}

	p.Type = t
	p.Seq = binary.LittleEndian.Uint16(body[seqOffset:])
	readBody(&reader{b: body[headerLen:]}, p)
	return true, nil
}

// crc16 is CRC-16/CCITT-FALSE: polynomial 0x1021, initial 0xFFFF, no
// reflection, no final XOR. Bitwise rather than table-driven: frames are
// under 64 bytes, and a 512-byte table costs more flash than it saves time.
func crc16(data []byte) uint16 {
	crc := crcInit
	for _, b := range data {
		crc ^= uint16(b) << bitsPerByte
		for range bitsPerByte {
			if crc&0x8000 != 0 {
				crc = crc<<1 ^ crcPoly
			} else {
				crc <<= 1
			}
		}
	}
	return crc
}

// appendCOBS appends the COBS encoding of src (without a delimiter).
func appendCOBS(dst, src []byte) []byte {
	codeIdx := len(dst)
	dst = append(dst, 0)
	code := byte(1)
	for _, b := range src {
		if b == 0 {
			dst[codeIdx] = code
			codeIdx = len(dst)
			dst = append(dst, 0)
			code = 1
			continue
		}
		dst = append(dst, b)
		code++
		if code == cobsMaxCode {
			dst[codeIdx] = code
			codeIdx = len(dst)
			dst = append(dst, 0)
			code = 1
		}
	}
	dst[codeIdx] = code
	return dst
}

// decodeCOBS decodes src (no delimiter) into dst, reporting the length and
// whether src was well-formed.
func decodeCOBS(dst, src []byte) (int, bool) {
	n := 0
	for i := 0; i < len(src); {
		code := int(src[i])
		if code == 0 || i+code > len(src) {
			return 0, false
		}
		i++
		for j := 1; j < code; j++ {
			if n == len(dst) {
				return 0, false
			}
			dst[n] = src[i]
			n++
			i++
		}
		if code < cobsMaxCode && i < len(src) {
			if n == len(dst) {
				return 0, false
			}
			dst[n] = 0
			n++
		}
	}
	return n, true
}
