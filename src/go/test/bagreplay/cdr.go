package bagreplay

import (
	"encoding/binary"
	"errors"
	"fmt"
)

// cdrHeaderLen is the encapsulation header every ROS2 CDR message opens
// with: one reserved byte, one representation identifier, then two option
// bytes. Payload offsets are counted from the end of it.
const cdrHeaderLen = 4

// cdrLengthLen is the width of the uint32 element count CDR prefixes a
// sequence or string with.
const cdrLengthLen = 4

// CDR representation identifiers. ROS2's rmw emits little-endian on every
// platform this project runs on, but the big-endian encoding is legal and
// costs one branch to honor rather than silently mis-decode.
const (
	cdrBigEndian    = 0x00
	cdrLittleEndian = 0x01
)

// ErrShortMessage is returned when a CDR buffer ends before the field being
// decoded does -- a truncated or misidentified message, never a valid one.
var ErrShortMessage = errors.New("bagreplay: CDR message truncated")

// ErrBadEncapsulation is returned when the four-byte CDR encapsulation
// header is not one of the two representations ROS2 emits.
var ErrBadEncapsulation = errors.New("bagreplay: unrecognized CDR encapsulation")

// byteOrderOf returns the byte order the message was encoded with, reading
// the representation identifier rather than assuming the host's.
func byteOrderOf(data []byte) (binary.ByteOrder, error) {
	if len(data) < cdrHeaderLen {
		return nil, fmt.Errorf("%w: %d bytes, need at least %d for the encapsulation header",
			ErrShortMessage, len(data), cdrHeaderLen)
	}
	switch data[1] {
	case cdrLittleEndian:
		return binary.LittleEndian, nil
	case cdrBigEndian:
		return binary.BigEndian, nil
	default:
		return nil, fmt.Errorf("%w: representation identifier 0x%02x", ErrBadEncapsulation, data[1])
	}
}

// DecodeStdMsgsString decodes a std_msgs/msg/String, the envelope
// /nav_debug's JSON arrives in.
//
// CDR encodes a string as a uint32 length INCLUDING its terminating NUL,
// followed by that many bytes. The NUL is stripped here so callers get the
// payload they expect; a length of zero is rejected because even the empty
// string carries its terminator, so zero means the buffer is malformed
// rather than empty.
func DecodeStdMsgsString(data []byte) (string, error) {
	order, err := byteOrderOf(data)
	if err != nil {
		return "", err
	}

	const lengthEnd = cdrHeaderLen + cdrLengthLen
	if len(data) < lengthEnd {
		return "", fmt.Errorf("%w: %d bytes, need %d for the string length",
			ErrShortMessage, len(data), lengthEnd)
	}
	length := order.Uint32(data[cdrHeaderLen:lengthEnd])
	if length == 0 {
		return "", fmt.Errorf("%w: string length 0, which cannot include a NUL terminator", ErrShortMessage)
	}

	end := uint64(lengthEnd) + uint64(length)
	if uint64(len(data)) < end {
		return "", fmt.Errorf("%w: string declares %d bytes, only %d remain",
			ErrShortMessage, length, len(data)-lengthEnd)
	}
	// length counts the NUL; the payload is everything before it.
	return string(data[lengthEnd : end-1]), nil
}
