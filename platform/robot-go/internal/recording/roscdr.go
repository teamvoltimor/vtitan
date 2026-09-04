package recording

import (
	"encoding/binary"
	"math"
)

// ROS2 message type names and their .msg definitions, embedded as the MCAP
// schema so a reader that has never seen this repo can still decode the bag.
//
// These are the standard upstream definitions, reproduced verbatim rather
// than referenced: an MCAP is meant to be self-describing, and a bag that
// needs a ROS install on the far side to be read is not.
const (
	// LaserScanType/LaserScanSchema describe sensor_msgs/msg/LaserScan.
	LaserScanType   = "sensor_msgs/msg/LaserScan"
	LaserScanSchema = `std_msgs/Header header
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
`

	// ROS2MessageEncoding/ROS2SchemaEncoding are the MCAP encoding names
	// rosbag2 writes, and what Foxglove Studio keys its native renderers off.
	ROS2MessageEncoding = "cdr"
	ROS2SchemaEncoding  = "ros2msg"
)

// cdrHeaderLE is the four-byte encapsulation header opening every CDR
// message: representation identifier 0x0001 (little-endian CDR) followed by
// a two-byte options field.
var cdrHeaderLE = [4]byte{0x00, 0x01, 0x00, 0x00}

// cdrWriter builds a CDR buffer honoring the alignment rule composed ROS2
// messages obey: each primitive sits at the next offset that is a multiple
// of its own size. Offsets are counted from the start of the buffer,
// INCLUDING the four-byte encapsulation header -- the same convention
// test/bagreplay's cdrReader uses, and equivalent for 4-byte primitives
// either way.
//
// Composed structs (std_msgs/Header below) do NOT re-prepend a header;
// alignment simply continues from where the parent left off.
type cdrWriter struct {
	buf []byte
}

func newCDRWriter() *cdrWriter {
	return &cdrWriter{buf: append([]byte{}, cdrHeaderLE[:]...)}
}

// align pads to the next multiple of size.
func (w *cdrWriter) align(size int) {
	if rem := len(w.buf) % size; rem != 0 {
		w.buf = append(w.buf, make([]byte, size-rem)...)
	}
}

func (w *cdrWriter) writeU32(v uint32) {
	w.align(4)
	w.buf = binary.LittleEndian.AppendUint32(w.buf, v)
}

func (w *cdrWriter) writeI32(v int32) { w.writeU32(uint32(v)) }

func (w *cdrWriter) writeF32(v float32) { w.writeU32(math.Float32bits(v)) }

// writeString writes a CDR string: a uint32 length that INCLUDES the
// terminating NUL, then the bytes, then the NUL.
func (w *cdrWriter) writeString(s string) {
	w.writeU32(uint32(len(s)) + 1)
	w.buf = append(w.buf, s...)
	w.buf = append(w.buf, 0)
}

// writeF32Seq writes a CDR sequence<float32>: a uint32 count, then the
// values.
func (w *cdrWriter) writeF32Seq(values []float32) {
	w.writeU32(uint32(len(values)))
	for _, v := range values {
		w.writeF32(v)
	}
}

// LaserScanCDR is the subset of sensor_msgs/msg/LaserScan a caller supplies;
// the rest of the wire message is filled with the zero values the field
// means (no intensities, no per-ray time increment).
type LaserScanCDR struct {
	FrameID                       string
	StampSec                      int32
	StampNanosec                  uint32
	AngleMin, AngleMax, AngleIncr float32
	TimeIncrement, ScanTime       float32
	RangeMin, RangeMax            float32
	Ranges                        []float32
}

// EncodeLaserScan serializes a LaserScan into ROS2 CDR, the encoding
// rosbag2 records and Foxglove Studio renders natively.
//
// Intensities are written as an empty sequence rather than omitted: the
// field is not optional in the message definition, and a reader that walks
// the struct positionally (which is what CDR requires) would run off the end
// of the buffer without it.
func EncodeLaserScan(scan LaserScanCDR) []byte {
	w := newCDRWriter()
	// std_msgs/Header: builtin_interfaces/Time then string frame_id.
	w.writeI32(scan.StampSec)
	w.writeU32(scan.StampNanosec)
	w.writeString(scan.FrameID)

	w.writeF32(scan.AngleMin)
	w.writeF32(scan.AngleMax)
	w.writeF32(scan.AngleIncr)
	w.writeF32(scan.TimeIncrement)
	w.writeF32(scan.ScanTime)
	w.writeF32(scan.RangeMin)
	w.writeF32(scan.RangeMax)
	w.writeF32Seq(scan.Ranges)
	w.writeF32Seq(nil)
	return w.buf
}
