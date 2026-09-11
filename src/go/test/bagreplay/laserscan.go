package bagreplay

import (
	"encoding/binary"
	"fmt"
	"math"
)

// LaserScan is a decoded sensor_msgs/msg/LaserScan, the physical lidar's raw
// sweep. Only the fields the Go navigator actually consumes are carried:
// angle_min/max/increment and the ranges[] array. time_increment, scan_time,
// range_min/max and intensities[] are part of the wire type but the navigator
// reads neither here nor in the Python baseline it is being checked against,
// so decoding them would be dead fixture-reading code.
//
// This is deliberately NOT a general ROS2 CDR type: it exists only to replay
// old Python/ROS2 bags through the Go navigator for a one-time parity gate.
// The Go port's own runtime scan type is controllers.LidarScan (protobuf),
// which the reader below feeds.
type LaserScan struct {
	AngleMin       float32
	AngleMax       float32
	AngleIncrement float32
	RangesM        []float32
}

// cdrReader walks a CDR buffer honoring the alignment rules the primitives
// in a composed ROS2 message obey: each field is placed at the next offset
// that is a multiple of its size, after the four-byte encapsulation header
// that every message opens with. Composed structs (the Header below) do NOT
// re-prepend a header; alignment simply continues from where the parent left
// off.
type cdrReader struct {
	data  []byte
	order binary.ByteOrder
	pos   int
}

// ScanTopic is the topic the physical LIDAR publishes its sweep on, matching
// bag_io.Topics.SCAN.
const ScanTopic = "/scan"

// cdrAlign32 is the CDR wire size and alignment shared by the 32-bit
// primitives this reader decodes (uint32 and float32): each is placed at the
// next offset that is a multiple of four.
const cdrAlign32 = 4

func newCDRReader(data []byte) (*cdrReader, error) {
	order, err := byteOrderOf(data)
	if err != nil {
		return nil, err
	}
	return &cdrReader{data: data, order: order, pos: cdrHeaderLen}, nil
}

// align advances pos to the next multiple of size, matching CDR's
// "next multiple of the primitive's alignment" rule for size 1/2/4/8.
func (r *cdrReader) align(size int) error {
	if r.pos%size != 0 {
		r.pos += size - (r.pos % size)
	}
	if r.pos < 0 {
		return fmt.Errorf("%w: alignment overflow", ErrShortMessage)
	}
	return nil
}

func (r *cdrReader) readU32() (uint32, error) {
	if err := r.align(cdrAlign32); err != nil {
		return 0, err
	}
	if r.pos+cdrAlign32 > len(r.data) {
		return 0, fmt.Errorf("%w: need uint32 at offset %d, only %d bytes", ErrShortMessage, r.pos, len(r.data))
	}
	v := r.order.Uint32(r.data[r.pos : r.pos+cdrAlign32])
	r.pos += cdrAlign32
	return v, nil
}

func (r *cdrReader) readF32() (float32, error) {
	if err := r.align(cdrAlign32); err != nil {
		return 0, err
	}
	if r.pos+cdrAlign32 > len(r.data) {
		return 0, fmt.Errorf("%w: need float32 at offset %d, only %d bytes", ErrShortMessage, r.pos, len(r.data))
	}
	v := r.order.Uint32(r.data[r.pos : r.pos+cdrAlign32])
	r.pos += cdrAlign32
	return math.Float32frombits(v), nil
}

// skipHeader consumes the std_msgs/Header that opens every LaserScan: a time
// (two int32: sec, nanosec, matching ROS2 Humble+ where the legacy uint32 seq
// field was dropped) and a string frame_id. None of it feeds the navigator --
// the pose comes from /nav_debug and the scan only supplies ranges -- so it is
// read purely to position the cursor at the angles that follow.
func (r *cdrReader) skipHeader() error {
	// time stamp: two int32, each alignment 4.
	if _, err := r.readU32(); err != nil { // sec
		return err
	}
	if _, err := r.readU32(); err != nil { // nanosec
		return err
	}
	// string frame_id: uint32 length INCLUDING its NUL, then that many bytes.
	length, err := r.readU32()
	if err != nil {
		return err
	}
	if length == 0 {
		return fmt.Errorf("%w: header frame_id length 0, cannot include NUL", ErrShortMessage)
	}
	end := uint64(r.pos) + uint64(length)
	if end > uint64(len(r.data)) {
		return fmt.Errorf("%w: frame_id declares %d bytes, only %d remain", ErrShortMessage, length, len(r.data)-r.pos)
	}
	r.pos = int(end)
	return nil
}

// readF32Seq reads a CDR sequence<float32>: a uint32 count, then that many
// float32s at alignment 4.
func (r *cdrReader) readF32Seq() ([]float32, error) {
	count, err := r.readU32()
	if err != nil {
		return nil, err
	}
	if err = r.align(cdrAlign32); err != nil {
		return nil, err
	}
	need := int(count) * cdrAlign32
	if r.pos+need > len(r.data) {
		return nil, fmt.Errorf("%w: float32 sequence of %d needs %d bytes at offset %d, only %d remain",
			ErrShortMessage, count, need, r.pos, len(r.data)-r.pos)
	}
	out := make([]float32, count)
	for i := range out {
		out[i], err = r.readF32()
		if err != nil {
			return nil, err
		}
	}
	return out, nil
}

// DecodeLaserScan decodes a sensor_msgs/msg/LaserScan, matching
// bag_io's /scan decode. Only the angles and ranges[] the navigator reads are
// extracted; the rest of the wire message is skipped (see LaserScan).
func DecodeLaserScan(data []byte) (LaserScan, error) {
	r, err := newCDRReader(data)
	if err != nil {
		return LaserScan{}, err
	}
	if err = r.skipHeader(); err != nil {
		return LaserScan{}, fmt.Errorf("bagreplay: decoding LaserScan header: %w", err)
	}
	scan := LaserScan{}
	if scan.AngleMin, err = r.readF32(); err != nil {
		return scan, fmt.Errorf("bagreplay: decoding LaserScan.angle_min: %w", err)
	}
	if scan.AngleMax, err = r.readF32(); err != nil {
		return scan, fmt.Errorf("bagreplay: decoding LaserScan.angle_max: %w", err)
	}
	if scan.AngleIncrement, err = r.readF32(); err != nil {
		return scan, fmt.Errorf("bagreplay: decoding LaserScan.angle_increment: %w", err)
	}
	// time_increment, scan_time, range_min, range_max: four float32 the
	// navigator does not consume. Read and discard to reach ranges[].
	for _, name := range []string{"time_increment", "scan_time", "range_min", "range_max"} {
		if _, err = r.readF32(); err != nil {
			return scan, fmt.Errorf("bagreplay: decoding LaserScan.%s: %w", name, err)
		}
	}
	if scan.RangesM, err = r.readF32Seq(); err != nil {
		return scan, fmt.Errorf("bagreplay: decoding LaserScan.ranges: %w", err)
	}
	return scan, nil
}
