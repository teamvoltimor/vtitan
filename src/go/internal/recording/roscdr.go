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

// cdrHeaderLen is the size of the encapsulation header every CDR message
// opens with, and the origin all alignment is measured from.
const cdrHeaderLen = 4

// cdrHeaderLE is the four-byte encapsulation header opening every CDR
// message: representation identifier 0x0001 (little-endian CDR) followed by
// a two-byte options field.
var cdrHeaderLE = [4]byte{0x00, 0x01, 0x00, 0x00}

// cdrWriter builds a CDR buffer honoring the alignment rule composed ROS2
// messages obey: each primitive sits at the next offset that is a multiple
// of its own size.
//
// Offsets are counted from the start of the ENCAPSULATION BODY, i.e. after
// the four-byte header -- not from the start of the buffer. The distinction
// is invisible for 4-byte primitives (the header is itself 4 bytes, so the
// two agree modulo 4) and is exactly wrong for float64: a double at body
// offset 4 must pad to body offset 8, which is buffer offset 12, whereas
// aligning the raw buffer length would leave it at 8. test/bagreplay's
// reader only ever reads 4-byte fields, so it aligns on the absolute
// position and never had to make this distinction.
//
// Composed structs (std_msgs/Header below) do NOT re-prepend a header;
// alignment simply continues from where the parent left off.
type cdrWriter struct {
	buf []byte
}

func newCDRWriter() *cdrWriter {
	return &cdrWriter{buf: append([]byte{}, cdrHeaderLE[:]...)}
}

// align pads to the next multiple of size, measured from the body start.
func (w *cdrWriter) align(size int) {
	if rem := (len(w.buf) - cdrHeaderLen) % size; rem != 0 {
		w.buf = append(w.buf, make([]byte, size-rem)...)
	}
}

func (w *cdrWriter) writeU32(v uint32) {
	w.align(4)
	w.buf = binary.LittleEndian.AppendUint32(w.buf, v)
}

func (w *cdrWriter) writeI32(v int32) { w.writeU32(uint32(v)) }

func (w *cdrWriter) writeF32(v float32) { w.writeU32(math.Float32bits(v)) }

func (w *cdrWriter) writeF64(v float64) {
	w.align(8)
	w.buf = binary.LittleEndian.AppendUint64(w.buf, math.Float64bits(v))
}

// writeF64Array writes a FIXED-SIZE float64 array (no length prefix), the
// shape ROS covariance fields use.
func (w *cdrWriter) writeF64Array(values []float64) {
	for _, v := range values {
		w.writeF64(v)
	}
}

// writeHeader writes a std_msgs/Header: builtin_interfaces/Time then a
// string frame_id.
func (w *cdrWriter) writeHeader(sec int32, nanosec uint32, frameID string) {
	w.writeI32(sec)
	w.writeU32(nanosec)
	w.writeString(frameID)
}

// writeVector3 writes a geometry_msgs/Vector3: three float64.
func (w *cdrWriter) writeVector3(x, y, z float64) {
	w.writeF64(x)
	w.writeF64(y)
	w.writeF64(z)
}

// writeQuaternion writes a geometry_msgs/Quaternion in ROS field order,
// x/y/z/w -- w LAST, which is the opposite of how quaternions are usually
// written down and an easy way to produce a robot that renders lying on its
// side.
func (w *cdrWriter) writeQuaternion(x, y, z, wq float64) {
	w.writeF64(x)
	w.writeF64(y)
	w.writeF64(z)
	w.writeF64(wq)
}

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

// The remaining ROS2 types this recorder writes, with their .msg
// definitions embedded for the same self-describing reason as LaserScan.
const (
	// ImuType/ImuSchema describe sensor_msgs/msg/Imu.
	ImuType   = "sensor_msgs/msg/Imu"
	ImuSchema = `std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
`

	// TFMessageType/TFMessageSchema describe tf2_msgs/msg/TFMessage.
	TFMessageType   = "tf2_msgs/msg/TFMessage"
	TFMessageSchema = `geometry_msgs/TransformStamped[] transforms
================================================================================
MSG: geometry_msgs/TransformStamped
std_msgs/Header header
string child_frame_id
geometry_msgs/Transform transform
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/Transform
geometry_msgs/Vector3 translation
geometry_msgs/Quaternion rotation
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
`

	// AckermannType/AckermannSchema describe
	// ackermann_msgs/msg/AckermannDriveStamped.
	AckermannType   = "ackermann_msgs/msg/AckermannDriveStamped"
	AckermannSchema = `std_msgs/Header header
ackermann_msgs/AckermannDrive drive
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: ackermann_msgs/AckermannDrive
float32 steering_angle
float32 steering_angle_velocity
float32 speed
float32 acceleration
float32 jerk
`

	// Float32Type/Float32Schema describe std_msgs/msg/Float32.
	Float32Type   = "std_msgs/msg/Float32"
	Float32Schema = "float32 data\n"
)

// EncodeFloat32 serializes a std_msgs/msg/Float32.
func EncodeFloat32(v float32) []byte {
	w := newCDRWriter()
	w.writeF32(v)
	return w.buf
}

// ImuCDR is the subset of sensor_msgs/msg/Imu the simulator can honestly
// fill: an orientation and a yaw rate. Covariances are written as zeros,
// which ROS reads as "unknown" rather than "perfectly certain" only when the
// first element is -1 -- so the linear acceleration, which this model does
// not produce at all, is marked that way instead of shipped as a confident
// zero.
type ImuCDR struct {
	FrameID      string
	StampSec     int32
	StampNanosec uint32
	// YawRad is the heading the IMU REPORTS, not ground truth: with
	// sensorerrors configured the two differ, and a bag that recorded truth
	// here would hide exactly the error being modelled.
	YawRad       float64
	YawRateRadPS float64
}

// EncodeImu serializes an Imu message. Only yaw is modelled, so the
// quaternion is a rotation about Z alone.
func EncodeImu(imu ImuCDR) []byte {
	w := newCDRWriter()
	w.writeHeader(imu.StampSec, imu.StampNanosec, imu.FrameID)

	half := imu.YawRad / 2
	w.writeQuaternion(0, 0, math.Sin(half), math.Cos(half))
	w.writeF64Array(make([]float64, 9))

	w.writeVector3(0, 0, imu.YawRateRadPS)
	w.writeF64Array(make([]float64, 9))

	w.writeVector3(0, 0, 0)
	// -1 in the first covariance element is ROS's "this field is not
	// measured", which is the truth here: the kinematic model has no
	// accelerometer.
	unknown := make([]float64, 9)
	unknown[0] = -1
	w.writeF64Array(unknown)
	return w.buf
}

// TransformCDR is one geometry_msgs/TransformStamped.
type TransformCDR struct {
	FrameID      string
	ChildFrameID string
	StampSec     int32
	StampNanosec uint32
	X, Y, Z      float64
	YawRad       float64
}

// EncodeTFMessage serializes a tf2_msgs/msg/TFMessage carrying transforms.
//
// This is what makes a bag show a robot DRIVING rather than a scan sweeping
// in place: without a transform tree every consumer can only draw the scan
// in its own sensor frame. The simulator can publish it honestly because it
// knows ground truth, which is precisely what a real recording cannot do --
// so a sim bag is strictly richer here than a pulled one.
func EncodeTFMessage(transforms []TransformCDR) []byte {
	w := newCDRWriter()
	w.writeU32(uint32(len(transforms)))
	for _, tf := range transforms {
		w.writeHeader(tf.StampSec, tf.StampNanosec, tf.FrameID)
		w.writeString(tf.ChildFrameID)
		w.writeVector3(tf.X, tf.Y, tf.Z)
		half := tf.YawRad / 2
		w.writeQuaternion(0, 0, math.Sin(half), math.Cos(half))
	}
	return w.buf
}

// AckermannCDR is an ackermann_msgs/msg/AckermannDriveStamped: the command
// the navigator issued, in the same units /ackermann_cmd carries on the
// robot (radians for the steering angle, m/s for speed).
type AckermannCDR struct {
	FrameID          string
	StampSec         int32
	StampNanosec     uint32
	SteeringAngleRad float32
	SpeedMPS         float32
}

// EncodeAckermannDriveStamped serializes an AckermannDriveStamped. The
// steering-angle velocity, acceleration and jerk fields are zero: the
// navigator commands none of them, and the robot's own publisher leaves
// them unset too.
func EncodeAckermannDriveStamped(cmd AckermannCDR) []byte {
	w := newCDRWriter()
	w.writeHeader(cmd.StampSec, cmd.StampNanosec, cmd.FrameID)
	w.writeF32(cmd.SteeringAngleRad)
	w.writeF32(0)
	w.writeF32(cmd.SpeedMPS)
	w.writeF32(0)
	w.writeF32(0)
	return w.buf
}

// PathType/PathSchema describe nav_msgs/msg/Path, the message Foxglove
// draws a planned route from and the one the Python live visualizer
// publishes its plan on.
const (
	PathType   = "nav_msgs/msg/Path"
	PathSchema = `std_msgs/Header header
geometry_msgs/PoseStamped[] poses
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/PoseStamped
std_msgs/Header header
geometry_msgs/Pose pose
================================================================================
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
`
)

// PathPointCDR is one waypoint of a planned route. Only a position: the
// planner produces points to drive through, not headings to hold at them,
// and inventing an orientation per point would draw a confidence the plan
// does not express.
type PathPointCDR struct {
	X, Y float64
}

// PathCDR is a nav_msgs/msg/Path.
type PathCDR struct {
	FrameID      string
	Points       []PathPointCDR
	StampSec     int32
	StampNanosec uint32
}

// EncodePath serializes a Path. Every pose carries its own header, which is
// redundant with the outer one and is what the message definition requires;
// omitting it would shift every field after the first pose.
func EncodePath(path PathCDR) []byte {
	w := newCDRWriter()
	w.writeHeader(path.StampSec, path.StampNanosec, path.FrameID)
	w.writeU32(uint32(len(path.Points)))
	for _, p := range path.Points {
		w.writeHeader(path.StampSec, path.StampNanosec, path.FrameID)
		w.writeVector3(p.X, p.Y, 0)
		// Identity orientation: see PathPointCDR.
		w.writeQuaternion(0, 0, 0, 1)
	}
	return w.buf
}

// StringType/StringSchema describe std_msgs/msg/String, the envelope the
// Python stack publishes its JSON telemetry inside (/nav_debug,
// /race_metrics, /robot_state).
const (
	StringType   = "std_msgs/msg/String"
	StringSchema = "string data\n"
)

// EncodeString serializes a std_msgs/msg/String.
func EncodeString(s string) []byte {
	w := newCDRWriter()
	w.writeString(s)
	return w.buf
}
