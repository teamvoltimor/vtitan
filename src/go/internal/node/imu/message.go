package imu

import (
	"google.golang.org/protobuf/types/known/timestamppb"

	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
	"github.com/teamvoltimor/vtitan/src/go/pkg/driver/imu"
)

// FrameID is this sensor's TF frame, matching
// shared.config.constants.identifiers.TfFrames.IMU_LINK (the value the
// existing ROS2 uart_rvc_node.py publishes Imu messages under).
const FrameID = "imu_link"

// orientationCovarianceUnknown/angularVelocityCovarianceUnknown are the
// row-major 3x3 "no estimate" covariance matrices sensor_msgs/Imu's own
// convention defines (element [0] = -1, the rest 0) -- matches
// uart_rvc_node.py exactly: RVC mode reports Euler angles (used to derive
// orientation) but never angular velocity, so only orientation gets a real
// (if unknown-magnitude) covariance marker and angular velocity is zeroed
// out entirely.
var orientationCovarianceUnknown = [9]float64{-1, 0, 0, 0, 0, 0, 0, 0, 0}

var angularVelocityCovarianceUnknown = [9]float64{-1, 0, 0, 0, 0, 0, 0, 0, 0}

// linearAccelerationCovarianceDiag01 is uart_rvc_node.py's approximate
// diagonal 0.01 covariance for linear acceleration -- not a measured
// sensor spec, the Python driver's own comment calls it approximate too.
var linearAccelerationCovarianceDiag01 = [9]float64{0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01}

// vec3 builds a sensor_msgs/Vector3-equivalent message.
func vec3(x, y, z float64) *sensorv1.Vector3 {
	return &sensorv1.Vector3{X: x, Y: y, Z: z}
}

// MessageFor builds the Imu message to publish for one decoded RVC reading.
func MessageFor(reading imu.Reading) *sensorv1.Imu {
	q := imu.QuaternionFromEuler(reading.Yaw, reading.Pitch, reading.Roll)

	return &sensorv1.Imu{
		Stamp:   timestamppb.Now(),
		FrameId: FrameID,

		Orientation:           &sensorv1.Quaternion{X: q.X, Y: q.Y, Z: q.Z, W: q.W},
		OrientationCovariance: orientationCovarianceUnknown[:],

		AngularVelocity:           vec3(0, 0, 0),
		AngularVelocityCovariance: angularVelocityCovarianceUnknown[:],

		LinearAcceleration:           vec3(reading.XAccel, reading.YAccel, reading.ZAccel),
		LinearAccelerationCovariance: linearAccelerationCovarianceDiag01[:],
	}
}
