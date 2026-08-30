package kinematics

import "github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"

// AckermannState is the full kinematic state of the simulated car,
// matching src.simulation.kinematics.AckermannState. (X, Y) is the
// chassis center, not the rear axle -- see the package doc for why.
type AckermannState struct {
	X, Y, Yaw float64
	// V is the current linear speed (m/s).
	V float64
	// Steer is the current front-wheel angle (radians).
	Steer float64
}

// WheelPose is one road wheel's chassis-frame mount point and its own
// steer angle, matching src.simulation.kinematics.WheelPose. Name matches
// the corresponding URDF link in
// platform/gazebo/runtime/robot_description/wro_robot.urdf.xacro so the
// two descriptions of the same wheel can be lined up by eye.
type WheelPose struct {
	Name string
	// X is +forward, Y is +left, both measured from the chassis center.
	X, Y float64
	// Steer is this wheel's own angle (radians, +ve = turning left).
	Steer float64
}

// WheelPoseSet is the four road wheels' poses, matching the four-tuple
// (front_left, front_right, rear_left, rear_right) that wheel_poses()
// returns in Python. A named struct rather than an array/tuple, since Go
// has no positional unpacking -- named fields keep call sites as
// self-documenting as the Python tuple order.
type WheelPoseSet struct {
	FrontLeft, FrontRight, RearLeft, RearRight WheelPose
}

// URDF link names, matching wro_robot.urdf.xacro.
const (
	frontLeftWheelName  = "front_left_wheel"
	frontRightWheelName = "front_right_wheel"
	rearLeftWheelName   = "rear_left_wheel"
	rearRightWheelName  = "rear_right_wheel"
)

// WheelPoses places the four road wheels for a bicycle-equivalent front
// angle, matching wheel_poses(). Both wheels on an axle carry the SAME
// angle: the vTitan turns each axle with one servo through one linkage, so
// there is no inner/outer Ackermann differential to render here.
//
// The rear axle is the front angle negated and scaled by rearSteerRatio.
// That counter-phase is the whole reason this chassis yaws twice as fast
// as a front-steer car at the same angle (see the package doc).
//
// Coordinates are relative to the chassis center, matching AckermannState.
func WheelPoses(steer, wheelbase, trackWidth, rearSteerRatio float64) WheelPoseSet {
	rearSteer := -steer * rearSteerRatio
	halfWheelbase := wheelbase / navutil.Half
	halfTrack := trackWidth / navutil.Half
	return WheelPoseSet{
		FrontLeft:  WheelPose{Name: frontLeftWheelName, X: halfWheelbase, Y: halfTrack, Steer: steer},
		FrontRight: WheelPose{Name: frontRightWheelName, X: halfWheelbase, Y: -halfTrack, Steer: steer},
		RearLeft:   WheelPose{Name: rearLeftWheelName, X: -halfWheelbase, Y: halfTrack, Steer: rearSteer},
		RearRight:  WheelPose{Name: rearRightWheelName, X: -halfWheelbase, Y: -halfTrack, Steer: rearSteer},
	}
}
