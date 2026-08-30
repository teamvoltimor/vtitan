package controllers

import (
	"math"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// LidarScan is a single LIDAR sweep in the robot frame (0 rad = forward,
// +pi/2 = left), matching ports.LidarScan.
type LidarScan struct {
	RangesM   []float64
	AnglesRad []float64
}

// DriveCommand is the motor command contract every adapter must decode
// identically, matching ports.DriveCommand.
type DriveCommand struct {
	SpeedMPS float64
	// SteeringNorm is in [-1, 1], + = left.
	SteeringNorm float64
}

// WheelOdometry is the distance traveled at the wheel and the rate it is
// traveling, matching ports.WheelOdometry.
//
// Deliberately not a pose: the encoder measures wheel rotation and nothing
// else. DistanceM accumulates from an arbitrary zero, so only differences
// between samples are meaningful; StampS is required for the same reason.
type WheelOdometry struct {
	DistanceM float64
	SpeedMPS  float64
	StampS    float64
}

// DriveSink commands the robot to drive at a given speed and steering angle,
// matching ports.HardwareGateway.PublishDrive.
type DriveSink interface {
	// PublishDrive commands the robot to drive at the given speed and
	// steering angle.
	PublishDrive(command DriveCommand)
}

// PoseSource reports the robot's current estimated state, matching the
// read-side of ports.HardwareGateway (pose, LIDAR sweep, wheel odometry).
type PoseSource interface {
	// GetCurrentPose returns the current estimated pose of the robot, and
	// ok=false if none is available yet.
	GetCurrentPose() (pose trackmodel.Pose, ok bool)
	// GetLidarScan returns the latest LIDAR sweep, and ok=false if none
	// is available yet.
	GetLidarScan() (scan LidarScan, ok bool)
	// GetWheelOdometry returns the latest wheel travel and speed, and
	// ok=false if unavailable -- a normal state, not an error: a drive
	// backend without an encoder has nothing to report.
	GetWheelOdometry() (odometry WheelOdometry, ok bool)
}

// WallSetter re-points the localizer at the layout the robot currently
// believes in, matching ports.HardwareGateway.SetBelievedWalls.
type WallSetter interface {
	// SetBelievedWalls re-points the localizer at the layout the robot
	// currently believes in. Blind operation estimates corridor widths as
	// it drives, so the wall model must be updated mid-round.
	SetBelievedWalls(walls *trackmodel.TrackWalls)
}

// PositionResetter re-seeds the estimator's position and heading reference,
// matching ports.HardwareGateway.ResetPosition / ResetHeadingReference.
type PositionResetter interface {
	// ResetPosition re-seeds the estimator's position, e.g. at the start
	// of a new race.
	ResetPosition(x, y float64)
	// ResetHeadingReference re-zeros the estimator's heading against the
	// next IMU reading.
	ResetHeadingReference()
}

// HeadingCorrector shifts the estimator's heading by a known amount, matching
// ports.HardwareGateway.CorrectHeadingForDirectionChange.
type HeadingCorrector interface {
	// CorrectHeadingForDirectionChange shifts the estimator's heading by
	// a known amount, applied in full -- used when blind direction
	// inference overturns the direction assumed at construction.
	CorrectHeadingForDirectionChange(deltaRad float64)
}

// HardwareGateway is the interface for robot hardware interaction (ROS2 or
// simulation), matching ports.HardwareGateway. Declared here -- not in a
// hardware/simulation package -- so the dependency direction matches
// hexagonal architecture: adapters import this port, this domain package
// never imports an adapter. It composes the smaller, single-responsibility
// ports above (DriveSink, PoseSource, WallSetter, PositionResetter,
// HeadingCorrector) so each can be consumed independently where only part of
// the hardware surface is needed (e.g. a sim runner that only advances
// state). Both implementers satisfy it trivially by implementing all methods.
type HardwareGateway interface { //nolint:interfacebloat // composed of 5 small ports (each <=3 methods); the aggregate is the 1:1 ports.HardwareGateway contract
	DriveSink
	PoseSource
	WallSetter
	PositionResetter
	HeadingCorrector
}

// SanitizeLidarRanges replaces non-finite LIDAR returns with lidarMaxRangeM,
// then clips to [0, lidarMaxRangeM], matching ports.sanitize_lidar_ranges.
//
// Slamtec drivers (and the simulator's synthetic dropout model) emit
// no-return rays as NaN/inf: NaN silently drops out of every downstream
// mask and inf reads as "far away", so both must become max range before
// anything else touches the scan.
func SanitizeLidarRanges(ranges []float64, lidarMaxRangeM float64) []float64 {
	out := make([]float64, len(ranges))
	for i, r := range ranges {
		if !math.IsInf(r, 0) && !math.IsNaN(r) {
			out[i] = r
		} else {
			out[i] = lidarMaxRangeM
		}
		out[i] = navutil.Clamp(out[i], 0.0, lidarMaxRangeM)
	}
	return out
}
