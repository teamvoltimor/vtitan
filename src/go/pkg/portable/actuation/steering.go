package actuation

import (
	"errors"
	"fmt"
	"math"

	"github.com/teamvoltimor/vtitan/src/go/pkg/geom"
)

// SteeringConfig converts an AckermannCmd's wheel angle into a servo angle,
// the way ackermann_motor_node.py's _ackermann_callback does.
type SteeringConfig struct {
	// LinkageRatio is road-wheel degrees per servo degree:
	// robot.toml max_wheel_angle_deg / servo_max_angle_deg
	// (profile.RobotConfig.LinkageRatio, Python RobotSpecs.LINKAGE_RATIO).
	LinkageRatio float64
	// ServoMaxAngleDeg is the servo's travel limit, +/- (robot.toml
	// steering.servo_max_angle_deg, from the servo profile).
	ServoMaxAngleDeg float64
	// OffsetDeg is motors.toml steering.offset: a servo-side trim added
	// after the linkage conversion.
	OffsetDeg float64
}

// SteeringCenterDeg is the servo angle the motor loop commands on a
// watchdog timeout and on its exit (motors/base.py:40
// STEERING_CENTER_DEG). As in Python's center_steering, the offset trim is
// not added to it.
const SteeringCenterDeg = 0.0

// Validate rejects a conversion that cannot produce a finite, bounded
// servo angle.
func (c SteeringConfig) Validate() error {
	if !(c.LinkageRatio > 0) || math.IsInf(c.LinkageRatio, 0) {
		return fmt.Errorf("actuation: steering linkage ratio %v must be positive and finite", c.LinkageRatio)
	}
	if !(c.ServoMaxAngleDeg > 0) || math.IsInf(c.ServoMaxAngleDeg, 0) {
		return fmt.Errorf("actuation: servo max angle %v must be positive and finite", c.ServoMaxAngleDeg)
	}
	if math.IsNaN(c.OffsetDeg) || math.IsInf(c.OffsetDeg, 0) {
		return errors.New("actuation: steering offset must be finite")
	}
	return nil
}

// SteeringToServoDeg is ackermann_motor_node.py:545-559: the command's
// wheel angle [rad] to degrees, divided by the linkage ratio (the message
// carries a WHEEL angle, the servo needs a SERVO angle; feeding one as the
// other under-turned the wheels by ~22%), plus the offset trim, clamped to
// +/-ServoMaxAngleDeg. clamped reports whether the clamp bit. The sign is
// passed through untouched: positive (left, per the ROS convention and
// natsgw's driveCommand) stays positive, and servo.Config.Reversed is the
// one place it may flip. The caller rejects a non-finite angle first.
func SteeringToServoDeg(steeringAngleRad float32, cfg SteeringConfig) (servoDeg float64, clamped bool) {
	wheelDeg := float64(steeringAngleRad) * geom.DegreesPerHalfTurn / math.Pi
	calibrated := wheelDeg/cfg.LinkageRatio + cfg.OffsetDeg
	limit := cfg.ServoMaxAngleDeg
	return min(max(calibrated, -limit), limit), math.Abs(calibrated) > limit
}
