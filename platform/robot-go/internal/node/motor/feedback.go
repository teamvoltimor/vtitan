//go:build linux

package motor

import (
	"context"
	"log/slog"
	"math"
	"time"

	"google.golang.org/protobuf/types/known/timestamppb"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/encoder"
	actuationv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/actuation/v1"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/transport/nats"
)

// OdometrySource is the encoder half the feedback loop reads. It is the
// subset of *encoder.Quadrature this package uses, declared here so the
// loop can be tested without GPIO -- the same port-shaped split
// controllers.HardwareGateway draws for the navigator.
type OdometrySource interface {
	// RPM samples the counter and advances the speed estimator's window.
	RPM() (float64, error)
	// Odometry returns the accumulated travel, using the RPM last sampled.
	Odometry() (encoder.Odometry, error)
}

// DefaultFeedbackInterval is how often the feedback loop samples the
// encoder and publishes JointStates: 50Hz, matching the rate the navigator
// drives its own control loop at, so a consumer differencing successive
// distances (internal/nav/bayexit) never has to wait more than one nav tick
// for a fresh sample.
const DefaultFeedbackInterval = 20 * time.Millisecond

// radiansPerRevolution converts the encoder's revolution count into the
// accumulating wheel ANGLE that JointStates.position carries.
const radiansPerRevolution = 2 * math.Pi

// Feedback publishes wheel odometry as JointStates on
// vtitan.actuation.v1.joint_states, the subject
// natsgw.Gateway.GetWheelOdometry reads and the direct analogue of the
// /joint_states topic ackermann_motor_node.py publishes and
// ros2_hardware_gateway.py consumes.
//
// It carries what MotorStatus.measured_speed cannot: an ACCUMULATING wheel
// angle rather than an instantaneous speed, plus the stamp that says when
// the sample was taken. Both are what a motion prior needs -- wheel
// distance is angle x wheel radius, and integrating it between LIDAR scans
// is only meaningful if you know the interval.
//
// Only the drive joint is published. Python publishes a steering entry
// alongside it, but the Go stack has no servo driver in this process to
// read a position from, and a zero-filled steering joint would be a
// fabricated measurement rather than a missing one.
type Feedback struct {
	logger   *slog.Logger
	source   OdometrySource
	pub      *nats.Publisher[*actuationv1.JointStates]
	interval time.Duration
}

// NewFeedback builds a Feedback over source and pub, sampling every
// interval (DefaultFeedbackInterval when interval is not positive).
func NewFeedback(
	logger *slog.Logger,
	source OdometrySource,
	pub *nats.Publisher[*actuationv1.JointStates],
	interval time.Duration,
) *Feedback {
	if interval <= 0 {
		interval = DefaultFeedbackInterval
	}
	return &Feedback{logger: logger, source: source, pub: pub, interval: interval}
}

// JointStatesFor builds the message for one odometry sample. position is
// the accumulated wheel angle [rad] and velocity the wheel rate [rad/s],
// both SI -- unlike the degree-based topics the Python node also publishes.
func JointStatesFor(odometry encoder.Odometry) *actuationv1.JointStates {
	return &actuationv1.JointStates{
		Stamp:    timestamppb.Now(),
		FrameId:  FrameID,
		Name:     []string{actuationv1.DriveJoint},
		Position: []float64{odometry.Revolutions * radiansPerRevolution},
		Velocity: []float64{odometry.RPM * radiansPerRevolution / secondsPerMinute},
	}
}

// secondsPerMinute converts the encoder's RPM into the rad/s
// JointStates.velocity carries.
const secondsPerMinute = 60.0

// Run samples the encoder and publishes JointStates until ctx is done.
//
// A sampling error is logged and skipped rather than returned: the encoder
// is a telemetry source, not the control path, and tearing down the motor
// node (and with it the command-deadline watchdog) over a transient GPIO
// read would trade a missing odometry sample for a robot that keeps
// driving its last command.
func (f *Feedback) Run(ctx context.Context) error {
	ticker := time.NewTicker(f.interval)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			return nil
		case <-ticker.C:
			f.publishOnce()
		}
	}
}

// publishOnce samples the encoder once and publishes the result. RPM is
// called before Odometry because Odometry reports the CACHED rpm -- see
// encoder.Quadrature.Odometry for why it must not resample.
func (f *Feedback) publishOnce() {
	if _, err := f.source.RPM(); err != nil {
		f.logger.Error("node/motor: sampling encoder RPM", "error", err)
		return
	}
	odometry, err := f.source.Odometry()
	if err != nil {
		f.logger.Error("node/motor: reading encoder odometry", "error", err)
		return
	}
	if pubErr := f.pub.Publish(JointStatesFor(odometry)); pubErr != nil {
		f.logger.Error("node/motor: publishing JointStates", "error", pubErr)
	}
}
