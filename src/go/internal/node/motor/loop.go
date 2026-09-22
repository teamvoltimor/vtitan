//go:build linux

package motor

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"math"
	"time"

	"google.golang.org/protobuf/types/known/timestamppb"

	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
)

// SpeedSetter is the one driver call the loop makes; *motor.Driver
// satisfies it. It is an interface so the failsafe behaviour can be tested
// without an H-bridge.
type SpeedSetter interface {
	SetSpeed(ctx context.Context, normalizedSpeed float64) error
}

// StatusPublisher is the one transport call the loop makes on the way out;
// *nats.Publisher[*actuationv1.MotorStatus] satisfies it.
type StatusPublisher interface {
	Publish(msg *actuationv1.MotorStatus) error
}

// CommandReader is the one transport call the loop makes on the way in;
// *nats.Subscriber[*actuationv1.AckermannCmd] satisfies it.
type CommandReader interface {
	Read(ctx context.Context) (*actuationv1.AckermannCmd, error)
}

// Loop holds the running control loop's state: the drivers/transport it was
// wired to by NewLoop, plus the mutable state each Run iteration updates
// (last-command time, currently-applied duty, whether the watchdog has
// already safety-stopped).
type Loop struct {
	logger                  *slog.Logger
	drv                     SpeedSetter
	pub                     StatusPublisher
	speedScalePercentPerMPS float64
	steering                Steering

	lastCmdAt   time.Time
	currentDuty float64
	stopped     bool
}

// FrameID is the frame_id every MotorStatus this package publishes carries.
const FrameID = "base_link"

// DefaultSpeedScalePercentPerMPS converts a commanded AckermannCmd.speed
// [m/s] into a motor duty percentage, matching motors.toml's
// `drive.speed_scale` (motor_speed = velocity_m_s * scale) - see
// src/config/hardware/motors/motors.toml. A caller with real
// hardware-profile data should load motors.HardwareMotorsMotors instead
// (internal/config/profile) and pass its Drive.SpeedScale to NewLoop; this
// is the fallback for callers that don't.
const DefaultSpeedScalePercentPerMPS = 30.0

// MaxDutyPercent is motors.toml's `drive.max_speed`/`min_speed` magnitude:
// motor duty percentage is clamped to [-100, 100].
const MaxDutyPercent = 100.0

// DefaultCommandTimeout is the motor loop's own deadline watchdog: how long
// it will keep driving the last commanded speed after the most recent
// AckermannCmd before treating the command stream as stale and safety-
// stopping. It is a first-derived value for the Go port, exposed as a
// caller-supplied duration rather than hardcoded so it can be tuned on real
// hardware. adr:0068-go-parallel-track-single-cutover
const DefaultCommandTimeout = 500 * time.Millisecond

// watchdogPollInterval is how often Run checks command staleness while no
// new AckermannCmd has arrived.
const watchdogPollInterval = 50 * time.Millisecond

// exitStopTimeout bounds the safety-stop Run performs on its way out. Run's
// own ctx may already be done by then, so the stop drops its cancellation.
const exitStopTimeout = 250 * time.Millisecond

// SpeedToNormalized converts an AckermannCmd's speed [m/s] into the signed
// duty fraction [-1, 1] motor.Actuator.SetSpeed expects, per
// scalePercentPerMPS (see DefaultSpeedScalePercentPerMPS).
func SpeedToNormalized(speedMPS float32, scalePercentPerMPS float64) float64 {
	percent := float64(speedMPS) * scalePercentPerMPS
	clamped := min(max(percent, -MaxDutyPercent), MaxDutyPercent)
	return clamped / MaxDutyPercent
}

// StatusFor builds the MotorStatus to publish after applying a command or a
// watchdog safety-stop.
func StatusFor(dutyFraction float64, commandAge time.Duration, setSpeedErr error) *actuationv1.MotorStatus {
	state := actuationv1.MotorStatus_STATE_IDLE
	detail := ""
	switch {
	case setSpeedErr != nil:
		state = actuationv1.MotorStatus_STATE_FAULT
		detail = setSpeedErr.Error()
	case dutyFraction != 0:
		state = actuationv1.MotorStatus_STATE_RUNNING
	}

	return &actuationv1.MotorStatus{
		Stamp:        timestamppb.Now(),
		FrameId:      FrameID,
		State:        state,
		Detail:       detail,
		DutyCycle:    float32(dutyFraction),
		CommandAgeMs: uint32(commandAge.Milliseconds()),
	}
}

// NewLoop builds a Loop over an already-connected drv and pub, converting
// commanded speeds using speedScalePercentPerMPS (see
// DefaultSpeedScalePercentPerMPS) and steering through steering (whose
// servo, if any, is already connected and centered).
func NewLoop(
	logger *slog.Logger,
	drv SpeedSetter,
	pub StatusPublisher,
	speedScalePercentPerMPS float64,
	steering Steering,
) *Loop {
	return &Loop{
		logger:                  logger,
		drv:                     drv,
		pub:                     pub,
		speedScalePercentPerMPS: speedScalePercentPerMPS,
		steering:                steering,
		lastCmdAt:               time.Now(),
		stopped:                 true,
	}
}

// Run applies each incoming AckermannCmd and enforces the command-deadline
// watchdog until ctx is done.
//
// Whatever way Run returns, it stops the drive and centers the steering
// first. The watchdog only runs while Run does, and under supervise a failed
// Run is restarted after a backoff of 1 s, then 2 s, 4 s...: without this
// stop the motor would hold its last duty, and the servo its last angle,
// unsupervised, for the whole of that wait.
func (l *Loop) Run(
	ctx context.Context,
	sub CommandReader,
	commandTimeout time.Duration,
) error {
	defer l.stopOnExit(ctx)

	cmdCh := make(chan *actuationv1.AckermannCmd)
	readErrCh := make(chan error, 1)
	go func() {
		for {
			cmd, err := sub.Read(ctx)
			if err != nil {
				readErrCh <- err
				return
			}
			select {
			case cmdCh <- cmd:
			case <-ctx.Done():
				return
			}
		}
	}()

	ticker := time.NewTicker(watchdogPollInterval)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			return nil
		case err := <-readErrCh:
			if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
				return nil
			}
			return err
		case cmd := <-cmdCh:
			l.applyCommand(ctx, cmd)
		case <-ticker.C:
			l.checkWatchdog(ctx, commandTimeout)
		}
	}
}

// applyCommand steers and drives per cmd and publishes the resulting
// MotorStatus. Steering goes first, as in ackermann_motor_node.py:597.
//
// A non-finite speed or steering angle rejects the whole command, which is
// treated as missing: it does not refresh the watchdog, so a stream of them
// stops the drive and centers the steering exactly as silence would. Acting
// on it is not an option - +Inf clamps to full forward or full lock, and NaN
// survives Go's min/max into the PWM layer. Half-applying it (the finite
// field only) would act on a command its sender got wrong.
func (l *Loop) applyCommand(ctx context.Context, cmd *actuationv1.AckermannCmd) {
	speed := float64(cmd.GetSpeed())
	steer := float64(cmd.GetSteeringAngle())
	if !finite(speed) || !finite(steer) {
		l.logger.Warn("node/motor: rejecting non-finite command, treating as missing",
			"speed", speed, "steering_angle", steer)
		return
	}

	l.lastCmdAt = time.Now()
	l.currentDuty = SpeedToNormalized(cmd.GetSpeed(), l.speedScalePercentPerMPS)
	l.stopped = false

	steerErr := l.steer(cmd.GetSteeringAngle())
	setErr := l.drv.SetSpeed(ctx, l.currentDuty)
	if setErr != nil {
		l.logger.Error("node/motor: SetSpeed", "error", setErr)
	}
	if pubErr := l.pub.Publish(StatusFor(l.currentDuty, 0, errors.Join(steerErr, setErr))); pubErr != nil {
		l.logger.Error("node/motor: publishing MotorStatus", "error", pubErr)
	}
}

// steer converts a wheel angle [rad] to a servo angle and applies it. It is
// a no-op without a servo (motor-node).
func (l *Loop) steer(steeringAngleRad float32) error {
	if l.steering.Servo == nil {
		return nil
	}
	servoDeg, clamped := SteeringToServoDeg(steeringAngleRad, l.steering.Config)
	if clamped {
		// Debug, not Python's per-command warning (ackermann_motor_node.py:562):
		// full lock is routine in escapes, and at the command rate a warning
		// per command would flood the Zero's log.
		l.logger.Debug("node/motor: servo angle clamped to travel limit",
			"wheel_angle_rad", steeringAngleRad, "servo_deg", servoDeg,
			"limit_deg", l.steering.Config.ServoMaxAngleDeg)
	}
	if err := l.steering.Servo.SetAngle(servoDeg); err != nil {
		l.logger.Error("node/motor: SetAngle", "error", err)
		return fmt.Errorf("steering: %w", err)
	}
	return nil
}

// center commands the servo to SteeringCenterDeg; a no-op without a servo.
func (l *Loop) center(why string) {
	if l.steering.Servo == nil {
		return
	}
	if err := l.steering.Servo.SetAngle(SteeringCenterDeg); err != nil {
		l.logger.Error("node/motor: centering steering", "on", why, "error", err)
	}
}

// checkWatchdog safety-stops the drive and centers the steering if no
// AckermannCmd has arrived within commandTimeout, and is a no-op otherwise
// (including once it has already stopped for this staleness episode, so it
// doesn't republish FAULT/IDLE status on every single poll tick).
//
// Centering follows go-future.md 2.5 contract row 1 ("steering to center,
// drive to neutral"). Python's _watchdog_check only stops the drive
// (ackermann_motor_node.py:770-790) and leaves the servo at its last angle.
func (l *Loop) checkWatchdog(ctx context.Context, commandTimeout time.Duration) {
	age := time.Since(l.lastCmdAt)
	if age < commandTimeout || l.stopped {
		return
	}

	l.stopped = true
	l.currentDuty = 0
	setErr := l.drv.SetSpeed(ctx, l.currentDuty)
	if setErr != nil {
		l.logger.Error("node/motor: safety-stop SetSpeed", "error", setErr)
	}
	l.center("command timeout")
	l.logger.Warn("node/motor: command timeout, safety-stopping drive and centering steering", "age", age)
	if pubErr := l.pub.Publish(StatusFor(l.currentDuty, age, setErr)); pubErr != nil {
		l.logger.Error("node/motor: publishing MotorStatus", "error", pubErr)
	}
}

// stopOnExit zeroes the drive and centers the steering when Run returns,
// unless the watchdog already has (ackermann_motor_node.py:489-500
// _stop_motors_safely does both). It publishes no status: on shutdown the
// connection may be gone, and the next Run (if any) reports state from its
// first command.
func (l *Loop) stopOnExit(runCtx context.Context) {
	if l.stopped {
		return
	}
	ctx, cancel := context.WithTimeout(context.WithoutCancel(runCtx), exitStopTimeout)
	defer cancel()

	l.stopped = true
	l.currentDuty = 0
	if err := l.drv.SetSpeed(ctx, l.currentDuty); err != nil {
		l.logger.Error("node/motor: safety-stop on exit", "error", err)
	}
	l.center("exit")
}

func finite(v float64) bool {
	return !math.IsNaN(v) && !math.IsInf(v, 0)
}
