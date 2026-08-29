package motor

import (
	"context"
	"errors"
	"log/slog"
	"time"

	"google.golang.org/protobuf/types/known/timestamppb"

	motordriver "github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/motor"
	actuationv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/actuation/v1"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/transport/nats"
)

// Loop holds the running control loop's state: the driver/transport it was
// wired to by NewLoop, plus the mutable state each Run iteration updates
// (last-command time, currently-applied duty, whether the watchdog has
// already safety-stopped).
type Loop struct {
	logger *slog.Logger
	drv    *motordriver.Driver
	pub    *nats.Publisher[*actuationv1.MotorStatus]

	lastCmdAt   time.Time
	currentDuty float64
	stopped     bool
}

// FrameID is the frame_id every MotorStatus this package publishes carries.
const FrameID = "base_link"

// SpeedScalePercentPerMPS converts a commanded AckermannCmd.speed [m/s]
// into a motor duty percentage, matching motors.toml's `drive.speed_scale`
// (motor_speed = velocity_m_s * scale) — see
// platform/robot/config/hardware/motors/motors.toml. Restated here as a
// literal rather than read from a shared loader because
// internal/config/profile (the planned Go equivalent of that TOML file's
// per-component profile loading) doesn't exist yet; migrate this constant
// there once it does.
const SpeedScalePercentPerMPS = 30.0

// MaxDutyPercent is motors.toml's `drive.max_speed`/`min_speed` magnitude:
// motor duty percentage is clamped to [-100, 100].
const MaxDutyPercent = 100.0

// DefaultCommandTimeout is the motor loop's own deadline watchdog: how long
// it will keep driving the last commanded speed after the most recent
// AckermannCmd before treating the command stream as stale and safety-
// stopping. There's no prior Python value to port — the existing ROS2 stack
// relies on DDS DEADLINE QoS instead, which NATS has no equivalent for (see
// ackermann_cmd.proto) — so this is a first-derived value for the Go port,
// exposed as a caller-supplied duration rather than hardcoded so it can be
// tuned on real hardware.
const DefaultCommandTimeout = 500 * time.Millisecond

// watchdogPollInterval is how often Run checks command staleness while no
// new AckermannCmd has arrived.
const watchdogPollInterval = 50 * time.Millisecond

// SpeedToNormalized converts an AckermannCmd's speed [m/s] into the signed
// duty fraction [-1, 1] motor.Actuator.SetSpeed expects, per
// SpeedScalePercentPerMPS.
func SpeedToNormalized(speedMPS float32) float64 {
	percent := float64(speedMPS) * SpeedScalePercentPerMPS
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

// NewLoop builds a Loop over an already-connected drv and pub.
func NewLoop(logger *slog.Logger, drv *motordriver.Driver, pub *nats.Publisher[*actuationv1.MotorStatus]) *Loop {
	return &Loop{logger: logger, drv: drv, pub: pub, lastCmdAt: time.Now(), stopped: true}
}

// Run applies each incoming AckermannCmd and enforces the command-deadline
// watchdog until ctx is done.
func (l *Loop) Run(
	ctx context.Context,
	sub *nats.Subscriber[*actuationv1.AckermannCmd],
	commandTimeout time.Duration,
) error {
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

// applyCommand drives cmd's speed and publishes the resulting MotorStatus.
func (l *Loop) applyCommand(ctx context.Context, cmd *actuationv1.AckermannCmd) {
	l.lastCmdAt = time.Now()
	l.currentDuty = SpeedToNormalized(cmd.GetSpeed())
	l.stopped = false

	setErr := l.drv.SetSpeed(ctx, l.currentDuty)
	if setErr != nil {
		l.logger.Error("node/motor: SetSpeed", "error", setErr)
	}
	if pubErr := l.pub.Publish(StatusFor(l.currentDuty, 0, setErr)); pubErr != nil {
		l.logger.Error("node/motor: publishing MotorStatus", "error", pubErr)
	}
}

// checkWatchdog safety-stops the drive if no AckermannCmd has arrived within
// commandTimeout, and is a no-op otherwise (including once it has already
// stopped for this staleness episode, so it doesn't republish FAULT/IDLE
// status on every single poll tick).
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
	l.logger.Warn("node/motor: command timeout, safety-stopping drive", "age", age)
	if pubErr := l.pub.Publish(StatusFor(l.currentDuty, age, setErr)); pubErr != nil {
		l.logger.Error("node/motor: publishing MotorStatus", "error", pubErr)
	}
}
