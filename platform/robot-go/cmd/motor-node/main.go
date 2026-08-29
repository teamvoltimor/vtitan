//go:build linux

// Command motor-node is a bench/dev single-subsystem binary for the BTS7960
// motor control loop, sharing the same internal packages as cmd/pi-zero —
// used for isolated hardware bench testing and local debugging.
//
// It subscribes to AckermannCmd on the `vtitan.actuation.v1.ackermann_cmd`
// NATS subject and publishes MotorStatus on
// `vtitan.actuation.v1.motor_status`, enforcing its own command-deadline
// watchdog (see ackermann_cmd.proto's docstring: NATS has no DDS DEADLINE
// QoS equivalent, so the motor node must detect a stale command itself and
// stop the drive rather than keep applying the last one it heard).
package main

import (
	"context"
	"errors"
	"log/slog"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/spf13/cobra"
	"google.golang.org/protobuf/types/known/timestamppb"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/motor"
	actuationv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/actuation/v1"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/transport/nats"
)

// cliConfig holds every flag motor-node accepts.
type cliConfig struct {
	natsURL        string
	nodeName       string
	commandTimeout time.Duration
	invert         bool
}

// motorLoop holds the running control loop's state: the driver/transport it
// was wired to in run, plus the mutable state each select branch in
// controlLoop updates (last-command time, currently-applied duty, whether
// the watchdog has already safety-stopped). Splitting this out of
// controlLoop's local variables keeps each branch a short method call
// instead of inline logic, which is what keeps controlLoop's own cognitive
// complexity low.
type motorLoop struct {
	logger *slog.Logger
	drv    *motor.Driver
	pub    *nats.Publisher[*actuationv1.MotorStatus]

	lastCmdAt   time.Time
	currentDuty float64
	stopped     bool
}

// ackermannCmdSubject/motorStatusSubject are the NATS subjects declared in
// ackermann_cmd.proto/motor_status.proto's own docstrings — not restated
// anywhere else in this module yet (internal/config/profile, which will own
// subject/topic config once built, is still an empty package).
const (
	ackermannCmdSubject = "vtitan.actuation.v1.ackermann_cmd"
	motorStatusSubject  = "vtitan.actuation.v1.motor_status"
)

// speedScalePercentPerMPS converts a commanded AckermannCmd.speed [m/s] into
// a motor duty percentage, matching motors.toml's `drive.speed_scale`
// (motor_speed = velocity_m_s * scale) — see
// platform/robot/config/hardware/motors/motors.toml. Restated here as a
// literal rather than read from a shared loader because
// internal/config/profile (the planned Go equivalent of that TOML file's
// per-component profile loading) doesn't exist yet; migrate this constant
// there once it does.
const speedScalePercentPerMPS = 30.0

// maxDutyPercent is motors.toml's `drive.max_speed`/`min_speed` magnitude:
// motor duty percentage is clamped to [-100, 100].
const maxDutyPercent = 100.0

// defaultCommandTimeout is the motor node's own deadline watchdog: how long
// it will keep driving the last commanded speed after the most recent
// AckermannCmd before treating the command stream as stale and safety-
// stopping. There's no prior Python value to port — the existing ROS2 stack
// relies on DDS DEADLINE QoS instead, which NATS has no equivalent for (see
// ackermann_cmd.proto) — so this is a first-derived value for the Go port,
// exposed as a flag rather than hardcoded so it can be tuned on real
// hardware.
const defaultCommandTimeout = 500 * time.Millisecond

// watchdogPollInterval is how often the control loop checks command
// staleness while no new AckermannCmd has arrived.
const watchdogPollInterval = 50 * time.Millisecond

// defaultNATSURL is nats-server's own default client address, matching the
// Pi 5's planned deployment (see go_nats_migration_plan.md's "Process
// model": nats-server runs on the Pi 5, reachable at pi5.local:4222 from
// the Pi Zero over the USB-gadget link).
const defaultNATSURL = "nats://127.0.0.1:4222"

// exit codes: 0 means motor-node ran and shut down cleanly (including via
// SIGINT/SIGTERM). 1 means it could not start or hit an unrecoverable
// runtime error.
const (
	exitOK    = 0
	exitError = 1
)

func newRootCmd(cfg *cliConfig, logger *slog.Logger) *cobra.Command {
	cmd := &cobra.Command{
		Use:   "motor-node",
		Short: "Drive the BTS7960 motor from AckermannCmd messages over NATS",
		Long: "motor-node subscribes to AckermannCmd on the vtitan.actuation.v1.ackermann_cmd\n" +
			"NATS subject, drives the BTS7960 motor accordingly, and publishes MotorStatus on\n" +
			"vtitan.actuation.v1.motor_status. It enforces its own command-deadline watchdog,\n" +
			"safety-stopping the drive if no AckermannCmd arrives within --command-timeout.",
		SilenceUsage:  true,
		SilenceErrors: true,
		RunE: func(cmd *cobra.Command, _ []string) error {
			return run(cmd.Context(), logger, *cfg)
		},
	}

	flags := cmd.Flags()
	flags.StringVar(&cfg.natsURL, "nats-url", defaultNATSURL, "nats-server URL")
	flags.StringVar(&cfg.nodeName, "name", "motor-node", "NATS client name, visible in nats-server's connz output")
	flags.DurationVar(&cfg.commandTimeout, "command-timeout", defaultCommandTimeout,
		"safety-stop the drive if no AckermannCmd arrives within this duration")
	flags.BoolVar(
		&cfg.invert, "invert", false,
		"flip SetSpeed's sign convention, matching motors.toml's drive.reversed",
	)

	return cmd
}

// speedToNormalized converts an AckermannCmd's speed [m/s] into the signed
// duty fraction [-1, 1] motor.Actuator.SetSpeed expects, per
// speedScalePercentPerMPS.
func speedToNormalized(speedMPS float32) float64 {
	percent := float64(speedMPS) * speedScalePercentPerMPS
	clamped := min(max(percent, -maxDutyPercent), maxDutyPercent)
	return clamped / maxDutyPercent
}

// motorStatusFor builds the MotorStatus to publish after applying a command
// or a watchdog safety-stop.
func motorStatusFor(dutyFraction float64, commandAge time.Duration, setSpeedErr error) *actuationv1.MotorStatus {
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
		FrameId:      "base_link",
		State:        state,
		Detail:       detail,
		DutyCycle:    float32(dutyFraction),
		CommandAgeMs: uint32(commandAge.Milliseconds()),
	}
}

// run wires the motor driver to NATS and blocks until ctx is done.
func run(ctx context.Context, logger *slog.Logger, cfg cliConfig) error {
	motorCfg := motor.DefaultConfig()
	motorCfg.Invert = cfg.invert

	drv, err := motor.New(motorCfg)
	if err != nil {
		return err //nolint:wrapcheck // motor.New already wraps with "motor: ..." context
	}
	if err = drv.Connect(ctx); err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "motor: ..." context
	}
	// motor.Actuator.Close has no context parameter -- it's a best-effort
	// teardown that must still run when ctx is already done (that's
	// exactly when this defer fires), so there's no ctx to thread through
	// here.
	defer func() { //nolint:contextcheck // Close has no ctx param, see comment above
		if closeErr := drv.Close(); closeErr != nil {
			logger.Error("motor-node: closing motor driver", "error", closeErr)
		}
	}()

	conn, err := nats.Connect(nats.DefaultConfig(cfg.natsURL, cfg.nodeName))
	if err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "nats: ..." context
	}
	defer conn.Close()

	sub, err := nats.NewSubscriber(conn, ackermannCmdSubject, func() *actuationv1.AckermannCmd {
		return &actuationv1.AckermannCmd{}
	})
	if err != nil {
		return err //nolint:wrapcheck // NewSubscriber already wraps with "nats: ..." context
	}
	defer func() {
		if closeErr := sub.Close(); closeErr != nil {
			logger.Error("motor-node: closing NATS subscription", "error", closeErr)
		}
	}()

	pub := nats.NewPublisher[*actuationv1.MotorStatus](conn, motorStatusSubject)

	logger.Info("motor-node: connected", "nats_url", cfg.natsURL, "command_timeout", cfg.commandTimeout)
	loop := &motorLoop{logger: logger, drv: drv, pub: pub, lastCmdAt: time.Now(), stopped: true}
	return loop.run(ctx, sub, cfg.commandTimeout)
}

// applyCommand drives cmd's speed and publishes the resulting MotorStatus.
func (l *motorLoop) applyCommand(ctx context.Context, cmd *actuationv1.AckermannCmd) {
	l.lastCmdAt = time.Now()
	l.currentDuty = speedToNormalized(cmd.GetSpeed())
	l.stopped = false

	setErr := l.drv.SetSpeed(ctx, l.currentDuty)
	if setErr != nil {
		l.logger.Error("motor-node: SetSpeed", "error", setErr)
	}
	if pubErr := l.pub.Publish(motorStatusFor(l.currentDuty, 0, setErr)); pubErr != nil {
		l.logger.Error("motor-node: publishing MotorStatus", "error", pubErr)
	}
}

// checkWatchdog safety-stops the drive if no AckermannCmd has arrived within
// commandTimeout, and is a no-op otherwise (including once it has already
// stopped for this staleness episode, so it doesn't republish FAULT/IDLE
// status on every single poll tick).
func (l *motorLoop) checkWatchdog(ctx context.Context, commandTimeout time.Duration) {
	age := time.Since(l.lastCmdAt)
	if age < commandTimeout || l.stopped {
		return
	}

	l.stopped = true
	l.currentDuty = 0
	setErr := l.drv.SetSpeed(ctx, l.currentDuty)
	if setErr != nil {
		l.logger.Error("motor-node: safety-stop SetSpeed", "error", setErr)
	}
	l.logger.Warn("motor-node: command timeout, safety-stopping drive", "age", age)
	if pubErr := l.pub.Publish(motorStatusFor(l.currentDuty, age, setErr)); pubErr != nil {
		l.logger.Error("motor-node: publishing MotorStatus", "error", pubErr)
	}
}

// run applies each incoming AckermannCmd and enforces the command-deadline
// watchdog until ctx is done.
func (l *motorLoop) run(
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

func main() {
	os.Exit(mainWithExitCode())
}

// mainWithExitCode does the real work of main and returns the process exit
// code instead of calling os.Exit directly, so `defer` cleanup (closing the
// motor driver, the NATS connection, releasing the signal.NotifyContext)
// actually runs before the process exits.
func mainWithExitCode() int {
	logger := slog.New(slog.NewTextHandler(os.Stderr, nil))

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	var cfg cliConfig
	cmd := newRootCmd(&cfg, logger)
	cmd.SetArgs(os.Args[1:])

	if err := cmd.ExecuteContext(ctx); err != nil {
		logger.Error("motor-node run failed", "error", err)
		return exitError
	}
	return exitOK
}
