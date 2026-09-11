//go:build linux

// Command motor-node is a bench/dev single-subsystem binary for the BTS7960
// motor control loop, sharing internal/node/motor's control loop with
// cmd/pi-zero — used for isolated hardware bench testing and local
// debugging.
//
// It subscribes to AckermannCmd on the `vtitan.actuation.v1.ackermann_cmd`
// NATS subject and publishes MotorStatus on
// `vtitan.actuation.v1.motor_status`, enforcing its own command-deadline
// watchdog (see ackermann_cmd.proto's docstring: NATS has no DDS DEADLINE
// QoS equivalent, so the motor node must detect a stale command itself and
// stop the drive rather than keep applying the last one it heard).
//
// When a wheel encoder is configured (see internal/driver/encoder) it also
// publishes JointStates on `vtitan.actuation.v1.joint_states`, the wheel
// odometry natsgw.Gateway feeds to bay exit.
package main

import (
	"context"
	"fmt"
	"log/slog"
	"os"
	"os/signal"
	"syscall"
	"time"

	natsconn "github.com/nats-io/nats.go"
	"github.com/spf13/cobra"

	"github.com/teamvoltimor/vtitan/src/go/internal/driver/encoder"
	"github.com/teamvoltimor/vtitan/src/go/internal/driver/motor"
	nodemotor "github.com/teamvoltimor/vtitan/src/go/internal/node/motor"
	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
	"github.com/teamvoltimor/vtitan/src/go/internal/transport/nats"
)

// cliConfig holds every flag motor-node accepts.
type cliConfig struct {
	natsURL        string
	nodeName       string
	commandTimeout time.Duration
	invert         bool
	configRoot     string
}

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
	flags.StringVar(&cfg.natsURL, "nats-url", nats.DefaultURL(), "nats-server URL")
	flags.StringVar(
		&cfg.nodeName,
		"name",
		"motor-node",
		"NATS client name, visible in nats-server's connz output",
	)
	flags.DurationVar(&cfg.commandTimeout, "command-timeout", nodemotor.DefaultCommandTimeout,
		"safety-stop the drive if no AckermannCmd arrives within this duration")
	flags.BoolVar(
		&cfg.invert, "invert", false,
		"flip SetSpeed's sign convention, matching motors.toml's drive.reversed",
	)
	flags.StringVar(&cfg.configRoot, "config-root", "",
		"repo root to load the hardware profile (VTITAN_HARDWARE_PROFILE) from; "+
			"empty uses nodemotor.DefaultSpeedScalePercentPerMPS")

	return cmd
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

	conn, err := nats.Connect(ctx, nats.DefaultConfig(cfg.natsURL, cfg.nodeName))
	if err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "nats: ..." context
	}
	defer conn.Close()

	sub, err := nats.NewSubscriber(
		conn,
		actuationv1.AckermannCmdSubject,
		func() *actuationv1.AckermannCmd {
			return &actuationv1.AckermannCmd{}
		},
	)
	if err != nil {
		return err //nolint:wrapcheck // NewSubscriber already wraps with "nats: ..." context
	}
	defer func() {
		if closeErr := sub.Close(); closeErr != nil {
			logger.Error("motor-node: closing NATS subscription", "error", closeErr)
		}
	}()

	pub := nats.NewPublisher[*actuationv1.MotorStatus](conn, actuationv1.MotorStatusSubject)

	logger.Info(
		"motor-node: connected",
		"nats_url",
		cfg.natsURL,
		"command_timeout",
		cfg.commandTimeout,
	)

	stopFeedback, err := startEncoderFeedback(ctx, logger, conn, cfg.configRoot)
	if err != nil {
		return err
	}
	defer stopFeedback()

	loop := nodemotor.NewLoop(logger, drv, pub, nodemotor.SpeedScaleFor(logger, cfg.configRoot))
	if err = loop.Run(ctx, sub, cfg.commandTimeout); err != nil {
		return fmt.Errorf("motor-node: %w", err)
	}
	return nil
}

// startEncoderFeedback connects the wheel encoder and starts publishing
// JointStates, returning the teardown to defer.
//
// A missing or unloadable encoder config is NOT fatal: motor-node is a
// bench binary routinely run with no --config-root and no motor profile
// active, and refusing to drive the motor because odometry is unavailable
// would break the bench workflow this binary exists for. It logs why and
// runs without odometry, which is exactly the state natsgw.Gateway's
// ok=false already models. A configured encoder that then fails to CONNECT
// is fatal, since that is a wiring fault the operator asked us to use.
func startEncoderFeedback(
	ctx context.Context,
	logger *slog.Logger,
	conn *natsconn.Conn,
	configRoot string,
) (func(), error) {
	encCfg, err := encoder.ConfigFor(configRoot)
	if err != nil {
		logger.Warn("motor-node: no wheel encoder configured, not publishing joint_states",
			"error", err)
		return func() {}, nil
	}

	enc, err := encoder.New(encCfg)
	if err != nil {
		return nil, fmt.Errorf("motor-node: %w", err)
	}
	if err = enc.Connect(ctx); err != nil {
		return nil, fmt.Errorf("motor-node: %w", err)
	}

	jointPub := nats.NewPublisher[*actuationv1.JointStates](conn, actuationv1.JointStatesSubject)
	feedback := nodemotor.NewFeedback(logger, enc, jointPub, nodemotor.DefaultFeedbackInterval)

	done := make(chan struct{})
	go func() {
		defer close(done)
		if runErr := feedback.Run(ctx); runErr != nil {
			logger.Error("motor-node: encoder feedback loop", "error", runErr)
		}
	}()
	logger.Info("motor-node: publishing wheel odometry",
		"subject", actuationv1.JointStatesSubject,
		"pin_a", encCfg.PinA, "pin_b", encCfg.PinB,
		"counts_per_rev", encCfg.CountsPerRev)

	return func() {
		<-done
		if closeErr := enc.Close(); closeErr != nil {
			logger.Error("motor-node: closing wheel encoder", "error", closeErr)
		}
	}, nil
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
