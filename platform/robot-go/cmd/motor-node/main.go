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
package main

import (
	"context"
	"fmt"
	"log/slog"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/spf13/cobra"

	motordriver "github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/motor"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/node/motor"
	actuationv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/actuation/v1"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/transport/nats"
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
	flags.StringVar(&cfg.natsURL, "nats-url", nats.DefaultDevURL, "nats-server URL")
	flags.StringVar(&cfg.nodeName, "name", "motor-node", "NATS client name, visible in nats-server's connz output")
	flags.DurationVar(&cfg.commandTimeout, "command-timeout", motor.DefaultCommandTimeout,
		"safety-stop the drive if no AckermannCmd arrives within this duration")
	flags.BoolVar(
		&cfg.invert, "invert", false,
		"flip SetSpeed's sign convention, matching motors.toml's drive.reversed",
	)
	flags.StringVar(&cfg.configRoot, "config-root", "",
		"repo root to load the hardware profile (VTITAN_HARDWARE_PROFILE) from; "+
			"empty uses motor.DefaultSpeedScalePercentPerMPS")

	return cmd
}

// run wires the motor driver to NATS and blocks until ctx is done.
func run(ctx context.Context, logger *slog.Logger, cfg cliConfig) error {
	motorCfg := motordriver.DefaultConfig()
	motorCfg.Invert = cfg.invert

	drv, err := motordriver.New(motorCfg)
	if err != nil {
		return err //nolint:wrapcheck // motordriver.New already wraps with "motor: ..." context
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

	sub, err := nats.NewSubscriber(conn, actuationv1.AckermannCmdSubject, func() *actuationv1.AckermannCmd {
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

	pub := nats.NewPublisher[*actuationv1.MotorStatus](conn, actuationv1.MotorStatusSubject)

	logger.Info("motor-node: connected", "nats_url", cfg.natsURL, "command_timeout", cfg.commandTimeout)
	loop := motor.NewLoop(logger, drv, pub, motor.SpeedScaleFor(logger, cfg.configRoot))
	if err = loop.Run(ctx, sub, cfg.commandTimeout); err != nil {
		return fmt.Errorf("motor-node: %w", err)
	}
	return nil
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
