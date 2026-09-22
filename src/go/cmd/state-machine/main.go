//go:build linux

// Command state-machine is a bench/dev single-subsystem binary for the
// robot state machine, sharing the same internal packages as cmd/pi5 — used
// for isolated bench testing and local debugging without the whole board
// binary.
//
// It dials the backend's RobotCommandService (other/contracts/proto/telemetry/v1/
// commands.proto) via internal/statemachine/robotcmd, dispatches every
// received command through internal/statemachine/command.Dispatcher, and
// publishes synthetic button events on the vtitan.ui.v1.button_event NATS
// subject for start/stop/emergency-stop commands -- see
// internal/node/statemachine's doc comment for what's wired and what
// isn't yet.
package main

import (
	"context"
	"log/slog"
	"os"
	"os/signal"
	"syscall"

	"github.com/spf13/cobra"

	"github.com/teamvoltimor/vtitan/src/go/internal/cmdkit"
	"github.com/teamvoltimor/vtitan/src/go/internal/node/statemachine"
	"github.com/teamvoltimor/vtitan/src/go/internal/transport/nats"
)

// cliConfig holds every flag state-machine accepts.
type cliConfig struct {
	cmdkit.Common

	backendAddr string
	robotID     string
}

// exit codes: 0 means state-machine ran and shut down cleanly (including
// via SIGINT/SIGTERM). 1 means it could not start or hit an unrecoverable
// runtime error.
const (
	exitOK    = 0
	exitError = 1
)

func newRootCmd(cfg *cliConfig, logger *slog.Logger) *cobra.Command {
	cmd := &cobra.Command{
		Use:   "state-machine",
		Short: "Dispatch backend RobotCommands and publish synthetic button events over NATS",
		Long: "state-machine dials the backend's RobotCommandService, dispatches every\n" +
			"received command through internal/statemachine/command.Dispatcher, and\n" +
			"publishes synthetic button events on vtitan.ui.v1.button_event for\n" +
			"start/stop/emergency-stop commands. Backend-channel administrative\n" +
			"commands (vision debug, telemetry channel toggle, command-channel\n" +
			"disable) are not implemented yet and report StatusFailed.",
		SilenceUsage:  true,
		SilenceErrors: true,
		RunE: func(cmd *cobra.Command, _ []string) error {
			return run(cmd.Context(), logger, *cfg)
		},
	}

	flags := cmd.Flags()
	cfg.RegisterNATSURL(flags)
	cfg.RegisterNodeName(flags, "state-machine")
	flags.StringVar(
		&cfg.backendAddr,
		"backend-addr",
		statemachine.DefaultBackendAddr,
		"backend gRPC address (host:port)",
	)
	flags.StringVar(
		&cfg.robotID,
		"robot-id",
		"",
		"robot ID to identify as on the backend command channel",
	)
	// MarkFlagRequired only errors for a flag name that doesn't exist on
	// cmd, which "robot-id" always does -- it's registered immediately
	// above.
	if err := cmd.MarkFlagRequired("robot-id"); err != nil {
		panic(err)
	}

	return cmd
}

// run hands this binary's flags to the shared loop in
// internal/node/statemachine -- the same loop cmd/pi5 supervises, so bench
// runs and board runs cannot drift apart.
func run(ctx context.Context, logger *slog.Logger, cfg cliConfig) error {
	//nolint:wrapcheck // statemachine.Run's errors already carry their own package prefix
	return statemachine.Run(ctx, statemachine.Config{
		NATS:        nats.DefaultConfig(cfg.NATSURL, cfg.NodeName),
		BackendAddr: cfg.backendAddr,
		RobotID:     cfg.robotID,
	}, logger)
}

func main() {
	os.Exit(mainWithExitCode())
}

// mainWithExitCode does the real work of main and returns the process exit
// code instead of calling os.Exit directly, so `defer` cleanup (closing the
// backend connection, the NATS connection, releasing the
// signal.NotifyContext) actually runs before the process exits.
func mainWithExitCode() int {
	logger := slog.New(slog.NewTextHandler(os.Stderr, nil))

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	var cfg cliConfig
	cmd := newRootCmd(&cfg, logger)
	cmd.SetArgs(os.Args[1:])

	if err := cmd.ExecuteContext(ctx); err != nil {
		logger.Error("state-machine run failed", "error", err)
		return exitError
	}
	return exitOK
}
