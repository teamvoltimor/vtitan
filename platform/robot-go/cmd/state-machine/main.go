//go:build linux

// Command state-machine is a bench/dev single-subsystem binary for the
// robot state machine, sharing the same internal packages as cmd/pi5 — used
// for isolated bench testing and local debugging without the whole board
// binary.
//
// It dials the backend's RobotCommandService (platform/proto/telemetry/v1/
// commands.proto) via internal/statemachine/robotcmd, dispatches every
// received command through internal/statemachine/command.Dispatcher, and
// publishes synthetic button events on the vtitan.ui.v1.button_event NATS
// subject for start/stop/emergency-stop commands -- see
// internal/node/statemachine's doc comment for what's wired and what
// isn't yet.
package main

import (
	"context"
	"fmt"
	"log/slog"
	"os"
	"os/signal"
	"syscall"

	"github.com/spf13/cobra"

	nodestatemachine "github.com/teamvoltimor/vtitan/platform/robot-go/internal/node/statemachine"
	uiv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/ui/v1"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/statemachine/command"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/statemachine/robotcmd"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/transport/nats"
)

// cliConfig holds every flag state-machine accepts.
type cliConfig struct {
	natsURL     string
	nodeName    string
	backendAddr string
	robotID     string
}

// defaultBackendAddr matches platform/backend/cmd/server's default
// --grpc-addr for local bench/dev use against a backend running on the
// same machine.
const defaultBackendAddr = "127.0.0.1:50051"

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
	flags.StringVar(&cfg.natsURL, "nats-url", nats.DefaultDevURL, "nats-server URL")
	flags.StringVar(&cfg.nodeName, "name", "state-machine", "NATS client name, visible in nats-server's connz output")
	flags.StringVar(&cfg.backendAddr, "backend-addr", defaultBackendAddr, "backend gRPC address (host:port)")
	flags.StringVar(&cfg.robotID, "robot-id", "", "robot ID to identify as on the backend command channel")
	// MarkFlagRequired only errors for a flag name that doesn't exist on
	// cmd, which "robot-id" always does -- it's registered immediately
	// above.
	if err := cmd.MarkFlagRequired("robot-id"); err != nil {
		panic(err)
	}

	return cmd
}

// run wires the robotcmd client to a NATS-backed ButtonSink and blocks
// until ctx is done or the command channel hits an unrecoverable error.
func run(ctx context.Context, logger *slog.Logger, cfg cliConfig) error {
	conn, err := nats.Connect(nats.DefaultConfig(cfg.natsURL, cfg.nodeName))
	if err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "nats: ..." context
	}
	defer conn.Close()

	buttonPub := nats.NewPublisher[*uiv1.ButtonEvent](conn, uiv1.ButtonEventSubject)
	dispatcher := command.NewDispatcher(
		nodestatemachine.NewNATSButtonSink(buttonPub),
		nodestatemachine.UnimplementedChannelSink{},
	)

	client, err := robotcmd.New(robotcmd.Config{Addr: cfg.backendAddr, RobotID: cfg.robotID}, logger)
	if err != nil {
		return err //nolint:wrapcheck // robotcmd.New already wraps with "robotcmd: ..." context
	}
	defer func() {
		if closeErr := client.Close(); closeErr != nil {
			logger.Error("state-machine: closing backend connection", "error", closeErr)
		}
	}()

	logger.Info("state-machine: connected",
		"nats_url", cfg.natsURL, "backend_addr", cfg.backendAddr, "robot_id", cfg.robotID)
	if err = client.Run(ctx, dispatcher); err != nil {
		return fmt.Errorf("state-machine: %w", err)
	}
	return nil
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
