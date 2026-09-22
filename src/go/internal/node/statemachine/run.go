package statemachine

import (
	"context"
	"fmt"
	"log/slog"

	uiv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/ui/v1"
	"github.com/teamvoltimor/vtitan/src/go/internal/statemachine/command"
	"github.com/teamvoltimor/vtitan/src/go/internal/statemachine/robotcmd"
	"github.com/teamvoltimor/vtitan/src/go/internal/transport/nats"
)

// Config wires the NATS connection synthetic button events go out on and the
// backend command channel this node identifies itself on.
type Config struct {
	NATS        nats.Config
	BackendAddr string
	RobotID     string
}

// DefaultBackendAddr matches other/apps/backend/cmd/server's default
// --grpc-addr for local bench/dev use against a backend running on the
// same machine.
const DefaultBackendAddr = "127.0.0.1:50051"

// Run dials the backend's RobotCommandService, dispatches every received
// command through command.Dispatcher, and publishes synthetic button events
// on vtitan.ui.v1.button_event, until ctx is done or the command channel hits
// an unrecoverable error.
//
// Shared by cmd/state-machine (bench/dev, standalone) and cmd/pi5, which
// supervises it. Run owns the NATS connection and the backend client, so a
// supervised restart redials both rather than inheriting a dead channel --
// the backend going away and coming back is the expected case here, not an
// exceptional one.
func Run(ctx context.Context, cfg Config, logger *slog.Logger) error {
	conn, err := nats.Connect(ctx, cfg.NATS)
	if err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "nats: ..." context
	}
	defer conn.Close()

	buttonPub := nats.NewPublisher[*uiv1.ButtonEvent](conn, uiv1.ButtonEventSubject)
	dispatcher := command.NewDispatcher(
		NewNATSButtonSink(buttonPub),
		UnimplementedChannelSink{},
	)

	client, err := robotcmd.New(
		robotcmd.Config{Addr: cfg.BackendAddr, RobotID: cfg.RobotID},
		logger,
	)
	if err != nil {
		return err //nolint:wrapcheck // robotcmd.New already wraps with "robotcmd: ..." context
	}
	defer func() {
		if closeErr := client.Close(); closeErr != nil {
			logger.Error("statemachine: closing backend connection", "error", closeErr)
		}
	}()

	logger.Info("statemachine: connected",
		"nats_url", cfg.NATS.URL, "backend_addr", cfg.BackendAddr, "robot_id", cfg.RobotID)
	if err = client.Run(ctx, dispatcher); err != nil {
		return fmt.Errorf("statemachine: %w", err)
	}
	return nil
}
