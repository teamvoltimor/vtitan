package robotcmd

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"time"

	"google.golang.org/grpc"
	"google.golang.org/grpc/credentials/insecure"

	"github.com/teamvoltimor/vtitan/src/go/internal/statemachine/backoff"
	"github.com/teamvoltimor/vtitan/src/go/internal/statemachine/command"

	telemetryv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/telemetry/v1"
)

// Config parameterizes a Client's connection to the backend's
// RobotCommandService.
type Config struct {
	// Addr is the backend gRPC address (host:port).
	Addr string
	// RobotID identifies this robot to StreamCommands/AckCommand, matching
	// commands.proto's robot_id field on both RPCs.
	RobotID string
	// Backoff parameterizes the reconnect delay between StreamCommands
	// attempts. Defaults to backoff.DefaultConfig() (matching
	// grpc_backoff.py) if zero-valued.
	Backoff backoff.Config
}

// Client is the gRPC client half of the backend command channel (see
// doc.go). The connection is unauthenticated (grpc.WithTransportCredentials
// (insecure.NewCredentials())) -- matching platform/backend's
// RobotCommandServiceServer today, which has no auth/TLS interceptor in
// its chain (cmd/server/main.go); this client tracks that, it doesn't
// invent a stronger contract the server doesn't enforce.
type Client struct {
	cfg    Config
	logger *slog.Logger
	conn   *grpc.ClientConn
	rpc    telemetryv1.RobotCommandServiceClient
}

// New dials cfg.Addr and returns a Client. The dial is non-blocking
// (grpc.NewClient does not connect eagerly); Run's stream loop is what
// actually establishes and maintains the connection.
func New(cfg Config, logger *slog.Logger) (*Client, error) {
	if cfg.Backoff == (backoff.Config{}) {
		cfg.Backoff = backoff.DefaultConfig()
	}

	conn, err := grpc.NewClient(cfg.Addr, grpc.WithTransportCredentials(insecure.NewCredentials()))
	if err != nil {
		return nil, fmt.Errorf("robotcmd: dialing %s: %w", cfg.Addr, err)
	}

	return NewFromConn(conn, cfg, logger), nil
}

// NewFromConn builds a Client over an already-established conn, skipping
// New's grpc.NewClient(cfg.Addr, ...) dial -- for tests that need to inject
// an in-memory connection (e.g. google.golang.org/grpc/test/bufconn)
// instead of a real network dial. cfg.Addr is unused in this path.
func NewFromConn(conn *grpc.ClientConn, cfg Config, logger *slog.Logger) *Client {
	if cfg.Backoff == (backoff.Config{}) {
		cfg.Backoff = backoff.DefaultConfig()
	}
	return &Client{cfg: cfg, logger: logger, conn: conn, rpc: telemetryv1.NewRobotCommandServiceClient(conn)}
}

// Close releases the underlying gRPC connection.
func (c *Client) Close() error {
	if err := c.conn.Close(); err != nil {
		return fmt.Errorf("robotcmd: closing connection: %w", err)
	}
	return nil
}

// Run opens StreamCommands, dispatches every command it receives through
// dispatcher, and acks each outcome, reconnecting with exponential backoff
// (matching command_channel.py's own reconnect loop) until ctx is done.
// The stream resumes from the last successfully-received command's ID
// across reconnects, so the backend's replay-on-reconnect (see
// commands.proto's StreamCommandsRequest.last_command_id) doesn't
// redeliver commands this process already saw.
func (c *Client) Run(ctx context.Context, dispatcher *command.Dispatcher) error {
	bo := backoff.New(c.cfg.Backoff)
	var lastCommandID string

	for {
		if ctx.Err() != nil {
			return nil //nolint:nilerr // ctx cancellation is a clean shutdown, not a failure to report
		}

		err := c.streamOnce(ctx, dispatcher, &lastCommandID)
		if err == nil || errors.Is(err, context.Canceled) {
			return nil
		}

		c.logger.Warn("robotcmd: command stream ended, reconnecting", "error", err)
		delay := bo.Next()
		select {
		case <-ctx.Done():
			return nil
		case <-time.After(delay):
		}
	}
}

// streamOnce opens one StreamCommands call and processes messages until it
// ends (backend closes it, a newer connection for this robot_id supersedes
// it server-side, or ctx is done). Returns nil only when ctx is done first
// -- any other stream termination is reported as an error so Run retries.
func (c *Client) streamOnce(ctx context.Context, dispatcher *command.Dispatcher, lastCommandID *string) error {
	req := &telemetryv1.StreamCommandsRequest{RobotId: c.cfg.RobotID}
	if *lastCommandID != "" {
		req.LastCommandId = lastCommandID
	}

	stream, err := c.rpc.StreamCommands(ctx, req)
	if err != nil {
		return fmt.Errorf("robotcmd: opening StreamCommands: %w", err)
	}
	c.logger.Info("robotcmd: command stream connected", "robot_id", c.cfg.RobotID)

	for {
		rc, recvErr := stream.Recv()
		if recvErr != nil {
			if ctx.Err() != nil {
				return nil //nolint:nilerr // ctx cancellation is a clean shutdown, not a failure to report
			}
			return fmt.Errorf("robotcmd: receiving RobotCommand: %w", recvErr)
		}

		status, message := dispatcher.Dispatch(toCommand(rc))
		c.ack(ctx, rc.GetCommandId(), status, message)
		*lastCommandID = rc.GetCommandId()
	}
}

// ack reports a command's outcome via AckCommand. Failures are logged, not
// returned -- commands.proto's own docstring and
// RobotCommandServiceServer.AckCommand both treat acking as fire-and-forget
// from the robot's perspective (a failed ack must never block command
// execution, which has already happened by the time ack is called).
func (c *Client) ack(ctx context.Context, commandID string, status command.Status, message string) {
	if status == command.StatusUnspecified {
		c.logger.Warn("robotcmd: Dispatch returned StatusUnspecified, not acking", "command_id", commandID)
		return
	}

	_, err := c.rpc.AckCommand(ctx, &telemetryv1.AckCommandRequest{
		RobotId:   c.cfg.RobotID,
		CommandId: commandID,
		Status:    toProtoStatus(status),
		Message:   message,
	})
	if err != nil {
		c.logger.Warn("robotcmd: AckCommand failed", "command_id", commandID, "error", err)
	}
}
