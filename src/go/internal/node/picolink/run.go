package picolink

import (
	"context"
	"fmt"
	"io"
	"log/slog"
	"time"

	"go.bug.st/serial"

	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
	"github.com/teamvoltimor/vtitan/src/go/pkg/transport/nats"
)

// Config wires a Session to its serial port and NATS connection.
type Config struct {
	// Port is the board's serial device, e.g. /dev/ttyACM1 for USB CDC.
	Port string
	// NATS is the connection the actuation subjects are served on.
	NATS nats.Config
	// Session is the board configuration and link timing.
	Session SessionConfig
}

const (
	// serialBaudRate is ignored by USB CDC, which runs at USB speed, but
	// sets the line rate if the board is wired over a UART instead.
	serialBaudRate = 115200
	// serialReadTimeout bounds each port read, so the link reader notices
	// cancellation instead of blocking until the next byte.
	serialReadTimeout = 100 * time.Millisecond
)

// SessionConfigFor resolves a SessionConfig from configRoot and the active
// hardware profile (LoadProfile, then BoardConfig), with the Pi 5's
// equivalents of the Zero's --motor-invert and --motor-command-timeout.
func SessionConfigFor(
	logger *slog.Logger,
	configRoot string,
	invertDrive bool,
	commandTimeout time.Duration,
) (SessionConfig, error) {
	p, err := LoadProfile(logger, configRoot)
	if err != nil {
		return SessionConfig{}, err
	}
	board, err := BoardConfig(p, invertDrive, commandTimeout)
	if err != nil {
		return SessionConfig{}, err
	}
	return SessionConfig{Board: board, Encoder: p.Encoder}, nil
}

// Run opens the serial port and serves the link until ctx is done or the
// link fails.
//
// Run owns the port, as internal/node/lidar's Run does, so a supervised
// restart reopens it rather than inheriting a half-dead one. An open failure
// (board unplugged, wrong path) is returned, and pkg/supervise retries with
// backoff; a board that reappears is picked up on the next attempt.
func Run(ctx context.Context, cfg Config, logger *slog.Logger) error {
	port, err := serial.Open(cfg.Port, &serial.Mode{BaudRate: serialBaudRate})
	if err != nil {
		return fmt.Errorf("picolink: opening serial port %s: %w", cfg.Port, err)
	}
	defer func() {
		if closeErr := port.Close(); closeErr != nil {
			logger.Error("picolink: closing serial port", "port", cfg.Port, "error", closeErr)
		}
	}()
	if err = port.SetReadTimeout(serialReadTimeout); err != nil {
		return fmt.Errorf("picolink: setting read timeout on %s: %w", cfg.Port, err)
	}
	// Bytes queued before this session opened the port belong to no one;
	// the decoder would drop them anyway, this just avoids counting them.
	if err = port.ResetInputBuffer(); err != nil {
		return fmt.Errorf("picolink: purging stale input on %s: %w", cfg.Port, err)
	}

	logger.Info("picolink: serial port open", "port", cfg.Port)
	return Serve(ctx, cfg, logger, port)
}

// Serve connects NATS and runs a Session over an already-open link: it
// subscribes vtitan.actuation.v1.ackermann_cmd and publishes
// vtitan.actuation.v1.motor_status and, when an encoder is configured,
// vtitan.actuation.v1.joint_states, the subjects the Zero's motor node
// serves. cfg.Port is not used.
func Serve(ctx context.Context, cfg Config, logger *slog.Logger, link io.ReadWriter) error {
	conn, err := nats.Connect(ctx, cfg.NATS)
	if err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "nats: ..." context
	}
	defer conn.Close()

	sub, err := nats.NewSubscriber[actuationv1.AckermannCmd](conn, actuationv1.AckermannCmdSubject)
	if err != nil {
		return err
	}
	defer func() {
		if closeErr := sub.Close(); closeErr != nil {
			logger.Error("picolink: closing AckermannCmd subscription", "error", closeErr)
		}
	}()

	// The subscription must be live on the server before the session starts
	// and tells the board it is being served.
	if err = conn.Flush(); err != nil {
		return fmt.Errorf("picolink: flushing the AckermannCmd subscription: %w", err)
	}

	var joints JointStatesPublisher
	if cfg.Session.Encoder != nil {
		joints = nats.NewPublisher[*actuationv1.JointStates](conn, actuationv1.JointStatesSubject)
	}
	session, err := NewSession(
		logger,
		cfg.Session,
		nats.NewPublisher[*actuationv1.MotorStatus](conn, actuationv1.MotorStatusSubject),
		joints,
	)
	if err != nil {
		return err
	}

	logger.Info("picolink: serving actuation subjects", "nats_url", cfg.NATS.URL,
		"odometry", cfg.Session.Encoder != nil)
	return session.Run(ctx, link, sub)
}
