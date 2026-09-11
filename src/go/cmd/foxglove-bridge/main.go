// Command foxglove-bridge is the rviz2 replacement's live-visualization
// half: a Foxglove WebSocket protocol server (internal/foxglove) that
// republishes every known NATS/protobuf subject so Foxglove Studio can
// connect and render them, matching
// docs/internal/plans/go-migration-plan.md's "Visualization" row. The
// offline half -- recorded .mcap bags Foxglove Studio opens directly, no
// bridge needed -- already works via internal/recording.
package main

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"net/http"
	"os"
	"os/signal"
	"syscall"
	"time"

	natsio "github.com/nats-io/nats.go"
	"github.com/spf13/cobra"
	"golang.org/x/sync/errgroup"
	"google.golang.org/protobuf/proto"

	"github.com/teamvoltimor/vtitan/src/go/internal/cmdkit"
	"github.com/teamvoltimor/vtitan/src/go/internal/foxglove"
	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
	navv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/nav/v1"
	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
	statev1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/state/v1"
	uiv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/ui/v1"
	visionv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/vision/v1"
	"github.com/teamvoltimor/vtitan/src/go/internal/transport/nats"
)

type cliConfig struct {
	cmdkit.Common

	httpAddr string
}

// natsConn is the concrete *nats.Conn type from the nats-io client library
// -- named here only to keep bridgeSubject's signature readable without
// repeating the fully qualified import alias at every call site.
type natsConn = natsio.Conn

// defaultHTTPAddr matches Foxglove Studio's own "Open connection" default
// port for a custom WebSocket URL (ws://localhost:8765), so a fresh
// install needs no configuration beyond picking that connection type.
const defaultHTTPAddr = ":8765"

// exit codes: 0 means foxglove-bridge ran and shut down cleanly (including
// via SIGINT/SIGTERM). 1 means it could not start or hit an unrecoverable
// runtime error.
const (
	exitOK    = 0
	exitError = 1
)

// shutdownTimeout bounds how long the HTTP server gets to drain in-flight
// Foxglove connections after the context is canceled.
const shutdownTimeout = 5 * time.Second

func newRootCmd(cfg *cliConfig, logger *slog.Logger) *cobra.Command {
	cmd := &cobra.Command{
		Use:   "foxglove-bridge",
		Short: "Bridge every known NATS/protobuf subject to a live Foxglove WebSocket server",
		Long: "foxglove-bridge subscribes to every subject declared in internal/schema/pb and\n" +
			"republishes each one over the Foxglove WebSocket protocol, so Foxglove Studio can\n" +
			"connect (Open connection -> Foxglove WebSocket -> ws://<host>:8765) and render them\n" +
			"live -- the rviz2 replacement.",
		SilenceUsage:  true,
		SilenceErrors: true,
		RunE: func(cmd *cobra.Command, _ []string) error {
			return run(cmd.Context(), logger, *cfg)
		},
	}

	flags := cmd.Flags()
	cfg.RegisterNATSURL(flags)
	cfg.RegisterNodeName(flags, "foxglove-bridge")
	flags.StringVar(&cfg.httpAddr, "http-addr", defaultHTTPAddr, "address to serve the Foxglove WebSocket protocol on")

	return cmd
}

func run(ctx context.Context, logger *slog.Logger, cfg cliConfig) error {
	conn, err := nats.Connect(ctx, nats.DefaultConfig(cfg.NATSURL, cfg.NodeName))
	if err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "nats: ..." context
	}
	defer conn.Close()

	server := foxglove.NewServer(logger)

	httpServer := &http.Server{Addr: cfg.httpAddr, Handler: server.Handler()}
	group, gctx := errgroup.WithContext(ctx)

	if bridgeErr := bridgeAllSubjects(gctx, group, conn, logger, server); bridgeErr != nil {
		return bridgeErr
	}

	group.Go(func() error {
		logger.Info("foxglove-bridge: serving", "addr", cfg.httpAddr, "nats_url", cfg.NATSURL)
		if serveErr := httpServer.ListenAndServe(); serveErr != nil && !errors.Is(serveErr, http.ErrServerClosed) {
			return fmt.Errorf("foxglove-bridge: http server: %w", serveErr)
		}
		return nil
	})
	group.Go(func() error {
		<-gctx.Done()
		shutdownCtx, cancel := context.WithTimeout(context.WithoutCancel(ctx), shutdownTimeout)
		defer cancel()
		if shutdownErr := httpServer.Shutdown(shutdownCtx); shutdownErr != nil {
			return fmt.Errorf("foxglove-bridge: shutting down http server: %w", shutdownErr)
		}
		return nil
	})

	if err = group.Wait(); err != nil {
		return err //nolint:wrapcheck // each goroutine's own error is already package-prefixed
	}
	return nil
}

// bridgeAllSubjects registers and starts forwarding every subject declared
// across internal/schema/pb. Adding a new .proto/subject to this repo
// means adding one line here -- there is no reflection-based subject
// discovery, matching this tree's stated preference for an explicit,
// compile-time-checked table over a dynamic one (see
// feedback_go_code_style_maps_config_constants).
func bridgeAllSubjects(
	ctx context.Context, group *errgroup.Group, conn *natsConn, logger *slog.Logger, server *foxglove.Server,
) error {
	if err := bridgeSubject[actuationv1.AckermannCmd](ctx, group, conn, logger, server,
		actuationv1.AckermannCmdSubject); err != nil {
		return err
	}
	if err := bridgeSubject[actuationv1.MotorStatus](ctx, group, conn, logger, server,
		actuationv1.MotorStatusSubject); err != nil {
		return err
	}
	if err := bridgeSubject[navv1.NavigatorDebug](ctx, group, conn, logger, server,
		navv1.NavigatorDebugSubject); err != nil {
		return err
	}
	if err := bridgeSubject[navv1.LapsCompleted](ctx, group, conn, logger, server,
		navv1.LapsCompletedSubject); err != nil {
		return err
	}
	if err := bridgeSubject[navv1.CurrentCorridor](ctx, group, conn, logger, server,
		navv1.CurrentCorridorSubject); err != nil {
		return err
	}
	if err := bridgeSubject[sensorv1.Imu](ctx, group, conn, logger, server,
		sensorv1.ImuSubject); err != nil {
		return err
	}
	if err := bridgeSubject[sensorv1.Scan](ctx, group, conn, logger, server,
		sensorv1.ScanSubject); err != nil {
		return err
	}
	if err := bridgeSubject[sensorv1.CameraFrame](ctx, group, conn, logger, server,
		sensorv1.CameraSubject); err != nil {
		return err
	}
	if err := bridgeSubject[statev1.RobotState](ctx, group, conn, logger, server,
		statev1.RobotStateSubject); err != nil {
		return err
	}
	if err := bridgeSubject[statev1.RaceMetrics](ctx, group, conn, logger, server,
		statev1.RaceMetricsSubject); err != nil {
		return err
	}
	if err := bridgeSubject[statev1.SystemStatus](ctx, group, conn, logger, server,
		statev1.SystemStatusSubject); err != nil {
		return err
	}
	if err := bridgeSubject[uiv1.ButtonEvent](ctx, group, conn, logger, server,
		uiv1.ButtonEventSubject); err != nil {
		return err
	}
	if err := bridgeSubject[uiv1.TelemetrySummary](ctx, group, conn, logger, server,
		uiv1.TelemetrySummarySubject); err != nil {
		return err
	}
	if err := bridgeSubject[uiv1.ButtonHold](ctx, group, conn, logger, server,
		uiv1.ButtonHoldSubject); err != nil {
		return err
	}
	if err := bridgeSubject[uiv1.JumperInserted](ctx, group, conn, logger, server,
		uiv1.JumperInsertedSubject); err != nil {
		return err
	}
	if err := bridgeSubject[uiv1.ChallengeModeActive](ctx, group, conn, logger, server,
		uiv1.ChallengeModeActiveSubject); err != nil {
		return err
	}
	return bridgeSubject[visionv1.Detections](ctx, group, conn, logger, server,
		visionv1.DetectionsSubject)
}

// bridgeSubject registers subject as a Foxglove channel (using a fresh
// message purely for its schema) and starts a goroutine forwarding every
// message NATS delivers on it to that channel, until ctx is done. M is the
// concrete protobuf message type; the pointer type P is derived from it, so
// callers never pass a factory closure.
func bridgeSubject[M any, P interface {
	*M
	proto.Message
}](
	ctx context.Context, group *errgroup.Group, conn *natsConn, logger *slog.Logger,
	server *foxglove.Server, subject string,
) error {
	newT := func() P {
		var m M
		return &m
	}
	channelID, err := server.EnsureChannel(subject, newT())
	if err != nil {
		return fmt.Errorf("foxglove-bridge: registering channel for %s: %w", subject, err)
	}

	sub, err := nats.NewSubscriber[M, P](conn, subject)
	if err != nil {
		return err
	}

	group.Go(func() error {
		defer func() {
			if closeErr := sub.Close(); closeErr != nil {
				logger.Error("foxglove-bridge: closing subscription", "subject", subject, "error", closeErr)
			}
		}()
		for {
			msg, readErr := sub.Read(ctx)
			if readErr != nil {
				if errors.Is(readErr, context.Canceled) || errors.Is(readErr, context.DeadlineExceeded) {
					return nil
				}
				return readErr //nolint:wrapcheck // Read already wraps with "nats: ..." context
			}
			data, marshalErr := proto.Marshal(msg)
			if marshalErr != nil {
				logger.Error("foxglove-bridge: re-marshaling message", "subject", subject, "error", marshalErr)
				continue
			}
			server.Publish(channelID, time.Now(), data)
		}
	})
	return nil
}

func main() {
	os.Exit(mainWithExitCode())
}

// mainWithExitCode does the real work of main and returns the process exit
// code instead of calling os.Exit directly, so `defer` cleanup (closing
// the NATS connection, releasing the signal.NotifyContext) actually runs
// before the process exits.
func mainWithExitCode() int {
	logger := slog.New(slog.NewTextHandler(os.Stderr, nil))

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	var cfg cliConfig
	cmd := newRootCmd(&cfg, logger)
	cmd.SetArgs(os.Args[1:])

	if err := cmd.ExecuteContext(ctx); err != nil {
		logger.Error("foxglove-bridge run failed", "error", err)
		return exitError
	}
	return exitOK
}
