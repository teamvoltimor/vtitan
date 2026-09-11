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

	"github.com/teamvoltimor/vtitan/src/go/internal/foxglove"
	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
	navv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/nav/v1"
	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
	statev1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/state/v1"
	uiv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/ui/v1"
	visionv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/vision/v1"
	"github.com/teamvoltimor/vtitan/src/go/internal/transport/nats"
)

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

type cliConfig struct {
	natsURL  string
	nodeName string
	httpAddr string
}

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
	flags.StringVar(&cfg.natsURL, "nats-url", nats.DefaultURL(), "nats-server URL")
	flags.StringVar(&cfg.nodeName, "name", "foxglove-bridge", "NATS client name, visible in nats-server's connz output")
	flags.StringVar(&cfg.httpAddr, "http-addr", defaultHTTPAddr, "address to serve the Foxglove WebSocket protocol on")

	return cmd
}

// bridgedSubject pairs a subject with the factory bridgeSubject needs to
// build fresh messages of its concrete type -- declared once per subject in
// run() below, matching the table every other multi-subject cmd/* binary
// in this tree (e.g. telemetry-node) builds inline rather than via
// reflection, since the concrete type set is small, static, and known at
// compile time.
type bridgedSubject struct {
	subject string
	bridge  func(ctx context.Context, group *errgroup.Group, conn *natsConn, logger *slog.Logger, server *foxglove.Server) error
}

// natsConn is the concrete *nats.Conn type from the nats-io client library
// -- named here only to keep bridgeSubject's signature readable without
// repeating the fully qualified import alias at every call site.
type natsConn = natsio.Conn

func run(ctx context.Context, logger *slog.Logger, cfg cliConfig) error {
	conn, err := nats.Connect(ctx, nats.DefaultConfig(cfg.natsURL, cfg.nodeName))
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
		logger.Info("foxglove-bridge: serving", "addr", cfg.httpAddr, "nats_url", cfg.natsURL)
		if serveErr := httpServer.ListenAndServe(); serveErr != nil && !errors.Is(serveErr, http.ErrServerClosed) {
			return fmt.Errorf("foxglove-bridge: http server: %w", serveErr)
		}
		return nil
	})
	group.Go(func() error {
		<-gctx.Done()
		shutdownCtx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
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
	if err := bridgeSubject(ctx, group, conn, logger, server,
		actuationv1.AckermannCmdSubject, func() *actuationv1.AckermannCmd { return &actuationv1.AckermannCmd{} }); err != nil {
		return err
	}
	if err := bridgeSubject(ctx, group, conn, logger, server,
		actuationv1.MotorStatusSubject, func() *actuationv1.MotorStatus { return &actuationv1.MotorStatus{} }); err != nil {
		return err
	}
	if err := bridgeSubject(ctx, group, conn, logger, server,
		navv1.NavigatorDebugSubject, func() *navv1.NavigatorDebug { return &navv1.NavigatorDebug{} }); err != nil {
		return err
	}
	if err := bridgeSubject(ctx, group, conn, logger, server,
		navv1.LapsCompletedSubject, func() *navv1.LapsCompleted { return &navv1.LapsCompleted{} }); err != nil {
		return err
	}
	if err := bridgeSubject(ctx, group, conn, logger, server,
		navv1.CurrentCorridorSubject, func() *navv1.CurrentCorridor { return &navv1.CurrentCorridor{} }); err != nil {
		return err
	}
	if err := bridgeSubject(ctx, group, conn, logger, server,
		sensorv1.ImuSubject, func() *sensorv1.Imu { return &sensorv1.Imu{} }); err != nil {
		return err
	}
	if err := bridgeSubject(ctx, group, conn, logger, server,
		sensorv1.ScanSubject, func() *sensorv1.Scan { return &sensorv1.Scan{} }); err != nil {
		return err
	}
	if err := bridgeSubject(ctx, group, conn, logger, server,
		sensorv1.CameraSubject, func() *sensorv1.CameraFrame { return &sensorv1.CameraFrame{} }); err != nil {
		return err
	}
	if err := bridgeSubject(ctx, group, conn, logger, server,
		statev1.RobotStateSubject, func() *statev1.RobotState { return &statev1.RobotState{} }); err != nil {
		return err
	}
	if err := bridgeSubject(ctx, group, conn, logger, server,
		statev1.RaceMetricsSubject, func() *statev1.RaceMetrics { return &statev1.RaceMetrics{} }); err != nil {
		return err
	}
	if err := bridgeSubject(ctx, group, conn, logger, server,
		statev1.SystemStatusSubject, func() *statev1.SystemStatus { return &statev1.SystemStatus{} }); err != nil {
		return err
	}
	if err := bridgeSubject(ctx, group, conn, logger, server,
		uiv1.ButtonEventSubject, func() *uiv1.ButtonEvent { return &uiv1.ButtonEvent{} }); err != nil {
		return err
	}
	if err := bridgeSubject(ctx, group, conn, logger, server,
		uiv1.TelemetrySummarySubject, func() *uiv1.TelemetrySummary { return &uiv1.TelemetrySummary{} }); err != nil {
		return err
	}
	if err := bridgeSubject(ctx, group, conn, logger, server,
		uiv1.ButtonHoldSubject, func() *uiv1.ButtonHold { return &uiv1.ButtonHold{} }); err != nil {
		return err
	}
	if err := bridgeSubject(ctx, group, conn, logger, server,
		uiv1.JumperInsertedSubject, func() *uiv1.JumperInserted { return &uiv1.JumperInserted{} }); err != nil {
		return err
	}
	if err := bridgeSubject(ctx, group, conn, logger, server,
		uiv1.ChallengeModeActiveSubject, func() *uiv1.ChallengeModeActive { return &uiv1.ChallengeModeActive{} }); err != nil {
		return err
	}
	if err := bridgeSubject(ctx, group, conn, logger, server,
		visionv1.DetectionsSubject, func() *visionv1.Detections { return &visionv1.Detections{} }); err != nil {
		return err
	}
	return nil
}

// bridgeSubject registers subject as a Foxglove channel (using a fresh T
// purely for its schema) and starts a goroutine forwarding every message
// NATS delivers on it to that channel, until ctx is done.
func bridgeSubject[T proto.Message](
	ctx context.Context, group *errgroup.Group, conn *natsConn, logger *slog.Logger,
	server *foxglove.Server, subject string, newT func() T,
) error {
	channelID, err := server.EnsureChannel(subject, newT())
	if err != nil {
		return fmt.Errorf("foxglove-bridge: registering channel for %s: %w", subject, err)
	}

	sub, err := nats.NewSubscriber(conn, subject, newT)
	if err != nil {
		return err //nolint:wrapcheck // NewSubscriber already wraps with "nats: ..." context
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
