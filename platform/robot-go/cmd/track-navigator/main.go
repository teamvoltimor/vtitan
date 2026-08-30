// Command track-navigator is a bench/dev single-subsystem binary for the
// nav stack (track model, direction estimator, sign router, navigator),
// sharing the same internal packages as cmd/pi5 — used for isolated bench
// testing and local debugging, and for bag-replay/sim-corpus parity work
// before this subsystem is trusted inside cmd/pi5.
//
// It wires the NATS-backed controllers.HardwareGateway (internal/adapters/natsgw)
// to the navigator composition root (internal/nav/navigator) and drives Step at
// the nav control rate. The gateway subscribes to vtitan.sensor.v1.scan and
// vtitan.sensor.v1.imu (published by cmd/lidar-node and the IMU node), estimates
// pose in-process via localization.LidarLocalizer, and publishes drive commands
// on vtitan.actuation.v1.ackermann_cmd (consumed by the motor node) — no new
// NATS subjects are introduced here.
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
	"golang.org/x/sync/errgroup"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/adapters/natsgw"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/localization"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navigator"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
	sensorv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/sensor/v1"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/transport/nats"
)

// benchTrackCoord is the outer boundary of the default bench track layout.
// A 2 m-wide corridor on every side around a centered inner block — a sane,
// well-conditioned layout for isolated bench runs that do not load a real
// scenario's geometry.
const benchTrackCoord = 4.0

// cliConfig holds every flag track-navigator accepts.
type cliConfig struct {
	natsURL  string
	nodeName string
	rateHz   float64
}

// exit codes: 0 means track-navigator ran and shut down cleanly (including via
// SIGINT/SIGTERM). 1 means it could not start or hit an unrecoverable runtime
// error.
const (
	exitOK    = 0
	exitError = 1
)

var _ controllers.HardwareGateway = (*natsgw.Gateway)(nil)

func newRootCmd(cfg *cliConfig, logger *slog.Logger) *cobra.Command {
	cmd := &cobra.Command{
		Use:   "track-navigator",
		Short: "Run the nav stack (navigator + controllers) against live NATS topics",
		Long: "track-navigator wires the NATS-backed controllers.HardwareGateway to the\n" +
			"navigator composition root, subscribes to scan/IMU, estimates pose in-process,\n" +
			"and steps the navigator (publishing AckermannCmd) at --rate-hz.",
		SilenceUsage:  true,
		SilenceErrors: true,
		RunE: func(cmd *cobra.Command, _ []string) error {
			return run(cmd.Context(), logger, *cfg)
		},
	}

	flags := cmd.Flags()
	flags.StringVar(&cfg.natsURL, "nats-url", nats.DefaultDevURL, "nats-server URL")
	flags.StringVar(&cfg.nodeName, "name", "track-navigator", "NATS client name, visible in nats-server's connz output")
	flags.Float64Var(&cfg.rateHz, "rate-hz", 20.0, "navigator Step rate")

	return cmd
}

// benchWalls builds a default square Open Challenge layout for the gateway's
// localizer seed. A real run would load the scenario's geometry instead.
func benchWalls() *trackmodel.TrackWalls {
	geom := trackmodel.CorridorGeometryFromWidths(map[trackmodel.Section]float64{
		trackmodel.North: 2.0,
		trackmodel.South: 2.0,
		trackmodel.East:  2.0,
		trackmodel.West:  2.0,
	}, benchTrackCoord)
	return trackmodel.NewTrackWalls(geom, -benchTrackCoord, benchTrackCoord)
}

func run(ctx context.Context, logger *slog.Logger, cfg cliConfig) error {
	conn, err := nats.Connect(nats.DefaultConfig(cfg.natsURL, cfg.nodeName))
	if err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "nats: ..." context
	}
	defer conn.Close()

	scanSub, err := nats.NewSubscriber(conn, sensorv1.ScanSubject, func() *sensorv1.Scan { return &sensorv1.Scan{} })
	if err != nil {
		return err //nolint:wrapcheck // NewSubscriber already wraps with "nats: ..." context
	}
	defer func() {
		if closeErr := scanSub.Close(); closeErr != nil {
			logger.Error("track-navigator: closing Scan subscription", "error", closeErr)
		}
	}()

	imuSub, err := nats.NewSubscriber(conn, sensorv1.ImuSubject, func() *sensorv1.Imu { return &sensorv1.Imu{} })
	if err != nil {
		return err //nolint:wrapcheck // NewSubscriber already wraps with "nats: ..." context
	}
	defer func() {
		if closeErr := imuSub.Close(); closeErr != nil {
			logger.Error("track-navigator: closing IMU subscription", "error", closeErr)
		}
	}()

	gw, err := natsgw.New(conn, benchWalls(), localization.DefaultConfig())
	if err != nil {
		return err //nolint:wrapcheck
	}
	defer func() {
		if closeErr := gw.Close(); closeErr != nil {
			logger.Error("track-navigator: closing gateway", "error", closeErr)
		}
	}()

	// A single canonical 4-corner lap around the bench track, so the
	// navigator has a path to drive. A real run loads the scenario's
	// waypoints.
	waypoints := []trackmodel.Waypoint{
		{X: -benchTrackCoord + 1, Y: -benchTrackCoord + 1},
		{X: benchTrackCoord - 1, Y: -benchTrackCoord + 1},
		{X: benchTrackCoord - 1, Y: benchTrackCoord - 1},
		{X: -benchTrackCoord + 1, Y: benchTrackCoord - 1},
	}

	nav, err := navigator.New(navigator.Params{
		Gateway:           gw,
		Waypoints:         waypoints,
		Direction:         trackmodel.Counterclockwise,
		Config:            navigator.DefaultConfig(),
		ControllersConfig: controllers.DefaultConfig(),
		Logger:            logger,
	})
	if err != nil {
		return err //nolint:wrapcheck
	}

	logger.Info("track-navigator: connected", "nats_url", cfg.natsURL, "rate_hz", cfg.rateHz)

	group, gctx := errgroup.WithContext(ctx)
	group.Go(func() error { return gw.Run(gctx, scanSub, imuSub) })
	group.Go(func() error { return stepLoop(gctx, logger, nav, cfg.rateHz) })

	if err = group.Wait(); err != nil {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return nil
		}
		return err //nolint:wrapcheck // each goroutine's own error is already package-prefixed
	}
	return nil
}

// stepLoop drives nav.Step at rateHz until ctx is done.
func stepLoop(ctx context.Context, logger *slog.Logger, nav *navigator.Navigator, rateHz float64) error {
	ticker := time.NewTicker(time.Duration(float64(time.Second) / rateHz))
	defer ticker.Stop()

	steps := 0
	for {
		select {
		case <-ctx.Done():
			logger.Info("track-navigator: stepped", "steps", steps)
			return nil
		case <-ticker.C:
			nav.Step()
			steps++
		}
	}
}

func main() {
	os.Exit(mainWithExitCode())
}

// mainWithExitCode does the real work of main and returns the process exit
// code instead of calling os.Exit directly, so deferred cleanup (closing NATS
// subscriptions/connection, releasing the signal.NotifyContext) actually runs
// before the process exits.
func mainWithExitCode() int {
	logger := slog.New(slog.NewTextHandler(os.Stderr, nil))

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	var cfg cliConfig
	cmd := newRootCmd(&cfg, logger)
	cmd.SetArgs(os.Args[1:])

	if err := cmd.ExecuteContext(ctx); err != nil {
		logger.Error("track-navigator run failed", "error", err)
		return exitError
	}
	return exitOK
}
