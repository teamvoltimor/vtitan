//go:build linux

// Command telemetry-node is a bench/dev single-subsystem binary for the
// telemetry-summary aggregation loop, sharing the same internal packages as
// cmd/pi5 — used for isolated bench testing and local debugging without the
// whole board binary.
//
// It subscribes to the IMU and LIDAR topics other Go nodes publish
// (vtitan.sensor.v1.imu, vtitan.sensor.v1.scan), feeds them into the
// existing internal/telemetry/diag.Aggregator, and publishes the resulting
// TelemetrySummary on vtitan.ui.v1.telemetry_summary for cmd/pi-zero's OLED
// to render. There is no vision detection wire schema yet (see
// diag.Detection's doc comment), so best-detection fields are always
// omitted here, matching Aggregator's own documented behavior for an input
// that has never arrived.
package main

import (
	"context"
	"errors"
	"log/slog"
	"os"
	"os/signal"
	"sync"
	"syscall"
	"time"

	"github.com/spf13/cobra"
	"golang.org/x/sync/errgroup"
	"google.golang.org/protobuf/proto"
	"google.golang.org/protobuf/types/known/timestamppb"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/telemetry/diag"

	sensorv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/sensor/v1"
	uiv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/ui/v1"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/transport/nats"
)

// cliConfig holds every flag telemetry-node accepts.
type cliConfig struct {
	natsURL    string
	nodeName   string
	rateHz     float64
	configRoot string
}

// natsSource implements diag.Source by caching the latest message received
// on each subscription -- the Go analog of telemetry_bridge_node.py's
// `_latest_scan`/`_latest_imu` instance caches, updated by each
// subscription's own read loop (watchLoop) rather than blocking Summarize
// on a topic.
type natsSource struct {
	mu   sync.RWMutex
	scan *sensorv1.Scan
	imu  *sensorv1.Imu
}

// defaultRateHz matches telemetry_bridge_node.py's ui_summary_rate_hz
// default (10Hz) -- the rate _publish_ui_summary redraws the OLED at.
const defaultRateHz = 10.0

// exit codes: 0 means telemetry-node ran and shut down cleanly (including
// via SIGINT/SIGTERM). 1 means it could not start or hit an unrecoverable
// runtime error.
const (
	exitOK    = 0
	exitError = 1
)

var _ diag.Source = (*natsSource)(nil)

func newRootCmd(cfg *cliConfig, logger *slog.Logger) *cobra.Command {
	cmd := &cobra.Command{
		Use:   "telemetry-node",
		Short: "Aggregate IMU/LIDAR topics into a TelemetrySummary and publish it over NATS",
		Long: "telemetry-node subscribes to vtitan.sensor.v1.imu and vtitan.sensor.v1.scan,\n" +
			"feeds them into the existing telemetry-summary aggregator, and publishes the\n" +
			"result on vtitan.ui.v1.telemetry_summary at --rate-hz for cmd/pi-zero's OLED.",
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
		"telemetry-node",
		"NATS client name, visible in nats-server's connz output",
	)
	flags.Float64Var(&cfg.rateHz, "rate-hz", defaultRateHz, "TelemetrySummary publish rate")
	flags.StringVar(&cfg.configRoot, "config-root", "",
		"repo root to load the hardware profile (VTITAN_HARDWARE_PROFILE) from; "+
			"empty uses a zero LIDAR yaw offset")

	return cmd
}

// LatestScan implements diag.Source.
func (s *natsSource) LatestScan(_ context.Context) (*sensorv1.Scan, bool) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.scan, s.scan != nil
}

// LatestIMU implements diag.Source.
func (s *natsSource) LatestIMU(_ context.Context) (*sensorv1.Imu, bool) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.imu, s.imu != nil
}

// LatestDetections implements diag.Source. Always reports no detection --
// there is no vision detection wire schema yet (see diag.Detection's doc
// comment: vision stays Python-only and hasn't been given a proto), and
// Aggregator's own documented behavior for an input that has never arrived
// is to omit it from the summary, so this is the correct permanent answer
// here, not a placeholder to fill in later.
func (s *natsSource) LatestDetections(_ context.Context) ([]diag.Detection, bool) {
	return nil, false
}

func (s *natsSource) setScan(scan *sensorv1.Scan) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.scan = scan
}

func (s *natsSource) setIMU(imu *sensorv1.Imu) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.imu = imu
}

// watchLoop reads successive messages from sub and hands each to store,
// until ctx is done or Read returns a non-cancellation error.
func watchLoop[T proto.Message](ctx context.Context, sub *nats.Subscriber[T], store func(T)) error {
	for {
		msg, err := sub.Read(ctx)
		if err != nil {
			if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
				return nil
			}
			return err //nolint:wrapcheck // Read already wraps with "nats: ..." context
		}
		store(msg)
	}
}

// summaryMessageFor converts a diag.TelemetrySummary into the
// TelemetrySummary message to publish.
func summaryMessageFor(summary diag.TelemetrySummary) *uiv1.TelemetrySummary {
	return &uiv1.TelemetrySummary{
		Stamp:                   timestamppb.Now(),
		BestDetectionClassId:    summary.BestDetectionClassID,
		BestDetectionConfidence: summary.BestDetectionConfidence,
		HasBestDetection:        summary.HasBestDetection,
		LidarFrontCm:            summary.LidarFrontCM,
		LidarLeftCm:             summary.LidarLeftCM,
		LidarRightCm:            summary.LidarRightCM,
		GyroYawDeg:              summary.GyroYawDeg,
	}
}

// run wires the aggregator to NATS and blocks until ctx is done or a
// subscription hits a non-cancellation error.
func run(ctx context.Context, logger *slog.Logger, cfg cliConfig) error {
	conn, err := nats.Connect(ctx, nats.DefaultConfig(cfg.natsURL, cfg.nodeName))
	if err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "nats: ..." context
	}
	defer conn.Close()

	imuSub, err := nats.NewSubscriber(
		conn,
		sensorv1.ImuSubject,
		func() *sensorv1.Imu { return &sensorv1.Imu{} },
	)
	if err != nil {
		return err //nolint:wrapcheck // NewSubscriber already wraps with "nats: ..." context
	}
	defer func() {
		if closeErr := imuSub.Close(); closeErr != nil {
			logger.Error("telemetry-node: closing IMU subscription", "error", closeErr)
		}
	}()

	scanSub, err := nats.NewSubscriber(
		conn,
		sensorv1.ScanSubject,
		func() *sensorv1.Scan { return &sensorv1.Scan{} },
	)
	if err != nil {
		return err //nolint:wrapcheck // NewSubscriber already wraps with "nats: ..." context
	}
	defer func() {
		if closeErr := scanSub.Close(); closeErr != nil {
			logger.Error("telemetry-node: closing Scan subscription", "error", closeErr)
		}
	}()

	pub := nats.NewPublisher[*uiv1.TelemetrySummary](conn, uiv1.TelemetrySummarySubject)

	source := &natsSource{}
	aggCfg := diag.DefaultConfig()
	aggCfg.LidarYawOffsetRad = diag.LidarYawOffsetRadFor(logger, cfg.configRoot)
	aggregator := diag.NewAggregator(source, aggCfg)

	logger.Info("telemetry-node: connected", "nats_url", cfg.natsURL, "rate_hz", cfg.rateHz)

	group, gctx := errgroup.WithContext(ctx)
	group.Go(func() error { return watchLoop(gctx, imuSub, source.setIMU) })
	group.Go(func() error { return watchLoop(gctx, scanSub, source.setScan) })
	group.Go(func() error { return publishLoop(gctx, logger, aggregator, pub, cfg.rateHz) })

	if err = group.Wait(); err != nil {
		return err //nolint:wrapcheck // each goroutine's own error is already package-prefixed
	}
	return nil
}

// publishLoop summarizes and publishes at rateHz until ctx is done.
func publishLoop(
	ctx context.Context,
	logger *slog.Logger,
	aggregator *diag.Aggregator,
	pub *nats.Publisher[*uiv1.TelemetrySummary],
	rateHz float64,
) error {
	ticker := time.NewTicker(time.Duration(float64(time.Second) / rateHz))
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			return nil
		case <-ticker.C:
			summary := aggregator.Summarize(ctx)
			if pubErr := pub.Publish(summaryMessageFor(summary)); pubErr != nil {
				logger.Error("telemetry-node: publishing TelemetrySummary", "error", pubErr)
			}
		}
	}
}

func main() {
	os.Exit(mainWithExitCode())
}

// mainWithExitCode does the real work of main and returns the process exit
// code instead of calling os.Exit directly, so `defer` cleanup (closing the
// NATS subscriptions/connection, releasing the signal.NotifyContext)
// actually runs before the process exits.
func mainWithExitCode() int {
	logger := slog.New(slog.NewTextHandler(os.Stderr, nil))

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	var cfg cliConfig
	cmd := newRootCmd(&cfg, logger)
	cmd.SetArgs(os.Args[1:])

	if err := cmd.ExecuteContext(ctx); err != nil {
		logger.Error("telemetry-node run failed", "error", err)
		return exitError
	}
	return exitOK
}
