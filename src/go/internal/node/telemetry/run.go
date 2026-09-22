package telemetry

import (
	"context"
	"errors"
	"log/slog"
	"time"

	"golang.org/x/sync/errgroup"
	"google.golang.org/protobuf/proto"

	"github.com/teamvoltimor/vtitan/src/go/internal/telemetry/diag"

	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
	uiv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/ui/v1"
	"github.com/teamvoltimor/vtitan/src/go/internal/transport/nats"
)

// Config wires the NATS connection and the summary publish rate.
type Config struct {
	NATS   nats.Config
	RateHz float64
}

// DefaultRateHz matches telemetry_bridge_node.py's ui_summary_rate_hz
// default (10Hz) -- the rate _publish_ui_summary redraws the OLED at.
const DefaultRateHz = 10.0

// Run subscribes to the IMU and Scan subjects, aggregates them, and publishes
// a TelemetrySummary at cfg.RateHz until ctx is done or a subscription hits a
// non-cancellation error.
//
// Run owns the connection and both subscriptions, so a supervised restart
// (cmd/pi5) re-subscribes from scratch rather than inheriting a dead
// subscription.
func Run(ctx context.Context, cfg Config, logger *slog.Logger) error {
	conn, err := nats.Connect(ctx, cfg.NATS)
	if err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "nats: ..." context
	}
	defer conn.Close()

	imuSub, err := nats.NewSubscriber[sensorv1.Imu](conn, sensorv1.ImuSubject)
	if err != nil {
		return err
	}
	defer func() {
		if closeErr := imuSub.Close(); closeErr != nil {
			logger.Error("telemetry: closing IMU subscription", "error", closeErr)
		}
	}()

	scanSub, err := nats.NewSubscriber[sensorv1.Scan](conn, sensorv1.ScanSubject)
	if err != nil {
		return err
	}
	defer func() {
		if closeErr := scanSub.Close(); closeErr != nil {
			logger.Error("telemetry: closing Scan subscription", "error", closeErr)
		}
	}()

	pub := nats.NewPublisher[*uiv1.TelemetrySummary](conn, uiv1.TelemetrySummarySubject)

	source := &natsSource{}
	// No LIDAR mount correction is applied here: lidar-node publishes in the
	// robot frame already (lidar.ConfigFor). Re-applying it would double it.
	aggregator := diag.NewAggregator(source, diag.DefaultConfig())

	logger.Info("telemetry: connected", "nats_url", cfg.NATS.URL, "rate_hz", cfg.RateHz)

	group, gctx := errgroup.WithContext(ctx)
	group.Go(func() error { return watchLoop(gctx, imuSub, source.setIMU) })
	group.Go(func() error { return watchLoop(gctx, scanSub, source.setScan) })
	group.Go(func() error { return publishLoop(gctx, logger, aggregator, pub, cfg.RateHz) })

	if err = group.Wait(); err != nil {
		return err //nolint:wrapcheck // each goroutine's own error is already package-prefixed
	}
	return nil
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
			if pubErr := pub.Publish(MessageFor(summary)); pubErr != nil {
				logger.Error("telemetry: publishing TelemetrySummary", "error", pubErr)
			}
		}
	}
}
