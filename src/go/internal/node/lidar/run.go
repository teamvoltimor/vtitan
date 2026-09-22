package lidar

import (
	"context"
	"errors"
	"log/slog"
	"time"

	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
	"github.com/teamvoltimor/vtitan/src/go/pkg/driver/lidar"
	"github.com/teamvoltimor/vtitan/src/go/pkg/transport/nats"
)

// Config wires the LIDAR driver and the NATS connection the publish loop uses.
type Config struct {
	Driver lidar.Config
	NATS   nats.Config
}

// Run connects the LIDAR driver and NATS, then publishes every assembled scan
// until ctx is done or Read returns a non-cancellation error.
//
// Run owns both resources rather than taking them from the caller, so a
// supervised restart (cmd/pi5) re-opens the serial port and reconnects instead
// of inheriting a half-dead one.
func Run(ctx context.Context, cfg Config, logger *slog.Logger) error {
	drv, err := lidar.NewClassic(cfg.Driver)
	if err != nil {
		return err //nolint:wrapcheck // lidar.NewClassic already wraps with "lidar: ..." context
	}
	if err = drv.Connect(ctx); err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "lidar: ..." context
	}
	defer func() {
		if closeErr := drv.Close(); closeErr != nil {
			logger.Error("lidar: closing LIDAR driver", "error", closeErr)
		}
	}()

	conn, err := nats.Connect(ctx, cfg.NATS)
	if err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "nats: ..." context
	}
	defer conn.Close()

	pub := nats.NewPublisher[*sensorv1.Scan](conn, sensorv1.ScanSubject)

	logger.Info("lidar: connected", "nats_url", cfg.NATS.URL, "port", cfg.Driver.Port)
	return publishLoop(ctx, logger, drv, pub)
}

// publishLoop reads successive Scans from drv and publishes each as a Scan
// message, until ctx is done or Read returns a non-cancellation error.
func publishLoop(
	ctx context.Context,
	logger *slog.Logger,
	drv *lidar.ClassicSerialDriver,
	pub *nats.Publisher[*sensorv1.Scan],
) error {
	lastScanAt := time.Now()

	for {
		scan, err := drv.Read(ctx)
		if err != nil {
			if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
				return nil
			}
			return err //nolint:wrapcheck // Read already wraps with "lidar: ..." context
		}

		now := time.Now()
		if pubErr := pub.Publish(MessageFor(scan, now.Sub(lastScanAt))); pubErr != nil {
			logger.Error("lidar: publishing Scan", "error", pubErr)
		}
		lastScanAt = now
	}
}
