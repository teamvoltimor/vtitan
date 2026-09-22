package imu

import (
	"context"
	"errors"
	"log/slog"

	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
	"github.com/teamvoltimor/vtitan/src/go/pkg/driver/imu"
	"github.com/teamvoltimor/vtitan/src/go/pkg/transport/nats"
)

// Config wires the IMU driver and the NATS connection the publish loop uses.
type Config struct {
	Driver imu.Config
	NATS   nats.Config
}

// Run connects the IMU driver and NATS, then publishes every reading until
// ctx is done or Read returns a non-cancellation error.
//
// Run owns both resources rather than taking them from the caller, so a
// supervised restart (cmd/pi5) re-opens the serial port and reconnects
// instead of inheriting a half-dead one -- which is the failure this loop is
// most likely to be restarted for.
func Run(ctx context.Context, cfg Config, logger *slog.Logger) error {
	drv, err := imu.New(cfg.Driver)
	if err != nil {
		return err //nolint:wrapcheck // imu.New already wraps with "imu: ..." context
	}
	if err = drv.Connect(ctx); err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "imu: ..." context
	}
	defer func() {
		if closeErr := drv.Close(); closeErr != nil {
			logger.Error("imu: closing IMU driver", "error", closeErr)
		}
	}()

	conn, err := nats.Connect(ctx, cfg.NATS)
	if err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "nats: ..." context
	}
	defer conn.Close()

	pub := nats.NewPublisher[*sensorv1.Imu](conn, sensorv1.ImuSubject)

	logger.Info("imu: connected", "nats_url", cfg.NATS.URL, "port", cfg.Driver.Port)
	return publishLoop(ctx, logger, drv, pub)
}

// publishLoop reads successive Readings from drv and publishes each as an
// Imu message, until ctx is done or Read returns a non-cancellation error.
func publishLoop(
	ctx context.Context,
	logger *slog.Logger,
	drv *imu.RVCDriver,
	pub *nats.Publisher[*sensorv1.Imu],
) error {
	for {
		reading, err := drv.Read(ctx)
		if err != nil {
			if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
				return nil
			}
			return err //nolint:wrapcheck // Read already wraps with "imu: ..." context
		}

		if pubErr := pub.Publish(MessageFor(reading)); pubErr != nil {
			logger.Error("imu: publishing Imu", "error", pubErr)
		}
	}
}
