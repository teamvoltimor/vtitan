package nav

import (
	"context"
	"errors"
	"fmt"
	"log/slog"

	natsio "github.com/nats-io/nats.go"

	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
	"github.com/teamvoltimor/vtitan/src/go/pkg/recording"
	"github.com/teamvoltimor/vtitan/src/go/pkg/transport/nats"
)

// recordMotorStatus writes every MotorStatus into the run's bag until ctx is
// done. It is the actuation board's side of the run: its state, the command
// age it sees and, from a Pico 2, the command link's health (LinkHealth),
// none of which the bags carried before (platform plan items 0.5 and 2.14).
func recordMotorStatus(ctx context.Context, conn *natsio.Conn, rec *recording.RunRecorder, logger *slog.Logger) error {
	sub, err := nats.NewSubscriber[actuationv1.MotorStatus](conn, actuationv1.MotorStatusSubject)
	if err != nil {
		return err
	}
	defer func() {
		if closeErr := sub.Close(); closeErr != nil {
			logger.Error("track-navigator: closing MotorStatus subscription", "error", closeErr)
		}
	}()
	for {
		st, readErr := sub.Read(ctx)
		if readErr != nil {
			if errors.Is(readErr, context.Canceled) || errors.Is(readErr, context.DeadlineExceeded) {
				return nil
			}
			return readErr //nolint:wrapcheck // Read already wraps with "nats: ..." context
		}
		if err = rec.WriteMessage(actuationv1.MotorStatusSubject, st, logTimeNow()); err != nil {
			return fmt.Errorf("track-navigator: recording MotorStatus: %w", err)
		}
	}
}
