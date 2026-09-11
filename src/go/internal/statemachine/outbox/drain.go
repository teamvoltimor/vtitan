package outbox

import (
	"context"
	"fmt"
)

// Streamer is what a background transport loop needs to deliver one Slot
// item somewhere -- the interface a future gRPC- or NATS-backed
// implementation satisfies (the direct analog of
// telemetry_ingest_channel.py's `_run_stream` opening a channel and
// calling `rpc(self._drain(q))`). Defined here, at the point of use
// (go-architect §4, matching internal/telemetry/diag.Source and
// internal/statemachine/command.Sink), so Drain's loop/cancellation logic
// is fully testable against a fake Streamer without any real transport.
type Streamer[T any] interface {
	Send(ctx context.Context, item T) error
}

// Drain repeatedly pulls the latest item from slot and forwards it to
// streamer until ctx is done or a Send fails, at which point it returns
// that error to the caller. Reconnect/backoff is deliberately not this
// function's job -- both real Python channels scope their backoff loop
// one level up, around re-opening the whole stream (`_command_channel_loop`/
// `_stream_loop`), and internal/statemachine/backoff is the Go port of
// that policy; a caller wraps Drain in its own retry loop using it,
// rather than Drain retrying internally.
func Drain[T any](ctx context.Context, slot *Slot[T], streamer Streamer[T]) error {
	for {
		item, ok := slot.Next(ctx)
		if !ok {
			return fmt.Errorf("outbox: drain canceled: %w", ctx.Err())
		}
		if err := streamer.Send(ctx, item); err != nil {
			return fmt.Errorf("outbox: send: %w", err)
		}
	}
}
