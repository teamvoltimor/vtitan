package outbox

import "context"

// Slot is a single-item, keep-latest, non-blocking mailbox for one
// outbound message type -- the Go analog of TelemetryIngestChannel's
// `queue.Queue(maxsize=1)` plus `_push_latest`'s drop-oldest-on-full
// logic. A producer calls Push on every tick without ever blocking; a
// consumer (Drain, in drain.go, or a caller driving Next directly) always
// sees the most recently pushed item, never a backlog.
type Slot[T any] struct {
	ch chan T
}

// slotCapacity is fixed at one, not a Config-worthy tunable: the whole
// point of Slot is "the next pushed item replaces whatever hasn't been
// sent yet," not buffering a backlog -- see _push_latest's docstring in
// telemetry_ingest_channel.py ("never makes the caller wait on a stalled
// stream"). A different capacity would change Slot's actual behavior, not
// just its tuning, so it stays a named constant rather than a field.
const slotCapacity = 1

// NewSlot builds an empty Slot.
func NewSlot[T any]() *Slot[T] {
	return &Slot[T]{ch: make(chan T, slotCapacity)}
}

// Push stores item, discarding whatever was previously queued and not
// yet collected by Next -- matching `_push_latest`'s
// "if full, drop the old one, then enqueue" sequence. Never blocks.
func (s *Slot[T]) Push(item T) {
	select {
	case <-s.ch: // drop the stale value, if any, matching q.get_nowait()
	default:
	}
	select {
	case s.ch <- item:
	default:
		// Only reachable if another goroutine's Push refilled the slot
		// between the drain above and this send -- Slot promises the most
		// recently *successfully* queued item, not delivery of every
		// single Push call, so losing this race is fine: the other
		// Push's item is what a concurrent caller intended to be latest
		// anyway.
	}
}

// Next blocks until an item is available or ctx is done, matching
// `_drain`'s `q.get(timeout=0.5)` polling loop collapsed into a single
// channel receive -- context-first cancellation per this project's
// concurrency convention. ok is false only when ctx was done first.
//
//nolint:ireturn // T is Slot's own type parameter, not a leaked interface -- ireturn's generic-instantiation false positive.
func (s *Slot[T]) Next(ctx context.Context) (item T, ok bool) {
	select {
	case item = <-s.ch:
		return item, true
	case <-ctx.Done():
		var zero T
		return zero, false
	}
}
