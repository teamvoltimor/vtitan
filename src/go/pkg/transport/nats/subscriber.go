package nats

import (
	"context"
	"errors"
	"fmt"

	"github.com/nats-io/nats.go"
	"google.golang.org/protobuf/proto"
)

// Subscriber receives protobuf messages of type T from a fixed NATS
// subject, one at a time via Read. Uses a synchronous subscription
// (NextMsgWithContext) rather than nats.go's async callback style, matching
// the blocking Read(ctx) shape already used by driver.Driver[T] and
// button.Driver -- one consistent pattern for "block until the next value
// arrives or ctx is done" across the whole module.
//
// When the connection's fault plan (Config.Faults) names the subject, the
// messages go through a faultQueue instead, fed by an async subscription so
// each message is stamped when it arrives rather than when Read asks.
type Subscriber[T proto.Message] struct {
	sub     *nats.Subscription
	faults  *faultQueue
	newT    func() T
	subject string
}

// NewSubscriber subscribes to subject on conn. M is the concrete protobuf
// message type and P the pointer type derived from it: each received
// message is unmarshaled into a freshly allocated M, so callers never pass
// a factory closure (the classic factory was needed because T is
// constrained to the proto.Message interface, which Go's generics cannot
// instantiate directly).
func NewSubscriber[M any, P interface {
	*M
	proto.Message
}](conn *nats.Conn, subject string) (*Subscriber[P], error) {
	s := &Subscriber[P]{subject: subject, newT: func() P {
		var m M
		return &m
	}}

	var err error
	if sf, seed, ok := faultsFor(conn, subject); ok {
		s.faults = newFaultQueue(subject, sf, seed)
		s.sub, err = conn.Subscribe(subject, s.faults.receive)
	} else {
		s.sub, err = conn.SubscribeSync(subject)
	}
	if err != nil {
		return nil, fmt.Errorf("nats: subscribing to subject %s: %w", subject, err)
	}
	return s, nil
}

// Read blocks until the next message arrives on the Subscriber's subject,
// or until ctx is done. Returns T, the same protobuf-message type parameter
// declared on Subscriber.
func (s *Subscriber[T]) Read(ctx context.Context) (T, error) {
	var zero T

	data, err := s.next(ctx)
	if err != nil {
		return zero, fmt.Errorf("nats: reading from subject %s: %w", s.subject, err)
	}

	out := s.newT()
	if err = proto.Unmarshal(data, out); err != nil {
		return zero, fmt.Errorf("nats: unmarshaling message from subject %s: %w", s.subject, err)
	}
	if s.faults != nil {
		s.faults.addNoise(out)
	}
	return out, nil
}

func (s *Subscriber[T]) next(ctx context.Context) ([]byte, error) {
	if s.faults == nil {
		msg, err := s.sub.NextMsgWithContext(ctx)
		if err != nil {
			return nil, err //nolint:wrapcheck // wrapped by Read
		}
		return msg.Data, nil
	}
	data, err := s.faults.next(ctx.Done())
	if errors.Is(err, errReadDone) {
		return nil, ctx.Err() //nolint:wrapcheck // wrapped by Read
	}
	return data, err
}

// FaultStats reports what the connection's fault plan did to this
// subject, and false when the plan does not name it.
func (s *Subscriber[T]) FaultStats() (FaultStats, bool) {
	if s.faults == nil {
		return FaultStats{}, false
	}
	return s.faults.snapshot(), true
}

// Close unsubscribes. Safe to call at most once.
func (s *Subscriber[T]) Close() error {
	if err := s.sub.Unsubscribe(); err != nil {
		return fmt.Errorf("nats: unsubscribing from subject %s: %w", s.subject, err)
	}
	return nil
}
