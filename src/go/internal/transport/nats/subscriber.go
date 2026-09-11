package nats

import (
	"context"
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
type Subscriber[T proto.Message] struct {
	sub     *nats.Subscription
	subject string
	newT    func() T
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
	sub, err := conn.SubscribeSync(subject)
	if err != nil {
		return nil, fmt.Errorf("nats: subscribing to subject %s: %w", subject, err)
	}
	return &Subscriber[P]{sub: sub, subject: subject, newT: func() P {
		var m M
		return &m
	}}, nil
}

// Read blocks until the next message arrives on the Subscriber's subject,
// or until ctx is done. Returns T, the same protobuf-message type parameter
// declared on Subscriber.
//
//nolint:ireturn // T is a generic constrained to proto.Message, not a hand-written interface-return choice
func (s *Subscriber[T]) Read(ctx context.Context) (T, error) {
	var zero T

	msg, err := s.sub.NextMsgWithContext(ctx)
	if err != nil {
		return zero, fmt.Errorf("nats: reading from subject %s: %w", s.subject, err)
	}

	out := s.newT()
	if err = proto.Unmarshal(msg.Data, out); err != nil {
		return zero, fmt.Errorf("nats: unmarshaling message from subject %s: %w", s.subject, err)
	}
	return out, nil
}

// Close unsubscribes. Safe to call at most once.
func (s *Subscriber[T]) Close() error {
	if err := s.sub.Unsubscribe(); err != nil {
		return fmt.Errorf("nats: unsubscribing from subject %s: %w", s.subject, err)
	}
	return nil
}
