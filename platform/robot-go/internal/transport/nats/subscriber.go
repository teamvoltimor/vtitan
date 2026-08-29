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

// NewSubscriber subscribes to subject on conn. newT constructs a fresh,
// empty T for each received message to unmarshal into -- required because
// T is constrained to the proto.Message interface, not a concrete type
// Go's generics can instantiate directly (the classic reason a factory
// closure is needed here instead of `new(T)`).
func NewSubscriber[T proto.Message](conn *nats.Conn, subject string, newT func() T) (*Subscriber[T], error) {
	sub, err := conn.SubscribeSync(subject)
	if err != nil {
		return nil, fmt.Errorf("nats: subscribing to subject %s: %w", subject, err)
	}
	return &Subscriber[T]{sub: sub, subject: subject, newT: newT}, nil
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
