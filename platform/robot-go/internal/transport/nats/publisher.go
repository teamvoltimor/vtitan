package nats

import (
	"fmt"

	"github.com/nats-io/nats.go"
	"google.golang.org/protobuf/proto"
)

// Publisher publishes protobuf messages of type T to a fixed NATS subject.
type Publisher[T proto.Message] struct {
	conn    *nats.Conn
	subject string
}

// NewPublisher returns a Publisher bound to subject on conn. conn must
// already be connected (see Connect).
func NewPublisher[T proto.Message](conn *nats.Conn, subject string) *Publisher[T] {
	return &Publisher[T]{conn: conn, subject: subject}
}

// Publish encodes msg as protobuf and publishes it to the Publisher's
// subject.
func (p *Publisher[T]) Publish(msg T) error {
	data, err := proto.Marshal(msg)
	if err != nil {
		return fmt.Errorf("nats: marshaling message for subject %s: %w", p.subject, err)
	}
	if err = p.conn.Publish(p.subject, data); err != nil {
		return fmt.Errorf("nats: publishing to subject %s: %w", p.subject, err)
	}
	return nil
}
