package nats

import (
	"fmt"

	"github.com/go-playground/validator/v10"
	"github.com/nats-io/nats.go"
)

// Connect opens a connection to nats-server per cfg. Reconnection is handled
// natively by nats.go (ReconnectWait/MaxReconnects options below) rather
// than by a second, custom backoff layer on top -- nats.go's reconnect
// logic is exactly what NATS is designed around and re-implementing it
// would be redundant, not an improvement. internal/statemachine/backoff
// exists for a different concern (retrying a higher-level operation), not
// this connection itself.
func Connect(cfg Config) (*nats.Conn, error) {
	if err := validator.New().Struct(cfg); err != nil {
		return nil, fmt.Errorf("nats: invalid config: %w", err)
	}

	conn, err := nats.Connect(
		cfg.URL,
		nats.Name(cfg.Name),
		nats.ReconnectWait(cfg.ReconnectWait),
		nats.MaxReconnects(cfg.MaxReconnects),
		nats.Timeout(cfg.ConnectTimeout),
	)
	if err != nil {
		return nil, fmt.Errorf("nats: connecting to %s: %w", cfg.URL, err)
	}
	return conn, nil
}
