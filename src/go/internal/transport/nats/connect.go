package nats

import (
	"context"
	"fmt"
	"time"

	"github.com/go-playground/validator/v10"
	"github.com/nats-io/nats.go"
)

// Connect opens a connection to nats-server per cfg. Reconnection (after a
// successful initial connect) is handled natively by nats.go
// (ReconnectWait/MaxReconnects/ReconnectJitter options below) rather than by a
// second, custom backoff layer on top -- nats.go's reconnect logic is exactly
// what NATS is designed around and re-implementing it would be redundant, not
// an improvement. internal/statemachine/backoff exists for a different concern
// (retrying a higher-level operation), not this connection itself.
//
// The *initial* dial is retried up to cfg.InitialConnectAttempts times (or
// indefinitely while ctx is alive when that is -1). nats.go's initial connect
// is a single attempt that fails hard on a not-yet-listening server; on this
// hardware a Pi 5 reboot cold-boots both boards over the USB-gadget link and
// nats-server can take tens of seconds to come up (see
// scripts/provisioning/wait-for-gadget-link.sh), so retrying the dial lets a
// client that starts first still connect once the server is ready, instead of
// dying at startup.
func Connect(ctx context.Context, cfg Config) (*nats.Conn, error) {
	if err := validator.New().Struct(cfg); err != nil {
		return nil, fmt.Errorf("nats: invalid config: %w", err)
	}

	opts := []nats.Option{
		nats.Name(cfg.Name),
		nats.ReconnectWait(cfg.ReconnectWait),
		nats.MaxReconnects(cfg.MaxReconnects),
		nats.ReconnectJitter(cfg.ReconnectJitter, cfg.ReconnectJitterTLS),
		nats.Timeout(cfg.ConnectTimeout),
		nats.PingInterval(cfg.PingInterval),
		nats.MaxPingsOutstanding(cfg.MaxPingsOutstanding),
	}
	if cfg.Logger != nil {
		log := cfg.Logger
		opts = append(opts,
			nats.DisconnectErrHandler(func(c *nats.Conn, err error) {
				if err != nil {
					log.Warn("nats: disconnected", "url", cfg.URL, "error", err)
				} else {
					log.Warn("nats: disconnected", "url", cfg.URL)
				}
			}),
			nats.ReconnectHandler(func(c *nats.Conn) {
				log.Info("nats: reconnected", "url", c.ConnectedUrl())
			}),
		)
	}

	var lastErr error
	attempts := 0
	for {
		attempts++
		conn, err := nats.Connect(cfg.URL, opts...)
		if err == nil {
			return conn, nil
		}
		lastErr = err

		if cfg.InitialConnectAttempts >= 0 && attempts >= cfg.InitialConnectAttempts {
			break
		}
		if ctx.Err() != nil {
			break
		}

		// Back off a bounded amount before the next dial, but wake immediately
		// if the context is canceled. Uses the same ReconnectWait as nats.go's
		// own reconnect cadence so initial-dial retries and later reconnects
		// look identical to an operator reading the logs.
		select {
		case <-ctx.Done():
			return nil, fmt.Errorf("nats: initial connect to %s canceled: %w", cfg.URL, ctx.Err())
		case <-time.After(cfg.ReconnectWait):
		}
	}
	return nil, fmt.Errorf("nats: connecting to %s (after %d attempt(s)): %w", cfg.URL, attempts, lastErr)
}
