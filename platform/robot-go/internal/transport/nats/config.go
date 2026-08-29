package nats

import "time"

// Config configures a connection to nats-server. Every field here is a real
// deployment-tunable (server address, client identity, reconnect timing) --
// not a protocol constant, so all of it belongs in Config rather than
// hardcoded.
type Config struct {
	// URL is the nats-server address, e.g. "nats://pi5.local:4222". Required.
	URL string `validate:"required"`
	// Name identifies this client to the server (visible in nats-server's
	// own monitoring/connz output) -- set it to the binary name
	// (cmd/pi5, cmd/pi-zero, cmd/lidar-node, ...) so a connection shows up
	// as something a human can actually identify.
	Name string `validate:"required"`
	// ReconnectWait is how long the client waits between reconnect attempts
	// after losing the connection.
	ReconnectWait time.Duration `validate:"required,gt=0"`
	// MaxReconnects caps how many consecutive reconnect attempts nats.go
	// makes before giving up. -1 means unlimited, which is the right
	// default for a robot that should keep trying to reach nats-server for
	// as long as it's powered on, not give up after N tries.
	MaxReconnects int
	// ConnectTimeout bounds the initial connection attempt.
	ConnectTimeout time.Duration `validate:"required,gt=0"`
}

const (
	// DefaultReconnectWait matches nats.go's own default, stated explicitly
	// here rather than left as an unstated library default -- see the style
	// preference for named constants over implicit values.
	DefaultReconnectWait = 2 * time.Second
	// DefaultMaxReconnects means unlimited (see Config.MaxReconnects).
	DefaultMaxReconnects = -1
	// DefaultConnectTimeout matches nats.go's own default.
	DefaultConnectTimeout = 2 * time.Second
)

// DefaultConfig returns a Config with sane defaults for everything except
// URL and Name, which the caller must always set explicitly -- there's no
// safe default nats-server address or client identity.
func DefaultConfig(url, name string) Config {
	return Config{
		URL:            url,
		Name:           name,
		ReconnectWait:  DefaultReconnectWait,
		MaxReconnects:  DefaultMaxReconnects,
		ConnectTimeout: DefaultConnectTimeout,
	}
}
