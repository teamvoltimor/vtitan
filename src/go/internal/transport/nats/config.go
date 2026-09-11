package nats

import (
	"log/slog"
	"os"
	"time"
)

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
	// ConnectTimeout bounds a single initial connection attempt.
	ConnectTimeout time.Duration `validate:"required,gt=0"`
	// InitialConnectAttempts is how many times Connect retries the *initial*
	// dial before returning an error. nats.go's initial connect is a single
	// attempt that fails hard on a not-yet-listening server; on this
	// hardware a Pi 5 reboot cold-boots both boards over the USB-gadget link
	// (see scripts/provisioning/wait-for-gadget-link.sh) and nats-server
	// can take tens of seconds to come up. Retrying the initial dial within
	// a bounded window lets a client come up before the server and still
	// connect once it does, instead of dying at startup. -1 means keep
	// retrying while ctx is alive (used in production alongside
	// MaxReconnects=-1).
	InitialConnectAttempts int
	// ReconnectJitter/ReconnectJitterTLS spread reconnect attempts across
	// clients so they don't all hammer nats-server in lockstep after a link
	// flap (a thundering herd on the shared USB-gadget link). Each client
	// waits ReconnectWait +/- up to this much.
	ReconnectJitter    time.Duration
	ReconnectJitterTLS time.Duration
	// PingInterval/MaxPingsOutstanding let a client detect a silently-dead
	// link faster than nats.go's defaults. Over a flaky gadget link a
	// half-dead connection (socket open, no traffic) is worse than a clean
	// drop -- the client believes it's connected and drives blind. Tighten
	// both so a stale socket is recycled into a real reconnect promptly.
	PingInterval        time.Duration
	MaxPingsOutstanding int
	// Logger, if set, receives disconnect/reconnect/async-error events so the
	// operator can see (and the navigator can react to) link state changes.
	// Optional; nil disables callbacks.
	Logger *slog.Logger
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
	// DefaultInitialConnectAttempts means retry the initial dial as long as
	// the context is alive (see Config.InitialConnectAttempts). Production
	// clients use this so a cold boot that precedes nats-server still connects.
	DefaultInitialConnectAttempts = -1
	// DefaultReconnectJitter spreads reconnects by up to 1s either side of
	// ReconnectWait, enough to break lockstep without adding much latency.
	DefaultReconnectJitter = 1 * time.Second
	// DefaultReconnectJitterTLS matches ReconnectJitter for the TLS case
	// (no TLS today, but the option exists and should not be zero).
	DefaultReconnectJitterTLS = 1 * time.Second
	// DefaultPingInterval is tighter than nats.go's 2m default: a robot
	// driving blind on a silently-dead link is the failure we are avoiding.
	DefaultPingInterval = 10 * time.Second
	// DefaultMaxPingsOutstanding recycles a socket after 2 missed pings
	// (~20s at DefaultPingInterval) -- well within the 55s gadget-link
	// settle window but fast enough to stop driving blind for long.
	DefaultMaxPingsOutstanding = 2
)

// DefaultDevURL is nats-server's own default client address
// (nats-server's own out-of-the-box listen address), used across cmd/*'s
// --nats-url flag defaults for local/bench runs. This is deliberately not
// baked into DefaultConfig/Connect as an implicit URL default -- see
// Config.URL's doc comment: there is no single correct production
// nats-server address (see go_nats_migration_plan.md's "Process model":
// the Pi 5 hosts it, reachable at pi5.local:4222 from the Pi Zero), so
// every real deployment must still pass its own URL explicitly. This
// constant exists only so cmd/*'s bench-run convenience default is
// declared once, not re-typed identically in every binary's flag setup.
const DefaultDevURL = "nats://127.0.0.1:4222"

// EnvURLVar is the environment variable a deployment sets to point every Go
// binary at the broker without editing each binary's --nats-url flag. The
// systemd units source .env (which sets VTITAN_NATS_URL), so a production
// image carries its broker address in one place. Bench/local runs that pass
// --nats-url (or rely on DefaultDevURL) are unaffected.
const EnvURLVar = "VTITAN_NATS_URL"

// DefaultURL returns the broker URL to use when no --nats-url flag was given:
// VTITAN_NATS_URL from the environment if set, otherwise DefaultDevURL. This is
// the flag default for every cmd/* binary, so "no flag" means "use the deployed
// broker address, else localhost for bench".
func DefaultURL() string {
	if v := os.Getenv(EnvURLVar); v != "" {
		return v
	}
	return DefaultDevURL
}

// DefaultConfig returns a Config with sane defaults for everything except
// URL and Name, which the caller must always set explicitly -- there's no
// safe default nats-server address or client identity.
func DefaultConfig(url, name string) Config {
	return Config{
		URL:                    url,
		Name:                   name,
		ReconnectWait:          DefaultReconnectWait,
		MaxReconnects:          DefaultMaxReconnects,
		ConnectTimeout:         DefaultConnectTimeout,
		InitialConnectAttempts: DefaultInitialConnectAttempts,
		ReconnectJitter:        DefaultReconnectJitter,
		ReconnectJitterTLS:     DefaultReconnectJitterTLS,
		PingInterval:           DefaultPingInterval,
		MaxPingsOutstanding:    DefaultMaxPingsOutstanding,
	}
}
