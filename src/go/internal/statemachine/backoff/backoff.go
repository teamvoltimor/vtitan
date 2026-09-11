package backoff

import "time"

// Config parameterizes a Backoff's timing. Every field is a real
// Config-worthy tunable -- see doc.go.
type Config struct {
	// Initial is the delay before the first reconnect attempt, and the
	// delay Reset restores.
	Initial time.Duration `validate:"required,gt=0"`
	// Max is the ceiling the doubling delay is capped at.
	Max time.Duration `validate:"required,gtfield=Initial"`
}

// Backoff is a stateful exponential-backoff delay generator: each call to
// Next returns the current delay and then doubles it, capped at
// cfg.Max, matching both Python channels' identical
// `min(delay * 2, BACKOFF_MAX_S)` doubling. Not safe for concurrent use --
// callers own one Backoff per reconnect loop, matching the Python
// original's one `_command_backoff_delay`/`backoff` local per channel.
type Backoff struct {
	cfg     Config
	current time.Duration
}

// DefaultInitial and DefaultMax match grpc_backoff.py's
// BACKOFF_INITIAL_S/BACKOFF_MAX_S exactly (1s / 60s).
const (
	DefaultInitial = 1 * time.Second
	DefaultMax     = 60 * time.Second
)

// DefaultConfig returns the Config matching grpc_backoff.py's constants.
func DefaultConfig() Config {
	return Config{Initial: DefaultInitial, Max: DefaultMax}
}

// New builds a Backoff starting at cfg.Initial. cfg is assumed already
// validated, matching this project's constructor convention (validation
// happens once at the call site, not silently inside every constructor).
func New(cfg Config) *Backoff {
	return &Backoff{cfg: cfg, current: cfg.Initial}
}

// Next returns the delay to wait before the next reconnect attempt, then
// doubles it (capped at cfg.Max) for the following call.
func (b *Backoff) Next() time.Duration {
	delay := b.current
	b.current = min(b.current*2, b.cfg.Max)
	return delay
}

// Reset restores the delay to cfg.Initial, matching both Python channels
// resetting their backoff local back to BACKOFF_INITIAL_S after a
// successful connection (command_channel.py does this per received
// command; telemetry_ingest_channel.py's stream loop does it implicitly
// by re-entering _stream_loop's while body after a successful, since
// backoff there is scoped per _stream_loop call -- Backoff.Reset gives
// callers the explicit equivalent either shape needs).
func (b *Backoff) Reset() {
	b.current = b.cfg.Initial
}
