package supervise

import (
	"time"

	"github.com/teamvoltimor/vtitan/src/go/internal/statemachine/backoff"
)

// Config parameterizes a Supervisor. Every field here is a real
// deployment-tunable (restart backoff timing, how long a goroutine must run
// before it's considered healthy again) -- not a fixed protocol fact, so it
// belongs in Config rather than a package-level constant.
type Config struct {
	// Backoff controls the delay between restart attempts after fn returns
	// an error or panics.
	Backoff backoff.Config `validate:"required"`
	// HealthyDuration is how long fn must run continuously before a
	// subsequent failure resets the backoff delay back to Backoff.Initial,
	// matching backoff.Backoff.Reset's own "reset after a successful
	// connection" semantics -- without this, a goroutine that has been
	// running fine for hours inherits whatever backoff delay it was at the
	// last time it flapped, which could be Backoff.Max from long ago.
	HealthyDuration time.Duration `validate:"required,gt=0"`
}

// DefaultHealthyDuration is a conservative "this goroutine is healthy
// again" threshold: long enough that a goroutine crash-looping within a
// single bench/race run never sees its backoff reset mid-run, short enough
// that a goroutine which failed once, then ran cleanly for a while, isn't
// stuck at Backoff.Max for the rest of the process's lifetime.
const DefaultHealthyDuration = 5 * time.Minute

// DefaultConfig returns a Config built from backoff.DefaultConfig and
// DefaultHealthyDuration.
func DefaultConfig() Config {
	return Config{
		Backoff:         backoff.DefaultConfig(),
		HealthyDuration: DefaultHealthyDuration,
	}
}
