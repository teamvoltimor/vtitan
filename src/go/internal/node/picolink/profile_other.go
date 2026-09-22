//go:build !linux

package picolink

import (
	"errors"
	"log/slog"
	"time"
)

// DefaultCommandTimeout mirrors internal/node/motor.DefaultCommandTimeout,
// which is built only on Linux. It exists so cmd/pi5 compiles on a dev
// machine; the Linux build, the only one that drives a car, uses the Zero's
// constant directly.
const DefaultCommandTimeout = 500 * time.Millisecond

// LoadProfile always fails off Linux: the resolvers it shares with the Zero
// (internal/node/motor) are Linux-only.
func LoadProfile(_ *slog.Logger, _ string) (Profile, error) {
	return Profile{}, errors.New("picolink: loading the board profile needs the Linux build")
}
