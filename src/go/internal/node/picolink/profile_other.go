//go:build !linux

package picolink

import (
	"errors"
	"log/slog"

	nodemotor "github.com/teamvoltimor/vtitan/src/go/internal/node/motor"
)

// DefaultCommandTimeout mirrors internal/node/motor.DefaultCommandTimeout, so
// cmd/pi5 compiles on a dev machine with the same value the Linux build, the
// only one that drives a car, uses. It reads that constant directly rather
// than repeating its value; the resolvers LoadProfile needs stay Linux-only.
const DefaultCommandTimeout = nodemotor.DefaultCommandTimeout

// LoadProfile always fails off Linux: the resolvers it shares with the Zero
// (internal/node/motor) are Linux-only.
func LoadProfile(_ *slog.Logger, _ string) (Profile, error) {
	return Profile{}, errors.New("picolink: loading the board profile needs the Linux build")
}
