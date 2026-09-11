package supervise

import (
	"context"
	"fmt"
	"net"
	"os"
	"strings"
)

// notifySocketEnv is the fixed sd_notify(3) protocol env var systemd sets
// on a supervised unit -- a protocol fact, not a deployment tunable, so
// it's a package constant rather than a Config field.
const notifySocketEnv = "NOTIFY_SOCKET"

// ReadyState, WatchdogState and StoppingState are the sd_notify(3) payload
// strings this package sends. They're the protocol's own wire format, not
// something a caller should ever need to spell differently.
const (
	ReadyState    = "READY=1"
	WatchdogState = "WATCHDOG=1"
	StoppingState = "STOPPING=1"
)

// Notify sends state to systemd via the sd_notify(3) protocol. If
// NOTIFY_SOCKET is unset -- true for essentially every dev/bench run,
// which isn't launched under systemd at all -- this is a safe no-op rather
// than an error, so callers can call it unconditionally.
//
// A NOTIFY_SOCKET value starting with "@" denotes a Linux abstract socket;
// sd_notify(3) specifies this is translated to a leading NUL byte before
// the connect(2) call, not the literal "@" character.
//
// ctx bounds the dial only -- a unixgram "connect" is really just binding
// a local socket and recording the peer address locally, not a network
// round trip, so it's expected to return effectively immediately either
// way. Taking ctx is about this package's own I/O-does-a-context
// convention (see contextcheck in .golangci.yml) more than an actual need
// to cancel a slow dial.
func Notify(ctx context.Context, state string) (err error) {
	socketPath := os.Getenv(notifySocketEnv)
	if socketPath == "" {
		return nil
	}
	if strings.HasPrefix(socketPath, "@") {
		socketPath = "\x00" + socketPath[1:]
	}

	var dialer net.Dialer
	conn, err := dialer.DialContext(ctx, "unixgram", socketPath)
	if err != nil {
		return fmt.Errorf("supervise: dialing NOTIFY_SOCKET: %w", err)
	}
	defer func() {
		if closeErr := conn.Close(); closeErr != nil && err == nil {
			err = fmt.Errorf("supervise: closing NOTIFY_SOCKET connection: %w", closeErr)
		}
	}()

	if _, err = conn.Write([]byte(state)); err != nil {
		return fmt.Errorf("supervise: writing sd_notify state: %w", err)
	}
	return nil
}
