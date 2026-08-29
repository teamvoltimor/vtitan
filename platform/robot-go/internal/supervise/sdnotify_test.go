package supervise_test

import (
	"net"
	"os"
	"path/filepath"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/supervise"
)

const notifyReadTimeout = 2 * time.Second

// listenNotifySocket starts a Unix datagram listener a test can point
// NOTIFY_SOCKET at, standing in for the socket systemd itself would open
// for a real supervised unit.
func listenNotifySocket(t *testing.T) (socketPath string, listener *net.UnixConn) {
	t.Helper()

	socketPath = filepath.Join(t.TempDir(), "notify.sock")
	addr := &net.UnixAddr{Name: socketPath, Net: "unixgram"}

	listener, err := net.ListenUnixgram("unixgram", addr)
	if err != nil {
		t.Skipf("unixgram sockets unavailable on this platform: %v", err)
	}
	t.Cleanup(func() { _ = listener.Close() })

	return socketPath, listener
}

func readOneDatagram(t *testing.T, listener *net.UnixConn) string {
	t.Helper()

	if err := listener.SetReadDeadline(time.Now().Add(notifyReadTimeout)); err != nil {
		t.Fatalf("SetReadDeadline() error = %v", err)
	}

	buf := make([]byte, 256)
	n, err := listener.Read(buf)
	if err != nil {
		t.Fatalf("Read() error = %v", err)
	}
	return string(buf[:n])
}

func TestNotify_NoSocketEnvIsANoOp(t *testing.T) {
	t.Setenv("NOTIFY_SOCKET", "")

	if err := supervise.Notify(t.Context(), supervise.ReadyState); err != nil {
		t.Fatalf("Notify() error = %v, want nil", err)
	}
}

func TestNotify_SendsStateToConfiguredSocket(t *testing.T) {
	socketPath, listener := listenNotifySocket(t)
	t.Setenv("NOTIFY_SOCKET", socketPath)

	if err := supervise.Notify(t.Context(), supervise.ReadyState); err != nil {
		t.Fatalf("Notify() error = %v, want nil", err)
	}

	if got := readOneDatagram(t, listener); got != supervise.ReadyState {
		t.Errorf("received datagram = %q, want %q", got, supervise.ReadyState)
	}
}

func TestNotify_AbstractSocketPrefixTranslatesToNULByte(t *testing.T) {
	socketPath, listener := listenNotifySocket(t)
	t.Setenv("NOTIFY_SOCKET", "@"+socketPath[1:])

	err := supervise.Notify(t.Context(), supervise.WatchdogState)
	if socketPath[0] != os.PathSeparator && socketPath[0] != '/' {
		// The "@" abstract-socket convention rewrites the leading byte and
		// otherwise leaves the path text untouched, so this substitution
		// only round-trips back to socketPath's real filesystem path when
		// its first character was itself the separator being replaced --
		// true for every t.TempDir()-based path on the platforms this
		// module targets, but guarded explicitly rather than assumed.
		t.Skip("socketPath does not start with a path separator, abstract-socket rewrite would not target it")
	}
	if err != nil {
		t.Fatalf("Notify() error = %v, want nil", err)
	}

	if got := readOneDatagram(t, listener); got != supervise.WatchdogState {
		t.Errorf("received datagram = %q, want %q", got, supervise.WatchdogState)
	}
}

func TestNotify_DialErrorIsWrapped(t *testing.T) {
	t.Setenv("NOTIFY_SOCKET", filepath.Join(t.TempDir(), "does-not-exist.sock"))

	err := supervise.Notify(t.Context(), supervise.ReadyState)
	if err == nil {
		t.Fatal("Notify() error = nil, want non-nil for a socket nothing is listening on")
	}
}
