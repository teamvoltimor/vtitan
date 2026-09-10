package supervise_test

import (
	"net"
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

// listenAbstractNotifySocket binds a listener directly in the Linux
// abstract socket namespace (a leading NUL byte in the address, not a
// filesystem path) -- the actual target Notify's "@" rewrite dials, and a
// wholly disjoint address space from any path-based socket: replacing "@"
// with NUL never resolves back to a real filesystem path (e.g. "@/tmp/x"
// becomes "\x00/tmp/x", which the kernel treats as the abstract name
// "/tmp/x", not a lookup of the file "/tmp/x"), so this is the only way to
// exercise that code path for real rather than by coincidence.
func listenAbstractNotifySocket(t *testing.T) (name string, listener *net.UnixConn) {
	t.Helper()

	name = "\x00vtitan-test-notify-" + t.Name()
	addr := &net.UnixAddr{Name: name, Net: "unixgram"}

	listener, err := net.ListenUnixgram("unixgram", addr)
	if err != nil {
		t.Skipf("abstract unixgram sockets unavailable on this platform: %v", err)
	}
	t.Cleanup(func() { _ = listener.Close() })

	return name, listener
}

func TestNotify_AbstractSocketPrefixTranslatesToNULByte(t *testing.T) {
	name, listener := listenAbstractNotifySocket(t)
	t.Setenv("NOTIFY_SOCKET", "@"+name[1:])

	if err := supervise.Notify(t.Context(), supervise.WatchdogState); err != nil {
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
