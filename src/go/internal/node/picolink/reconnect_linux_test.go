//go:build linux

package picolink_test

import (
	"context"
	"errors"
	"log/slog"
	"os"
	"path/filepath"
	"sync"
	"testing"
	"time"

	"github.com/creack/pty"
	natstest "github.com/nats-io/nats-server/v2/test"
	"golang.org/x/sys/unix"

	"github.com/teamvoltimor/vtitan/src/go/internal/node/picolink"
	"github.com/teamvoltimor/vtitan/src/go/pkg/backoff"
	"github.com/teamvoltimor/vtitan/src/go/pkg/boardsim"
	"github.com/teamvoltimor/vtitan/src/go/pkg/supervise"
	"github.com/teamvoltimor/vtitan/src/go/pkg/transport/nats"
)

// These tests run the production path, picolink.Run under pkg/supervise,
// against a virtual board on a pseudo-terminal, so the serial port is a
// real tty device that can disappear and come back under another name, as
// a replugged USB board does.

// ptyBoard is a virtual board serving the master side of a pseudo-terminal
// whose slave is the device the host opens.
type ptyBoard struct {
	board  *boardsim.Board
	master *os.File
	// slave is held open until unplug: with no slave open, the master's
	// reads fail and the board would stop before the host opened it.
	slave  *os.File
	done   chan error
	cancel context.CancelFunc
	once   sync.Once
	// device is the slave's path, e.g. /dev/pts/7.
	device string
}

// plugBoard starts a virtual board on a fresh pseudo-terminal.
func plugBoard(t *testing.T, bootID uint32) *ptyBoard {
	t.Helper()

	ptmx, slave, err := pty.Open()
	if err != nil {
		t.Skipf("no pseudo-terminals here: %v", err)
	}
	master := pollable(t, ptmx)
	// The host sets raw mode when it opens the port, but the board may
	// write first; in the default cooked mode the tty would echo those
	// bytes back into the board.
	makeRaw(t, slave)
	device := slave.Name()

	board, err := boardsim.New(master, boardsim.Options{BootID: bootID})
	if err != nil {
		t.Fatalf("boardsim.New: %v", err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	b := &ptyBoard{
		board:  board,
		master: master,
		slave:  slave,
		done:   make(chan error, 1),
		cancel: cancel,
		device: device,
	}
	go func() { b.done <- board.Run(ctx) }()
	t.Cleanup(b.unplug)
	return b
}

// unplug stops the board and closes the master, which hangs up the tty for
// the host as unplugging a USB device does. Safe to call more than once.
func (b *ptyBoard) unplug() {
	b.once.Do(func() {
		b.cancel()
		_ = b.master.Close()
		_ = b.slave.Close()
		select {
		case <-b.done:
		case <-time.After(waitTimeout):
		}
	})
}

// pollable returns a non-blocking copy of f and closes f. pty.Open leaves
// the master in blocking mode (it calls Fd), and Close cannot interrupt a
// blocking Read: the fd would only really close, hanging up the tty, once
// the board's pending read returned, on the host's next ping. A pollable
// file closes at once, as a USB unplug does.
func pollable(t *testing.T, f *os.File) *os.File {
	t.Helper()

	fd, err := unix.Dup(int(f.Fd()))
	if err != nil {
		t.Fatalf("duplicating %s: %v", f.Name(), err)
	}
	if err = f.Close(); err != nil {
		t.Fatal(err)
	}
	if err = unix.SetNonblock(fd, true); err != nil {
		t.Fatalf("making %s non-blocking: %v", f.Name(), err)
	}
	return os.NewFile(uintptr(fd), f.Name())
}

func makeRaw(t *testing.T, f *os.File) {
	t.Helper()

	fd := int(f.Fd())
	tio, err := unix.IoctlGetTermios(fd, unix.TCGETS)
	if err != nil {
		t.Fatalf("reading termios: %v", err)
	}
	tio.Iflag &^= unix.IGNBRK | unix.BRKINT | unix.PARMRK | unix.ISTRIP | unix.INLCR | unix.IGNCR | unix.ICRNL | unix.IXON
	tio.Oflag &^= unix.OPOST
	tio.Lflag &^= unix.ECHO | unix.ECHONL | unix.ICANON | unix.ISIG | unix.IEXTEN
	tio.Cflag &^= unix.CSIZE | unix.PARENB
	tio.Cflag |= unix.CS8
	if err = unix.IoctlSetTermios(fd, unix.TCSETS, tio); err != nil {
		t.Fatalf("setting raw mode: %v", err)
	}
}

// pointAt makes link a symlink to target, replacing it atomically, as udev
// does when a device reappears.
func pointAt(t *testing.T, link, target string) {
	t.Helper()

	tmp := link + ".new"
	_ = os.Remove(tmp)
	if err := os.Symlink(target, tmp); err != nil {
		t.Fatal(err)
	}
	if err := os.Rename(tmp, link); err != nil {
		t.Fatal(err)
	}
}

// runLink runs picolink.Run on port under pkg/supervise with a short
// backoff until the test ends, and returns what each attempt returned.
func runLink(t *testing.T, port string) func() []error {
	t.Helper()

	opts := natstest.DefaultTestOptions
	opts.Port = -1
	srv := natstest.RunServer(&opts)
	t.Cleanup(srv.Shutdown)

	var (
		mu       sync.Mutex
		attempts []error
	)
	logger := slog.New(slog.DiscardHandler)
	sup, err := supervise.New(supervise.Config{
		Backoff:         backoff.Config{Initial: 20 * time.Millisecond, Max: 100 * time.Millisecond},
		HealthyDuration: time.Minute,
	}, logger)
	if err != nil {
		t.Fatalf("supervise.New: %v", err)
	}
	cfg := picolink.Config{
		Port:    port,
		NATS:    nats.DefaultConfig(srv.ClientURL(), t.Name()),
		Session: picolink.SessionConfig{Board: sampleBoard},
	}
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan struct{})
	go func() {
		defer close(done)
		_ = sup.RunAll(ctx, supervise.Target{Name: "picolink", Fn: func(ctx context.Context) error {
			runErr := picolink.Run(ctx, cfg, logger)
			mu.Lock()
			attempts = append(attempts, runErr)
			mu.Unlock()
			return runErr //nolint:wrapcheck // the production target returns Run's error as is
		}})
	}()
	t.Cleanup(func() {
		cancel()
		<-done
	})
	return func() []error {
		mu.Lock()
		defer mu.Unlock()
		return append([]error(nil), attempts...)
	}
}

// A board that is unplugged and comes back under another device name, with
// the stable symlink repointed at it, is reconfigured by the same running
// process: the lost port makes Run return, which closes it, and the
// supervised restart opens the symlink afresh.
func TestReconnect_RenamedDeviceIsServedAgain(t *testing.T) {
	t.Parallel()

	link := filepath.Join(t.TempDir(), "vtitan-actuation-board")
	first := plugBoard(t, 1)
	pointAt(t, link, first.device)
	attempts := runLink(t, link)

	waitFor(t, "the first board configured", func() bool {
		_, ok := first.board.Configured()
		return ok
	})

	// The replacement exists before the first is unplugged, so the kernel
	// cannot hand it the same pts number: the device really is renamed.
	second := plugBoard(t, 2)
	if second.device == first.device {
		t.Fatalf("both boards on %s, want distinct devices", first.device)
	}
	first.unplug()
	pointAt(t, link, second.device)

	waitFor(t, "the renamed board configured", func() bool {
		_, ok := second.board.Configured()
		return ok
	})
	if got, ok := second.board.Configured(); !ok || got != sampleBoard {
		t.Errorf("renamed board Config = %+v, want %+v", got, sampleBoard)
	}

	errs := attempts()
	if len(errs) == 0 {
		t.Fatal("Run never returned after the unplug, want it to give the dead port up")
	}
	for _, err := range errs {
		if err == nil || errors.Is(err, context.Canceled) {
			t.Errorf("Run returned %v while the test ran, want a port error", err)
		}
	}
}

// While the symlink points nowhere (the board is unplugged and not yet
// back), each attempt fails to open and is retried, and the board is
// served as soon as it reappears.
func TestReconnect_AbsentDeviceIsRetriedUntilItAppears(t *testing.T) {
	t.Parallel()

	link := filepath.Join(t.TempDir(), "vtitan-actuation-board")
	attempts := runLink(t, link)
	waitFor(t, "two failed opens", func() bool { return len(attempts()) >= 2 })

	board := plugBoard(t, 3)
	pointAt(t, link, board.device)
	waitFor(t, "the board configured", func() bool {
		_, ok := board.board.Configured()
		return ok
	})
}
