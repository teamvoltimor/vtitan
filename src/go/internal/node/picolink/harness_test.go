package picolink_test

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"io"
	"log/slog"
	"net"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/internal/node/picolink"
	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
)

// fakeBoard is the Pico end of the link, built from boardlink itself: it
// decodes whatever the host writes and encodes whatever the test sends.
type fakeBoard struct {
	conn net.Conn
	pkts chan boardlink.Packet

	mu  sync.Mutex
	raw bytes.Buffer

	seq uint16
}

// logBuffer is a goroutine-safe sink for a text slog handler.
type logBuffer struct {
	mu  sync.Mutex
	buf bytes.Buffer
}

// statusSink, jointSink and commandSource stand in for the NATS transport.
type statusSink chan *actuationv1.MotorStatus

type jointSink chan *actuationv1.JointStates

type commandSource chan *actuationv1.AckermannCmd

// harness is one running Session over a pipe to a fakeBoard.
type harness struct {
	board   *fakeBoard
	session *picolink.Session
	status  statusSink
	joints  jointSink
	cmds    commandSource
	logs    *logBuffer
	cancel  context.CancelFunc
	done    chan error
}

const (
	// waitTimeout bounds every wait for the host to react.
	waitTimeout = 3 * time.Second
	sinkDepth   = 256
)

func (s statusSink) Publish(msg *actuationv1.MotorStatus) error {
	s <- msg
	return nil
}

func (j jointSink) Publish(msg *actuationv1.JointStates) error {
	j <- msg
	return nil
}

func (c commandSource) Read(ctx context.Context) (*actuationv1.AckermannCmd, error) {
	select {
	case cmd := <-c:
		return cmd, nil
	case <-ctx.Done():
		return nil, fmt.Errorf("fake commands: %w", ctx.Err())
	}
}

func (l *logBuffer) Write(p []byte) (int, error) {
	l.mu.Lock()
	defer l.mu.Unlock()
	n, err := l.buf.Write(p)
	if err != nil {
		return n, fmt.Errorf("log buffer: %w", err)
	}
	return n, nil
}

func (l *logBuffer) String() string {
	l.mu.Lock()
	defer l.mu.Unlock()
	return l.buf.String()
}

// newFakeBoard starts decoding conn in the background.
func newFakeBoard(conn net.Conn) *fakeBoard {
	b := &fakeBoard{conn: conn, pkts: make(chan boardlink.Packet, sinkDepth)}
	go b.readLoop()
	return b
}

func (b *fakeBoard) readLoop() {
	var (
		dec boardlink.Decoder
		pkt boardlink.Packet
	)
	buf := make([]byte, boardlink.MaxEncodedLen)
	for {
		n, err := b.conn.Read(buf)
		b.mu.Lock()
		b.raw.Write(buf[:n])
		b.mu.Unlock()
		for _, c := range buf[:n] {
			if ok, _ := dec.Feed(c, &pkt); ok {
				b.pkts <- pkt
			}
		}
		if err != nil {
			close(b.pkts)
			return
		}
	}
}

// send encodes p as the board would and writes it to the host.
func (b *fakeBoard) send(t *testing.T, p boardlink.Packet) {
	t.Helper()

	p.Seq = b.seq
	b.seq++
	frame, err := boardlink.Append(nil, &p)
	if err != nil {
		t.Fatalf("encoding %s: %v", p.Type, err)
	}
	b.write(t, frame)
}

func (b *fakeBoard) write(t *testing.T, raw []byte) {
	t.Helper()

	// A deadline, so a host that stopped reading fails the test instead of
	// hanging it.
	if err := b.conn.SetWriteDeadline(time.Now().Add(waitTimeout)); err != nil {
		t.Fatalf("board write deadline: %v", err)
	}
	if _, err := b.conn.Write(raw); err != nil {
		t.Fatalf("board write: %v", err)
	}
}

// expect waits for the next packet of type want, skipping others (the host
// pings on its own schedule).
func (b *fakeBoard) expect(t *testing.T, want boardlink.Type) boardlink.Packet {
	t.Helper()

	timeout := time.After(waitTimeout)
	for {
		select {
		case p, ok := <-b.pkts:
			if !ok {
				t.Fatalf("link closed while waiting for %s", want)
			}
			if p.Type == want {
				return p
			}
		case <-timeout:
			t.Fatalf("no %s frame from the host within %s", want, waitTimeout)
		}
	}
}

// firstByte is the first byte the host ever wrote.
func (b *fakeBoard) firstByte() (byte, bool) {
	b.mu.Lock()
	defer b.mu.Unlock()
	if b.raw.Len() == 0 {
		return 0, false
	}
	return b.raw.Bytes()[0], true
}

// startSession runs a Session over a net.Pipe to a fakeBoard until the test
// ends, and consumes the Config the session sends before hearing from the
// board, checking it is cfg.Board.
func startSession(t *testing.T, cfg picolink.SessionConfig) *harness {
	t.Helper()

	h := startSessionRaw(t, cfg)
	if got := h.board.expect(t, boardlink.TypeConfig); got.Config != cfg.Board {
		t.Fatalf("initial Config = %+v, want %+v", got.Config, cfg.Board)
	}
	return h
}

// startSessionRaw is startSession without consuming anything.
func startSessionRaw(t *testing.T, cfg picolink.SessionConfig) *harness {
	t.Helper()

	logs := &logBuffer{}
	logger := slog.New(slog.NewTextHandler(logs, &slog.HandlerOptions{Level: slog.LevelDebug}))
	h := &harness{
		status: make(statusSink, sinkDepth),
		joints: make(jointSink, sinkDepth),
		cmds:   make(commandSource),
		logs:   logs,
		done:   make(chan error, 1),
	}
	var joints picolink.JointStatesPublisher
	if cfg.Encoder != nil {
		joints = h.joints
	}
	session, err := picolink.NewSession(logger, cfg, h.status, joints)
	if err != nil {
		t.Fatalf("NewSession: %v", err)
	}
	h.session = session

	hostEnd, boardEnd := net.Pipe()
	h.board = newFakeBoard(boardEnd)

	ctx, cancel := context.WithCancel(context.Background())
	h.cancel = cancel
	go func() { h.done <- session.Run(ctx, hostEnd, h.cmds) }()

	t.Cleanup(func() {
		cancel()
		select {
		case <-h.done:
		case <-time.After(waitTimeout):
			t.Error("Session.Run did not return after cancellation")
		}
		_ = hostEnd.Close()
		_ = boardEnd.Close()
	})
	return h
}

// nextStatus waits for a MotorStatus.
func (h *harness) nextStatus(t *testing.T) *actuationv1.MotorStatus {
	t.Helper()

	select {
	case st := <-h.status:
		return st
	case <-time.After(waitTimeout):
		t.Fatal("no MotorStatus published")
		return nil
	}
}

// nextJoints waits for a JointStates.
func (h *harness) nextJoints(t *testing.T) *actuationv1.JointStates {
	t.Helper()

	select {
	case js := <-h.joints:
		return js
	case <-time.After(waitTimeout):
		t.Fatal("no JointStates published")
		return nil
	}
}

// waitLog waits until the log holds a line containing every fragment.
func (h *harness) waitLog(t *testing.T, fragments ...string) {
	t.Helper()

	deadline := time.Now().Add(waitTimeout)
	for time.Now().Before(deadline) {
		for line := range strings.SplitSeq(h.logs.String(), "\n") {
			if containsAll(line, fragments) {
				return
			}
		}
		time.Sleep(time.Millisecond)
	}
	t.Fatalf("no log line containing %q; log:\n%s", fragments, h.logs.String())
}

func containsAll(s string, fragments []string) bool {
	for _, f := range fragments {
		if !strings.Contains(s, f) {
			return false
		}
	}
	return true
}

// isClosed reports whether err is what a closed pipe returns.
func isClosed(err error) bool {
	return errors.Is(err, io.EOF) || errors.Is(err, io.ErrClosedPipe)
}
