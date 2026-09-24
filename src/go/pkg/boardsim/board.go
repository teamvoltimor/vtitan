package boardsim

import (
	"context"
	"errors"
	"fmt"
	"io"
	"sync"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardloop"
)

// Options configure a Board.
type Options struct {
	// BootID and BootFaults are reported in Hello and Status, as
	// boardloop.Options.
	BootID     uint32
	BootFaults boardlink.Faults
	// Tick is the step period; zero means DefaultTick.
	Tick time.Duration
	// CountsPerSecondAtFullDuty drives the Encoder from the commanded
	// duty; zero leaves the count to the test (Encoder.Add).
	CountsPerSecondAtFullDuty float64
	// NoEncoder and NoButton build a board without that hardware, so the
	// Loop sends no Odometry or Button messages.
	NoEncoder bool
	NoButton  bool
}

// Board is a virtual actuation board. Build it with New, run it with Run,
// and inspect or poke its hardware through the exported fields while it
// runs.
type Board struct {
	Drive   *Drive
	Servo   *Servo
	Encoder *Encoder
	Button  *Button

	link *link
	opts Options

	// mu guards loop, which is not safe for concurrent use.
	mu   sync.Mutex
	loop *boardloop.Loop
}

// link adapts an io.ReadWriter to boardloop.Link: a goroutine drains the
// read side into buf, so Read never blocks.
type link struct {
	rw io.ReadWriter

	mu   sync.Mutex
	buf  []byte
	err  error
	done chan struct{}
}

// writeDeadliner is the part of net.Conn a link uses when it has it.
type writeDeadliner interface {
	SetWriteDeadline(t time.Time) error
}

// DefaultTick is how often Run steps the Loop. The firmware steps in a
// tight loop; 1 ms is well under every boardlink interval and command
// timeout the host configures.
const DefaultTick = time.Millisecond

// writeTimeout bounds one frame write, so a host that stopped reading makes
// Run return instead of hanging it.
const writeTimeout = time.Second

// ErrLinkClosed is returned by Run when the link's read side ends.
var ErrLinkClosed = errors.New("boardsim: link closed")

// New builds an unconfigured Board over rw. It starts reading rw at once.
func New(rw io.ReadWriter, opts Options) (*Board, error) {
	if opts.Tick <= 0 {
		opts.Tick = DefaultTick
	}
	b := &Board{
		Drive:   &Drive{},
		Servo:   &Servo{},
		Encoder: &Encoder{},
		Button:  &Button{},
		link:    &link{rw: rw, done: make(chan struct{})},
		opts:    opts,
	}
	hw := boardloop.Hardware{Link: b.link, Drive: b.Drive, Servo: b.Servo}
	if !opts.NoEncoder {
		hw.Encoder = b.Encoder
	}
	if !opts.NoButton {
		hw.Button = b.Button
	}
	loop, err := boardloop.New(hw, boardloop.Options{BootID: opts.BootID, BootFaults: opts.BootFaults})
	if err != nil {
		return nil, fmt.Errorf("boardsim: %w", err)
	}
	b.loop = loop
	go b.link.pump()
	return b, nil
}

// Run steps the Loop every Tick, with the time since Run started as the
// board's clock, until ctx ends (nil) or the link closes (ErrLinkClosed).
func (b *Board) Run(ctx context.Context) error {
	ticker := time.NewTicker(b.opts.Tick)
	defer ticker.Stop()

	boot := time.Now()
	last := time.Duration(0)
	for {
		select {
		case <-ctx.Done():
			return nil
		case <-b.link.done:
			return ErrLinkClosed
		case <-ticker.C:
		}
		now := time.Since(boot)
		// boardloop.Loop.Step takes no ctx: the firmware has none to give it.
		b.step(now, now-last) //nolint:contextcheck // see above
		last = now
	}
}

// Configured returns the Config in force and whether the board is
// configured.
func (b *Board) Configured() (boardlink.Config, bool) {
	b.mu.Lock()
	defer b.mu.Unlock()
	return b.loop.Config()
}

// Counters returns the Loop's diagnostic counters.
func (b *Board) Counters() boardloop.Counters {
	b.mu.Lock()
	defer b.mu.Unlock()
	return b.loop.Counters()
}

// step advances the wheel model over dt at the duty in force, then runs
// one Loop iteration at now.
func (b *Board) step(now, dt time.Duration) {
	if b.opts.CountsPerSecondAtFullDuty != 0 && !b.opts.NoEncoder {
		b.Encoder.advance(b.Drive.Duty()*b.opts.CountsPerSecondAtFullDuty, dt.Seconds())
	}
	b.mu.Lock()
	defer b.mu.Unlock()
	b.loop.Step(now)
}

// Read copies buffered bytes into p without blocking. Once the read side
// has ended and the buffer is empty it returns the read error.
func (l *link) Read(p []byte) (int, error) {
	l.mu.Lock()
	defer l.mu.Unlock()
	n := copy(p, l.buf)
	l.buf = l.buf[n:]
	if n == 0 && l.err != nil {
		return 0, l.err
	}
	return n, nil
}

// Write sends one frame, bounded by writeTimeout when rw supports
// deadlines.
func (l *link) Write(p []byte) (int, error) {
	if d, ok := l.rw.(writeDeadliner); ok {
		if err := d.SetWriteDeadline(time.Now().Add(writeTimeout)); err != nil {
			return 0, fmt.Errorf("boardsim: write deadline: %w", err)
		}
	}
	n, err := l.rw.Write(p)
	if err != nil {
		return n, fmt.Errorf("boardsim: write: %w", err)
	}
	return n, nil
}

// pump drains rw into buf until it fails, then closes done.
func (l *link) pump() {
	defer close(l.done)
	chunk := make([]byte, boardlink.ReadChunkSize)
	for {
		n, err := l.rw.Read(chunk)
		l.mu.Lock()
		l.buf = append(l.buf, chunk[:n]...)
		if err != nil {
			l.err = fmt.Errorf("boardsim: read: %w", err)
		}
		l.mu.Unlock()
		if err != nil {
			return
		}
	}
}
