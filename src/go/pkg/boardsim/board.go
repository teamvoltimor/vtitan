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
	// Link emulates the link's delay and corruption; the zero value is an
	// ideal link.
	Link LinkConfig
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

	// mu guards loop, which is not safe for concurrent use, and the boot
	// clock, which Reboot restarts.
	mu       sync.Mutex
	loop     *boardloop.Loop
	bootAt   time.Time
	lastStep time.Duration
}

// DefaultTick is how often Run steps the Loop. The firmware steps in a
// tight loop; 1 ms is well under every boardlink interval and command
// timeout the host configures.
const DefaultTick = time.Millisecond

const (
	// writeTimeout bounds one frame write, so a host that stopped reading
	// ends the link instead of hanging it.
	writeTimeout = time.Second
	// readChunk is the read size a board's link reader asks for.
	readChunk = boardlink.ReadChunkSize
)

// ErrLinkClosed is returned by Run when the link's read side ends.
var ErrLinkClosed = errors.New("boardsim: link closed")

// New builds an unconfigured Board over rw. It starts reading rw at once;
// close rw to release the link's goroutines.
func New(rw io.ReadWriter, opts Options) (*Board, error) {
	if opts.Tick < 0 {
		return nil, fmt.Errorf("boardsim: tick %v must not be negative", opts.Tick)
	}
	if opts.Tick == 0 {
		opts.Tick = DefaultTick
	}
	if err := opts.Link.Validate(); err != nil {
		return nil, err
	}
	b := &Board{
		Drive:   &Drive{},
		Servo:   &Servo{},
		Encoder: &Encoder{},
		Button:  &Button{},
		opts:    opts,
	}
	b.link = newLink(rw, opts.Link)
	loop, err := b.newLoop(opts.BootID, opts.BootFaults)
	if err != nil {
		return nil, err
	}
	b.loop = loop
	b.bootAt = time.Now()
	return b, nil
}

// Run steps the Loop every Tick, with the time since boot as the board's
// clock, until ctx ends (nil) or the link closes (ErrLinkClosed).
func (b *Board) Run(ctx context.Context) error {
	ticker := time.NewTicker(b.opts.Tick)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			return nil
		case <-b.link.done:
			return ErrLinkClosed
		case <-ticker.C:
		}
		// boardloop.Loop.Step takes no ctx: the firmware has none to give it.
		b.step() //nolint:contextcheck // see above
	}
}

// Reboot emulates a board reset, as a watchdog or a brownout causes: a new
// boot reporting bootID and faults (boardlink.FaultWatchdogReset for a
// watchdog), the board clock restarting at zero, the hardware back at
// power-on (drive disconnected at zero duty, servo without pulses, encoder
// at zero), and whatever sat unread in the receive buffer lost. The link
// itself survives, so the host sees a new Hello rather than a closed port.
func (b *Board) Reboot(bootID uint32, faults boardlink.Faults) error {
	b.mu.Lock()
	defer b.mu.Unlock()
	b.Drive.powerOn()
	b.Servo.powerOn()
	b.Encoder.powerOn()
	b.link.discardPending()
	loop, err := b.newLoop(bootID, faults)
	if err != nil {
		return err
	}
	b.loop = loop
	b.bootAt = time.Now()
	b.lastStep = 0
	return nil
}

// StallToBoard holds everything the host sends for d from now, as a stuck
// endpoint does, and delivers it together afterwards.
func (b *Board) StallToBoard(d time.Duration) { b.link.hold(true, d) }

// StallToHost holds everything the board sends for d from now.
func (b *Board) StallToHost(d time.Duration) { b.link.hold(false, d) }

// LinkStats reports what the link emulation did to the traffic so far.
func (b *Board) LinkStats() LinkStats { return b.link.stats() }

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

// newLoop builds a Loop over the Board's hardware.
func (b *Board) newLoop(bootID uint32, faults boardlink.Faults) (*boardloop.Loop, error) {
	hw := boardloop.Hardware{Link: b.link, Drive: b.Drive, Servo: b.Servo}
	if !b.opts.NoEncoder {
		hw.Encoder = b.Encoder
	}
	if !b.opts.NoButton {
		hw.Button = b.Button
	}
	loop, err := boardloop.New(hw, boardloop.Options{BootID: bootID, BootFaults: faults})
	if err != nil {
		return nil, fmt.Errorf("boardsim: %w", err)
	}
	return loop, nil
}

// step advances the wheel model since the previous step at the duty in
// force, then runs one Loop iteration at the time since boot.
func (b *Board) step() {
	b.mu.Lock()
	defer b.mu.Unlock()
	now := time.Since(b.bootAt)
	if b.opts.CountsPerSecondAtFullDuty != 0 && !b.opts.NoEncoder {
		b.Encoder.advance(b.Drive.Duty()*b.opts.CountsPerSecondAtFullDuty, (now - b.lastStep).Seconds())
	}
	b.lastStep = now
	b.loop.Step(now)
}
