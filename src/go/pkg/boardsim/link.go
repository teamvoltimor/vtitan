package boardsim

import (
	"errors"
	"fmt"
	"io"
	"math/rand/v2"
	"sync"
	"time"
)

// Direction is the emulation of one direction of the link.
//
// USB retries a packet that fails its CRC in hardware, so on a real USB CDC
// link the faults an application sees are rarely flipped bits: they are
// lost chunks (a reader that fell behind), stalls (an endpoint stuck busy,
// then a backlog delivered at once) and resets. Loss and stalls model those;
// CorruptRate stays for exercising the decoders.
type Direction struct {
	// Latency delays every chunk of bytes by this much.
	Latency time.Duration
	// Jitter adds a further uniform delay in [0, Jitter] per chunk. Order
	// is kept: a chunk is never delivered before the one sent ahead of it,
	// because the link is a byte stream (USB CDC or a UART), not datagrams.
	Jitter time.Duration
	// CorruptRate is the probability, per byte, that one random bit of it
	// is flipped, to exercise the decoder's resynchronization.
	CorruptRate float64
	// LossRate is the long-run fraction of chunks lost whole, in [0, 1).
	// Losses come in bursts (a Gilbert-Elliott two-state model): see
	// LossBurst.
	LossRate float64
	// LossBurst is the mean number of consecutive chunks lost once a loss
	// starts, at least 1 when LossRate is set. 1 gives independent losses.
	LossBurst float64
	// StallRate is how many stalls start per second on average (a Poisson
	// process); zero disables random stalls.
	StallRate float64
	// StallDuration is how long each stall holds delivery. Chunks sent
	// during a stall are delivered together when it ends, as a stuck
	// endpoint's backlog is.
	StallDuration time.Duration
}

// LinkConfig is the emulation of the board's link to the host. The zero
// value is an ideal link: no delay, no loss, no corruption.
type LinkConfig struct {
	// ToBoard is the host-to-board direction (Config, Command, Ping).
	ToBoard Direction
	// ToHost is the board-to-host direction (Hello, Status, Odometry,
	// Button, Pong).
	ToHost Direction
	// Seed makes the jitter, loss, stalls and corruption repeatable.
	Seed uint64
}

// LinkStats counts what the emulation did to the traffic, so a test can
// prove a fault actually happened.
type LinkStats struct {
	ToBoard PathStats
	ToHost  PathStats
}

// PathStats counts one direction.
type PathStats struct {
	Chunks        uint64
	ChunksLost    uint64
	BytesFlipped  uint64
	Stalls        uint64
	ScriptedHolds uint64
}

// link adapts an io.ReadWriter to boardloop.Link. A goroutine drains the
// read side into timestamped segments, so Read never blocks and returns
// only bytes whose delivery time has come. Writes are queued with their own
// delivery time and written in order by a second goroutine, so a slow or
// delayed host never stalls the Loop, as a real board's transmit buffer
// does not. Both goroutines end when the read side of rw ends.
type link struct {
	rw io.ReadWriter

	// mu guards everything below, including both paths: the rx path is
	// driven by pump, the tx path by the Loop's Write, and scripted holds
	// and stats come from test goroutines.
	mu      sync.Mutex
	rxPath  path
	txPath  path
	rx      []segment
	rxErr   error
	txErr   error
	txQueue chan segment
	done    chan struct{}
}

// path is the emulation state of one direction.
type path struct {
	cfg  Direction
	rand *rand.Rand
	// bad is the Gilbert-Elliott state: while bad, every chunk is lost.
	bad bool
	// last is the latest delivery time handed out, so jitter never
	// reorders.
	last time.Time
	// holdUntil is when the current stall (random or scripted) ends.
	holdUntil time.Time
	// nextStall is when the next random stall starts; zero until the
	// first chunk schedules it.
	nextStall time.Time
	stats     PathStats
}

// segment is a chunk of bytes and when it may be delivered.
type segment struct {
	data []byte
	due  time.Time
}

// writeDeadliner is the part of net.Conn a link uses when it has it.
type writeDeadliner interface {
	SetWriteDeadline(t time.Time) error
}

// txQueueFrames bounds the frames waiting to reach the host. A board's
// transmit buffer is finite too; past it a Write fails, which boardloop
// counts in LinkWriteErrors.
const txQueueFrames = 256

// bitsPerByte is how many bit positions corrupt chooses from.
const bitsPerByte = 8

// Streams for the two directions' random sources, so they do not share a
// sequence under one Seed.
const (
	rxStream = 1
	txStream = 2
)

// errTxQueueFull is returned by Write when txQueueFrames are pending.
var errTxQueueFull = errors.New("boardsim: transmit queue full")

// Validate reports a negative delay or duration, or a rate outside its
// range.
func (c LinkConfig) Validate() error {
	for name, d := range map[string]Direction{"to_board": c.ToBoard, "to_host": c.ToHost} {
		if err := d.validate(); err != nil {
			return fmt.Errorf("boardsim: %s: %w", name, err)
		}
	}
	return nil
}

func (d Direction) validate() error {
	switch {
	case d.Latency < 0 || d.Jitter < 0:
		return fmt.Errorf("latency %v and jitter %v must not be negative", d.Latency, d.Jitter)
	case d.CorruptRate < 0 || d.CorruptRate > 1:
		return fmt.Errorf("corrupt rate %v is outside [0, 1]", d.CorruptRate)
	case d.LossRate < 0 || d.LossRate >= 1:
		return fmt.Errorf("loss rate %v is outside [0, 1)", d.LossRate)
	case d.LossRate > 0 && d.LossBurst < 1:
		return fmt.Errorf("loss burst %v must be at least 1 when loss rate is set", d.LossBurst)
	case d.StallRate < 0 || d.StallDuration < 0:
		return fmt.Errorf("stall rate %v and duration %v must not be negative", d.StallRate, d.StallDuration)
	}
	return nil
}

func newLink(rw io.ReadWriter, cfg LinkConfig) *link {
	l := &link{
		rw:      rw,
		rxPath:  path{cfg: cfg.ToBoard, rand: rand.New(rand.NewPCG(cfg.Seed, rxStream))},
		txPath:  path{cfg: cfg.ToHost, rand: rand.New(rand.NewPCG(cfg.Seed, txStream))},
		txQueue: make(chan segment, txQueueFrames),
		done:    make(chan struct{}),
	}
	go l.pump()
	go l.drain()
	return l
}

// Read copies bytes that are due into p without blocking. Once the read
// side has ended and nothing is left it returns the read error.
func (l *link) Read(p []byte) (int, error) {
	now := time.Now()
	l.mu.Lock()
	defer l.mu.Unlock()
	n := 0
	for len(l.rx) > 0 && n < len(p) && !l.rx[0].due.After(now) {
		c := copy(p[n:], l.rx[0].data)
		n += c
		if l.rx[0].data = l.rx[0].data[c:]; len(l.rx[0].data) == 0 {
			l.rx = l.rx[1:]
		}
	}
	if n == 0 && len(l.rx) == 0 && l.rxErr != nil {
		return 0, l.rxErr
	}
	return n, nil
}

// Write queues one frame for the host. It fails with the error of an
// earlier queued write, or when the queue is full. A frame the emulation
// loses still reports success, as a real board cannot tell either.
func (l *link) Write(p []byte) (int, error) {
	l.mu.Lock()
	if l.txErr != nil {
		err := l.txErr
		l.mu.Unlock()
		return 0, err
	}
	data, due, lost := l.txPath.admit(append([]byte(nil), p...), time.Now())
	l.mu.Unlock()
	if lost {
		return len(p), nil
	}
	select {
	case l.txQueue <- segment{data: data, due: due}:
		return len(p), nil
	default:
		return 0, errTxQueueFull
	}
}

// hold stalls one direction for d from now, on top of any random stall.
func (l *link) hold(toBoard bool, d time.Duration) {
	l.mu.Lock()
	defer l.mu.Unlock()
	p := &l.txPath
	if toBoard {
		p = &l.rxPath
	}
	if until := time.Now().Add(d); until.After(p.holdUntil) {
		p.holdUntil = until
	}
	p.stats.ScriptedHolds++
}

// discardPending drops the bytes received but not yet read, as a board
// reset loses whatever sat in its receive buffer.
func (l *link) discardPending() {
	l.mu.Lock()
	defer l.mu.Unlock()
	l.rx = nil
}

// stats returns a copy of both directions' counters.
func (l *link) stats() LinkStats {
	l.mu.Lock()
	defer l.mu.Unlock()
	return LinkStats{ToBoard: l.rxPath.stats, ToHost: l.txPath.stats}
}

// pump drains rw into rx until it fails, then closes done.
func (l *link) pump() {
	defer close(l.done)
	chunk := make([]byte, readChunk)
	for {
		n, err := l.rw.Read(chunk)
		l.mu.Lock()
		if n > 0 {
			if data, due, lost := l.rxPath.admit(append([]byte(nil), chunk[:n]...), time.Now()); !lost {
				l.rx = append(l.rx, segment{data: data, due: due})
			}
		}
		if err != nil {
			l.rxErr = fmt.Errorf("boardsim: read: %w", err)
		}
		l.mu.Unlock()
		if err != nil {
			return
		}
	}
}

// drain writes queued frames to rw in order, each once it is due, until rw
// fails or its read side ends.
func (l *link) drain() {
	for {
		var seg segment
		select {
		case seg = <-l.txQueue:
		case <-l.done:
			return
		}
		if wait := time.Until(seg.due); wait > 0 {
			select {
			case <-time.After(wait):
			case <-l.done:
				return
			}
		}
		if err := l.write(seg.data); err != nil {
			l.mu.Lock()
			l.txErr = err
			l.mu.Unlock()
			return
		}
	}
}

// write sends data, bounded by writeTimeout when rw supports deadlines.
func (l *link) write(data []byte) error {
	if d, ok := l.rw.(writeDeadliner); ok {
		if err := d.SetWriteDeadline(time.Now().Add(writeTimeout)); err != nil {
			return fmt.Errorf("boardsim: write deadline: %w", err)
		}
	}
	if _, err := l.rw.Write(data); err != nil {
		return fmt.Errorf("boardsim: write: %w", err)
	}
	return nil
}

// admit applies this direction's emulation to a chunk sent at now: whether
// it is lost, its corruption, and when it may be delivered. The caller
// holds the link's lock.
func (p *path) admit(data []byte, now time.Time) (out []byte, due time.Time, lost bool) {
	p.stats.Chunks++
	if p.lose() {
		p.stats.ChunksLost++
		return nil, time.Time{}, true
	}
	p.stats.BytesFlipped += uint64(corrupt(data, p.cfg.CorruptRate, p.rand))

	p.advanceStalls(now)
	delay := p.cfg.Latency
	if p.cfg.Jitter > 0 {
		delay += time.Duration(p.rand.Int64N(int64(p.cfg.Jitter) + 1))
	}
	due = now.Add(delay)
	if due.Before(p.holdUntil) {
		due = p.holdUntil
	}
	if due.Before(p.last) {
		due = p.last
	}
	p.last = due
	return data, due, false
}

// lose steps the Gilbert-Elliott chain once and reports whether this chunk
// is lost. From LossRate (the stationary share of the bad state) and
// LossBurst (the mean stay in it), the chain leaves bad with probability
// 1/LossBurst and enters it with the probability that keeps the long-run
// share at LossRate.
func (p *path) lose() bool {
	if p.cfg.LossRate <= 0 {
		return false
	}
	leave := 1 / p.cfg.LossBurst
	enter := p.cfg.LossRate * leave / (1 - p.cfg.LossRate)
	if p.bad {
		p.bad = p.rand.Float64() >= leave
	} else {
		p.bad = p.rand.Float64() < enter
	}
	return p.bad
}

// advanceStalls starts every random stall scheduled up to now. Stalls are
// only noticed when traffic flows, which is when they matter.
func (p *path) advanceStalls(now time.Time) {
	if p.cfg.StallRate <= 0 || p.cfg.StallDuration <= 0 {
		return
	}
	if p.nextStall.IsZero() {
		p.nextStall = now.Add(p.stallGap())
	}
	for !now.Before(p.nextStall) {
		if end := p.nextStall.Add(p.cfg.StallDuration); end.After(p.holdUntil) {
			p.holdUntil = end
		}
		p.stats.Stalls++
		p.nextStall = p.nextStall.Add(p.cfg.StallDuration + p.stallGap())
	}
}

// stallGap draws the exponential time to the next stall.
func (p *path) stallGap() time.Duration {
	return time.Duration(p.rand.ExpFloat64() / p.cfg.StallRate * float64(time.Second))
}

// corrupt flips one random bit in each byte of data with probability rate,
// and returns how many bytes it changed.
func corrupt(data []byte, rate float64, r *rand.Rand) int {
	if rate <= 0 {
		return 0
	}
	flipped := 0
	for i := range data {
		if r.Float64() < rate {
			data[i] ^= 1 << r.IntN(bitsPerByte)
			flipped++
		}
	}
	return flipped
}
