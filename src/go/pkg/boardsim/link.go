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
}

// LinkConfig is the emulation of the board's link to the host. The zero
// value is an ideal link: no delay, no corruption.
type LinkConfig struct {
	// ToBoard is the host-to-board direction (Config, Command, Ping).
	ToBoard Direction
	// ToHost is the board-to-host direction (Hello, Status, Odometry,
	// Button, Pong).
	ToHost Direction
	// Seed makes the jitter and corruption repeatable.
	Seed uint64
}

// link adapts an io.ReadWriter to boardloop.Link. A goroutine drains the
// read side into timestamped segments, so Read never blocks and returns
// only bytes whose delivery time has come. Writes are queued with their own
// delivery time and written in order by a second goroutine, so a slow or
// delayed host never stalls the Loop, as a real board's transmit buffer
// does not. Both goroutines end when the read side of rw ends.
type link struct {
	rw io.ReadWriter

	toBoard Direction
	toHost  Direction
	// rxRand is used only by pump, txRand only by Write (the Loop's
	// goroutine), so neither needs a lock.
	rxRand *rand.Rand
	txRand *rand.Rand

	mu      sync.Mutex
	rx      []segment
	rxErr   error
	txErr   error
	lastRx  time.Time
	lastTx  time.Time
	txQueue chan segment
	done    chan struct{}
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

// Validate reports a negative delay or a corruption rate outside [0, 1].
func (c LinkConfig) Validate() error {
	for name, d := range map[string]Direction{"to_board": c.ToBoard, "to_host": c.ToHost} {
		if d.Latency < 0 || d.Jitter < 0 {
			return fmt.Errorf("boardsim: %s latency %v and jitter %v must not be negative", name, d.Latency, d.Jitter)
		}
		if d.CorruptRate < 0 || d.CorruptRate > 1 {
			return fmt.Errorf("boardsim: %s corrupt rate %v is outside [0, 1]", name, d.CorruptRate)
		}
	}
	return nil
}

func newLink(rw io.ReadWriter, cfg LinkConfig) *link {
	l := &link{
		rw:      rw,
		toBoard: cfg.ToBoard,
		toHost:  cfg.ToHost,
		rxRand:  rand.New(rand.NewPCG(cfg.Seed, rxStream)),
		txRand:  rand.New(rand.NewPCG(cfg.Seed, txStream)),
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
// earlier queued write, or when the queue is full.
func (l *link) Write(p []byte) (int, error) {
	l.mu.Lock()
	err := l.txErr
	l.mu.Unlock()
	if err != nil {
		return 0, err
	}
	data := corrupt(append([]byte(nil), p...), l.toHost.CorruptRate, l.txRand)
	seg := segment{data: data, due: l.nextDue(&l.lastTx, l.toHost, l.txRand)}
	select {
	case l.txQueue <- seg:
		return len(p), nil
	default:
		return 0, errTxQueueFull
	}
}

// pump drains rw into rx until it fails, then closes done.
func (l *link) pump() {
	defer close(l.done)
	chunk := make([]byte, readChunk)
	for {
		n, err := l.rw.Read(chunk)
		if n > 0 {
			data := corrupt(append([]byte(nil), chunk[:n]...), l.toBoard.CorruptRate, l.rxRand)
			due := l.nextDue(&l.lastRx, l.toBoard, l.rxRand)
			l.mu.Lock()
			l.rx = append(l.rx, segment{data: data, due: due})
			l.mu.Unlock()
		}
		if err != nil {
			l.mu.Lock()
			l.rxErr = fmt.Errorf("boardsim: read: %w", err)
			l.mu.Unlock()
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

// nextDue is when a chunk sent now in direction d may be delivered: the
// latency plus a jitter draw, never before the previous chunk (*last).
func (l *link) nextDue(last *time.Time, d Direction, r *rand.Rand) time.Time {
	delay := d.Latency
	if d.Jitter > 0 {
		delay += time.Duration(r.Int64N(int64(d.Jitter) + 1))
	}
	due := time.Now().Add(delay)
	l.mu.Lock()
	defer l.mu.Unlock()
	if due.Before(*last) {
		due = *last
	}
	*last = due
	return due
}

// corrupt flips one random bit in each byte of data with probability rate.
func corrupt(data []byte, rate float64, r *rand.Rand) []byte {
	if rate <= 0 {
		return data
	}
	for i := range data {
		if r.Float64() < rate {
			data[i] ^= 1 << r.IntN(bitsPerByte)
		}
	}
	return data
}
