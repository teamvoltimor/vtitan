package boardloop_test

import (
	"context"
	"fmt"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardloop"
)

// events records every actuator write, in order, across the drive and the
// servo, so a test can assert the sequence (steering before drive) as well
// as the values.
type events struct {
	list []string
}

// fakeLink is a Link whose receive side is a byte queue the test fills and
// whose transmit side decodes every frame the Loop writes.
type fakeLink struct {
	in      []byte
	dec     boardlink.Decoder
	out     []boardlink.Packet
	readErr error
}

// fakeDrive is a Drive recording connects and speeds.
type fakeDrive struct {
	ev         *events
	connects   int
	connectErr error
	setErr     error
	speeds     []float64
}

// fakeServo is a Servo recording pulses.
type fakeServo struct {
	ev     *events
	setErr error
	pulses []float64
}

// fakeEncoder is an Encoder with a settable count.
type fakeEncoder struct {
	counts int64
}

// nopLink, nopDrive and nopServo allocate nothing, so AllocsPerRun measures
// only the Loop.
type nopLink struct {
	frame []byte
	feed  bool
}

type nopDrive struct{}

type nopServo struct{}

// harness is a Loop wired to fakes, with a hand-advanced clock.
type harness struct {
	t     *testing.T
	loop  *boardloop.Loop
	link  *fakeLink
	drive *fakeDrive
	servo *fakeServo
	enc   *fakeEncoder
	ev    *events
	now   time.Duration
	seq   uint16
}

// Test calibration: the linkage, pulse and speed numbers of boardlink's own
// sample Config, less the inversion and reversal, which have their own
// tests.
const (
	testBootID       = 0xC0FFEE
	testTimeoutMS    = 500
	testStatusMS     = 100
	testOdometryMS   = 20
	testCenterPulse  = 1500.0
	testLinkage      = 0.5
	testServoMaxDeg  = 135.0
	testSpeedScale   = 30.0
	testMinPulseUS   = 500.0
	testMaxPulseUS   = 2500.0
	testServoRange   = 270.0
	testHWWatchdogMS = 250
	stepTick         = time.Millisecond
	floatTolerance   = 1e-4
)

func (l *fakeLink) Read(p []byte) (int, error) {
	if l.readErr != nil {
		err := l.readErr
		l.readErr = nil
		return 0, err
	}
	n := copy(p, l.in)
	l.in = l.in[n:]
	return n, nil
}

func (l *fakeLink) Write(p []byte) (int, error) {
	for _, b := range p {
		var pkt boardlink.Packet
		ok, err := l.dec.Feed(b, &pkt)
		if err != nil {
			return 0, fmt.Errorf("fakeLink: Loop wrote a bad frame: %w", err)
		}
		if ok {
			l.out = append(l.out, pkt)
		}
	}
	return len(p), nil
}

func (d *fakeDrive) Connect(context.Context) error {
	d.connects++
	d.ev.list = append(d.ev.list, "connect")
	return d.connectErr
}

func (d *fakeDrive) SetSpeed(_ context.Context, normalized float64) error {
	d.speeds = append(d.speeds, normalized)
	d.ev.list = append(d.ev.list, fmt.Sprintf("drive %.3f", normalized))
	return d.setErr
}

func (s *fakeServo) SetPulseUS(pulseUS float64) error {
	s.pulses = append(s.pulses, pulseUS)
	s.ev.list = append(s.ev.list, fmt.Sprintf("servo %.1f", pulseUS))
	return s.setErr
}

func (e *fakeEncoder) Counts() int64 { return e.counts }

func validConfig() boardlink.Config {
	return boardlink.Config{
		CommandTimeoutMS:    testTimeoutMS,
		SpeedScalePctPerMPS: testSpeedScale,
		InvertDrive:         false,
		LinkageRatio:        testLinkage,
		ServoMaxAngleDeg:    testServoMaxDeg,
		SteeringOffsetDeg:   0,
		ServoMinPulseUS:     testMinPulseUS,
		ServoMaxPulseUS:     testMaxPulseUS,
		ServoCenterPulseUS:  testCenterPulse,
		ServoRangeDeg:       testServoRange,
		ServoReversed:       false,
		HardwareWatchdogMS:  testHWWatchdogMS,
		StatusIntervalMS:    testStatusMS,
		OdometryIntervalMS:  testOdometryMS,
	}
}

func newHarness(t *testing.T, bootFaults uint8, withEncoder bool) *harness {
	t.Helper()

	ev := &events{}
	h := &harness{
		t:     t,
		link:  &fakeLink{},
		drive: &fakeDrive{ev: ev},
		servo: &fakeServo{ev: ev},
		ev:    ev,
	}
	hw := boardloop.Hardware{Link: h.link, Drive: h.drive, Servo: h.servo, Encoder: nil}
	if withEncoder {
		h.enc = &fakeEncoder{}
		hw.Encoder = h.enc
	}
	loop, err := boardloop.New(hw, boardloop.Options{BootID: testBootID, BootFaults: bootFaults})
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	h.loop = loop
	return h
}

// queue encodes a host->board packet onto the link.
func (h *harness) queue(p boardlink.Packet) {
	h.t.Helper()

	p.Seq = h.seq
	h.seq++
	frame, err := boardlink.Append(nil, &p)
	if err != nil {
		h.t.Fatalf("Append: %v", err)
	}
	h.link.in = append(h.link.in, frame...)
}

func (h *harness) queueConfig(c boardlink.Config) {
	h.t.Helper()
	h.queue(boardlink.Packet{Type: boardlink.TypeConfig, Config: c})
}

func (h *harness) queueCommand(speed, steer float32) {
	h.t.Helper()
	h.queue(boardlink.Packet{
		Type:    boardlink.TypeCommand,
		Command: boardlink.Command{SpeedMPS: speed, SteeringAngleRad: steer},
	})
}

// step advances the clock by stepTick and runs one Step.
func (h *harness) step() {
	h.now += stepTick
	h.loop.Step(h.now)
}

// run steps every stepTick for d.
func (h *harness) run(d time.Duration) {
	for end := h.now + d; h.now < end; {
		h.step()
	}
}

// configure sends a valid Config, steps once, and clears what that
// recorded, so a test starts from a configured, stopped board.
func (h *harness) configure() {
	h.t.Helper()

	h.queueConfig(validConfig())
	h.step()
	if _, ok := h.loop.Config(); !ok {
		h.t.Fatal("valid Config did not configure the board")
	}
	h.reset()
}

// reset forgets every recorded write and frame.
func (h *harness) reset() {
	h.ev.list = nil
	h.drive.speeds = nil
	h.servo.pulses = nil
	h.link.out = nil
}

// sent returns the frames of type t the Loop has written.
func (h *harness) sent(t boardlink.Type) []boardlink.Packet {
	var out []boardlink.Packet
	for i := range h.link.out {
		if h.link.out[i].Type == t {
			out = append(out, h.link.out[i])
		}
	}
	return out
}

// lastStatus returns the most recent Status, failing if there is none.
func (h *harness) lastStatus() boardlink.Status {
	h.t.Helper()

	st := h.sent(boardlink.TypeStatus)
	if len(st) == 0 {
		h.t.Fatal("no Status sent")
	}
	return st[len(st)-1].Status
}

func near(a, b float64) bool {
	d := a - b
	return d < floatTolerance && d > -floatTolerance
}

func (l *nopLink) Read(p []byte) (int, error) {
	if !l.feed {
		return 0, nil
	}
	l.feed = false
	return copy(p, l.frame), nil
}

func (*nopLink) Write(p []byte) (int, error) { return len(p), nil }

func (nopDrive) Connect(context.Context) error           { return nil }
func (nopDrive) SetSpeed(context.Context, float64) error { return nil }
func (nopServo) SetPulseUS(float64) error                { return nil }
