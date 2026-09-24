package boardsim

import (
	"context"
	"math"
	"sync"
)

// Drive is an in-memory H-bridge. It records what the Loop commanded and
// can be told to fail, to exercise the board's actuator fault path. Safe
// for concurrent use: the Loop writes it while a test reads it.
type Drive struct {
	mu        sync.Mutex
	err       error
	duty      float64
	connects  int
	connected bool
}

// Servo is an in-memory steering servo PWM output.
type Servo struct {
	mu      sync.Mutex
	err     error
	pulseUS float64
	written bool
}

// Encoder is an in-memory quadrature count, settable by a test or advanced
// by the Board's wheel model.
type Encoder struct {
	mu     sync.Mutex
	counts int64
	// frac carries the sub-count remainder of the wheel model between
	// steps, so a slow wheel still advances.
	frac float64
}

// Button is an in-memory raw button reading.
type Button struct {
	mu      sync.Mutex
	pressed bool
}

// fracEpsilon is far below one count and far above float64 rounding at any
// realistic count rate.
const fracEpsilon = 1e-9

// Connect brings the drive up, unless SetError made it fail.
func (d *Drive) Connect(context.Context) error {
	d.mu.Lock()
	defer d.mu.Unlock()
	d.connects++
	if d.err != nil {
		return d.err
	}
	d.connected = true
	return nil
}

// SetSpeed records the signed duty, unless SetError made it fail.
func (d *Drive) SetSpeed(_ context.Context, normalized float64) error {
	d.mu.Lock()
	defer d.mu.Unlock()
	if d.err != nil {
		return d.err
	}
	d.duty = normalized
	return nil
}

// SetError makes every later Connect and SetSpeed fail with err; nil
// restores the drive.
func (d *Drive) SetError(err error) {
	d.mu.Lock()
	defer d.mu.Unlock()
	d.err = err
}

// Duty is the last signed duty written, as the H-bridge would apply it.
func (d *Drive) Duty() float64 {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.duty
}

// Connected reports whether a Connect has succeeded.
func (d *Drive) Connected() bool {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.connected
}

// Connects counts Connect calls, failed ones included.
func (d *Drive) Connects() int {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.connects
}

// SetPulseUS records the pulse width, unless SetError made it fail.
func (s *Servo) SetPulseUS(pulseUS float64) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.err != nil {
		return s.err
	}
	s.pulseUS, s.written = pulseUS, true
	return nil
}

// SetError makes every later SetPulseUS fail with err; nil restores the
// servo.
func (s *Servo) SetError(err error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.err = err
}

// PulseUS is the last pulse width written, and false while the Loop has
// never written one (the firmware's servo pin carries no pulses then).
func (s *Servo) PulseUS() (float64, bool) {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.pulseUS, s.written
}

// Counts is the signed quadrature count since boot.
func (e *Encoder) Counts() int64 {
	e.mu.Lock()
	defer e.mu.Unlock()
	return e.counts
}

// Add moves the count by n.
func (e *Encoder) Add(n int64) {
	e.mu.Lock()
	defer e.mu.Unlock()
	e.counts += n
}

// advance integrates countsPerS over dtS, keeping the fractional
// remainder.
func (e *Encoder) advance(countsPerS, dtS float64) {
	e.mu.Lock()
	defer e.mu.Unlock()
	e.frac += countsPerS * dtS
	// The nudge keeps float rounding (ten steps of 0.1 summing to
	// 0.99999...) from holding back a count the wheel has turned.
	whole := math.Trunc(e.frac + math.Copysign(fracEpsilon, e.frac))
	e.counts += int64(whole)
	e.frac -= whole
}

// Pressed is the raw reading, true while the button is down.
func (b *Button) Pressed() bool {
	b.mu.Lock()
	defer b.mu.Unlock()
	return b.pressed
}

// SetPressed changes the raw reading.
func (b *Button) SetPressed(pressed bool) {
	b.mu.Lock()
	defer b.mu.Unlock()
	b.pressed = pressed
}
