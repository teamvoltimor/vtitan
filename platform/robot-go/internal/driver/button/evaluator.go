package button

import "time"

// Thresholds configures the evaluator's debounce and hold-threshold
// timings -- the direct analog of
// platform/robot/src/hardware/button/config.py's Config, minus PullUp
// (that's a GPIO wiring fact, part of Driver's Config in driver.go, not a
// timing tunable). Every field here is a real Config-worthy tunable: the
// debounce window depends on the specific physical switch, and the two
// hold thresholds are operator-facing behavior (how long an E-STOP hold or
// a shutdown hold takes) -- none of these are fixed facts safe to hardcode.
type Thresholds struct {
	// DebounceInterval is how long a raw pin transition must hold steady
	// before the evaluator accepts it as a real press/release, filtering
	// mechanical switch bounce.
	DebounceInterval time.Duration `validate:"required,gt=0"`
	// LongPressThreshold is how long the button must be held before
	// KindLongPress fires.
	LongPressThreshold time.Duration `validate:"required,gt=0"`
	// ShutdownPressThreshold is how long the button must be held before
	// KindShutdownPress fires. Kept well above LongPressThreshold -- see
	// config.py's field doc comment for why the gap matters operationally
	// (an operator holding through an emergency stop must not also
	// trigger a power-off).
	ShutdownPressThreshold time.Duration `validate:"required,gtfield=LongPressThreshold"`
}

// pressState is the evaluator's mutable state between samples: the raw
// (undebounced) pin reading and when it last changed, the debounced press
// state and when the current debounced press began, and which hold
// thresholds this press has already fired -- so a threshold fires exactly
// once per press, not on every sample tick past it.
type pressState struct {
	rawPressed    bool
	rawSince      time.Time
	pressed       bool
	pressedSince  time.Time
	longFired     bool
	shutdownFired bool
}

// Evaluator is the pure, hardware-independent debounce and hold-threshold
// state machine -- the Go analog of
// platform/robot/src/hardware/button/gpio/driver.py's
// _on_pressed/_on_hold_threshold/_on_released callbacks, collapsed into a
// single pull-based function so it can be driven by a poll loop
// (Driver.Read, driver.go) instead of gpiozero's own callback threads.
// Depends on nothing but its Thresholds config and the time.Time each
// sample is taken at, so it's fully unit-testable with a scripted sample
// sequence and no real GPIO line -- see evaluator_test.go. Exported (unlike
// motor's dutyWriter/enableWriter, which stay private interfaces) because,
// like motor.Controller, it is itself the thing under test from a
// black-box _test package.
type Evaluator struct {
	cfg   Thresholds
	state pressState
}

// DefaultDebounceInterval, DefaultLongPressThreshold, and
// DefaultShutdownPressThreshold match config.py's field defaults.
const (
	DefaultDebounceInterval       = 50 * time.Millisecond
	DefaultLongPressThreshold     = 3 * time.Second
	DefaultShutdownPressThreshold = 10 * time.Second
)

// DefaultThresholds returns the Thresholds matching config.py's defaults.
func DefaultThresholds() Thresholds {
	return Thresholds{
		DebounceInterval:       DefaultDebounceInterval,
		LongPressThreshold:     DefaultLongPressThreshold,
		ShutdownPressThreshold: DefaultShutdownPressThreshold,
	}
}

// NewEvaluator builds an Evaluator with no press in progress. cfg is
// assumed already validated (Driver.New does this once, covering the
// embedded Thresholds).
func NewEvaluator(cfg Thresholds) *Evaluator {
	return &Evaluator{cfg: cfg}
}

// Sample feeds one new raw pin reading (true = pressed, already normalized
// for pull-up/pull-down by the caller) at time now, and returns the Event
// it produced, or nil if this sample didn't cross a debounce or
// hold-threshold boundary.
func (e *Evaluator) Sample(rawPressed bool, now time.Time) *Event {
	if rawPressed != e.state.rawPressed {
		e.state.rawPressed = rawPressed
		e.state.rawSince = now
	}

	if rawPressed != e.state.pressed && now.Sub(e.state.rawSince) >= e.cfg.DebounceInterval {
		return e.debouncedTransition(rawPressed, now)
	}

	if e.state.pressed {
		return e.checkHoldThresholds(now)
	}

	return nil
}

// debouncedTransition handles a raw press/release that has just cleared the
// debounce window -- the direct analog of _on_pressed/_on_released.
func (e *Evaluator) debouncedTransition(pressed bool, now time.Time) *Event {
	e.state.pressed = pressed

	if pressed {
		e.state.pressedSince = now
		e.state.longFired = false
		e.state.shutdownFired = false
		return &Event{Kind: KindPressed, HeldSec: 0}
	}

	held := now.Sub(e.state.pressedSince).Seconds()
	kind := KindShortPress
	if e.state.longFired || e.state.shutdownFired {
		// A hold event already fired for this press -- release only
		// clears it, matching _on_released's _threshold_reached branch.
		kind = KindReleased
	}
	e.state.longFired = false
	e.state.shutdownFired = false
	return &Event{Kind: kind, HeldSec: held}
}

// checkHoldThresholds fires KindShutdownPress/KindLongPress the first
// sample where the current press has held past each threshold -- the
// pull-based analog of the Python driver's per-threshold threading.Timer
// callbacks (_on_hold_threshold), collapsed into one poll-driven check so
// no timer goroutines are needed.
func (e *Evaluator) checkHoldThresholds(now time.Time) *Event {
	held := now.Sub(e.state.pressedSince)

	if !e.state.shutdownFired && held >= e.cfg.ShutdownPressThreshold {
		e.state.shutdownFired = true
		return &Event{Kind: KindShutdownPress, HeldSec: held.Seconds()}
	}
	if !e.state.longFired && held >= e.cfg.LongPressThreshold {
		e.state.longFired = true
		return &Event{Kind: KindLongPress, HeldSec: held.Seconds()}
	}
	return nil
}
