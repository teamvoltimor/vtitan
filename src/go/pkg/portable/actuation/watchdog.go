package actuation

import "time"

// Watchdog is the command-deadline watchdog's state machine, with the clock
// injected: every method takes the current time rather than reading it, so
// the Linux motor loop (time.Now) and the Pico 2 firmware (its own tick)
// share the exact staleness rule, and tests drive it with a fake clock.
//
// It decides; it does not act. When Expire reports a new staleness episode
// the caller stops the drive and centers the steering, once, and it is not
// told again until a fresh command re-arms it. Not safe for concurrent use:
// the loop that owns it calls it from one goroutine.
type Watchdog struct {
	lastAcceptAt time.Time
	stopped      bool
}

// NewWatchdog returns a Watchdog whose clock starts at now, already in the
// stopped state: nothing has been commanded yet, so there is nothing for a
// timeout to stop and Expire stays quiet until the first Accept.
func NewWatchdog(now time.Time) Watchdog {
	return Watchdog{lastAcceptAt: now, stopped: true}
}

// Accept records a command accepted at now (one that passed FiniteCommand)
// and re-arms the watchdog for a new staleness episode.
func (w *Watchdog) Accept(now time.Time) {
	w.lastAcceptAt = now
	w.stopped = false
}

// Expire reports, at most once per staleness episode, that no command has
// been accepted within timeout of now. It returns the command's age and
// true the first time age >= timeout while armed, and marks the watchdog
// stopped; every other call returns false, including later polls of the
// same episode, so the caller does not republish its safety-stop on every
// tick. A command exactly timeout old has expired: fresh means age <
// timeout.
func (w *Watchdog) Expire(now time.Time, timeout time.Duration) (time.Duration, bool) {
	age := now.Sub(w.lastAcceptAt)
	if age < timeout || w.stopped {
		return age, false
	}
	w.stopped = true
	return age, true
}

// Stop marks the watchdog stopped without waiting for a timeout, for a
// caller that has safety-stopped for another reason (the loop's exit).
// A later Expire stays quiet until the next Accept.
func (w *Watchdog) Stop() {
	w.stopped = true
}

// Stopped reports whether the drive is currently safety-stopped: true from
// NewWatchdog, after an Expire that fired, or after Stop, until the next
// Accept.
func (w *Watchdog) Stopped() bool {
	return w.stopped
}
