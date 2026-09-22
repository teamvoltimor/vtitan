package actuation_test

import (
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/actuation"
)

// fakeClock is a hand-advanced clock: the Watchdog never reads time itself.
type fakeClock struct {
	now time.Time
}

// watchdogTimeout is the timeout every test here runs with; the rule under
// test is relative to it, not to its value.
const watchdogTimeout = 500 * time.Millisecond

func newFakeClock() *fakeClock {
	return &fakeClock{now: time.Date(2026, 9, 22, 12, 0, 0, 0, time.UTC)}
}

func (c *fakeClock) advance(d time.Duration) time.Time {
	c.now = c.now.Add(d)
	return c.now
}

func TestWatchdog_QuietBeforeTheFirstCommand(t *testing.T) {
	t.Parallel()

	clock := newFakeClock()
	w := actuation.NewWatchdog(clock.now)
	if !w.Stopped() {
		t.Fatal("new Watchdog: Stopped() = false, want true (nothing commanded yet)")
	}
	if _, fired := w.Expire(clock.advance(10*watchdogTimeout), watchdogTimeout); fired {
		t.Fatal("Expire fired before any command was accepted")
	}
}

func TestWatchdog_FiresOncePerEpisode(t *testing.T) {
	t.Parallel()

	clock := newFakeClock()
	w := actuation.NewWatchdog(clock.now)
	w.Accept(clock.now)

	if _, fired := w.Expire(clock.advance(watchdogTimeout/2), watchdogTimeout); fired {
		t.Fatal("Expire fired on a fresh command")
	}
	age, fired := w.Expire(clock.advance(watchdogTimeout), watchdogTimeout)
	if !fired {
		t.Fatal("Expire did not fire on a stale command")
	}
	if want := 3 * watchdogTimeout / 2; age != want {
		t.Errorf("Expire age = %v, want %v", age, want)
	}
	if !w.Stopped() {
		t.Error("Stopped() = false after Expire fired")
	}
	for range 3 {
		if _, fired = w.Expire(clock.advance(watchdogTimeout), watchdogTimeout); fired {
			t.Fatal("Expire fired twice in one staleness episode")
		}
	}
}

func TestWatchdog_NewCommandRearms(t *testing.T) {
	t.Parallel()

	clock := newFakeClock()
	w := actuation.NewWatchdog(clock.now)
	w.Accept(clock.now)
	if _, fired := w.Expire(clock.advance(watchdogTimeout), watchdogTimeout); !fired {
		t.Fatal("first episode did not fire")
	}

	w.Accept(clock.advance(time.Millisecond))
	if w.Stopped() {
		t.Fatal("Stopped() = true after Accept")
	}
	if _, fired := w.Expire(clock.advance(watchdogTimeout-time.Nanosecond), watchdogTimeout); fired {
		t.Fatal("Expire fired before the re-armed command went stale")
	}
	if _, fired := w.Expire(clock.advance(time.Nanosecond), watchdogTimeout); !fired {
		t.Fatal("second episode did not fire")
	}
}

// TestWatchdog_Boundary pins the motor loop's original test, age <
// timeout: a command exactly timeout old has expired, one nanosecond
// younger has not.
func TestWatchdog_Boundary(t *testing.T) {
	t.Parallel()

	for _, tc := range []struct {
		name string
		age  time.Duration
		want bool
	}{
		{"just fresh", watchdogTimeout - time.Nanosecond, false},
		{"exactly timeout", watchdogTimeout, true},
		{"just stale", watchdogTimeout + time.Nanosecond, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()

			clock := newFakeClock()
			w := actuation.NewWatchdog(clock.now)
			w.Accept(clock.now)
			age, fired := w.Expire(clock.advance(tc.age), watchdogTimeout)
			if fired != tc.want {
				t.Errorf("Expire at age %v: fired = %v, want %v", tc.age, fired, tc.want)
			}
			if age != tc.age {
				t.Errorf("Expire age = %v, want %v", age, tc.age)
			}
		})
	}
}

func TestWatchdog_StopSilencesTheEpisode(t *testing.T) {
	t.Parallel()

	clock := newFakeClock()
	w := actuation.NewWatchdog(clock.now)
	w.Accept(clock.now)
	w.Stop()
	if !w.Stopped() {
		t.Fatal("Stopped() = false after Stop")
	}
	if _, fired := w.Expire(clock.advance(2*watchdogTimeout), watchdogTimeout); fired {
		t.Fatal("Expire fired after Stop")
	}
}
