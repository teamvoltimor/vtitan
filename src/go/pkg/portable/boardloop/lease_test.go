package boardloop_test

import (
	"slices"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
)

// testLease is the lease these tests give a command, well inside the
// testTimeoutMS watchdog so the two cannot be confused.
const testLease = 100 * time.Millisecond

// queueLeased sends a command whose lease ends lease after the board's
// current time, with the given expiry action.
func (h *harness) queueLeased(lease time.Duration, onExpiry boardlink.Expiry) {
	h.t.Helper()
	h.queue(boardlink.Packet{Type: boardlink.TypeCommand, Command: boardlink.Command{
		DeadlineUS:       uint64((h.now + lease).Microseconds()),
		SpeedMPS:         testSpeedMPS,
		SteeringAngleRad: testSteerRad,
		OnExpiry:         onExpiry,
	}})
}

// leaseRun applies a leased command, then runs to one tick before and to
// its deadline, returning the writes made at the deadline.
func leaseRun(t *testing.T, onExpiry boardlink.Expiry) (h *harness, atDeadline []string) {
	t.Helper()

	h = newHarness(t, 0, false)
	h.configure()
	h.queueLeased(testLease, onExpiry)
	h.step()
	h.reset()
	h.run(testLease - 2*stepTick)
	if len(h.ev.list) != 0 {
		t.Fatalf("acted before the lease ran out: %v", h.ev.list)
	}
	h.run(2 * stepTick)
	return h, append([]string(nil), h.ev.list...)
}

func TestLease_ExpiryStopCenterStopsAndCenters(t *testing.T) {
	t.Parallel()

	h, got := leaseRun(t, boardlink.ExpiryStopCenter)
	if want := []string{"drive 0.000", wantCenterMsg}; !slices.Equal(got, want) {
		t.Fatalf("writes at the deadline = %v, want %v", got, want)
	}
	if st := h.lastStatus(); st.Duty != 0 {
		t.Errorf("Status at the deadline = %+v, want one sent at once with zero duty", st)
	}
	if c := h.loop.Counters(); c.LeaseExpiries != 1 || c.WatchdogStops != 0 {
		t.Errorf("LeaseExpiries %d, WatchdogStops %d, want 1 and 0", c.LeaseExpiries, c.WatchdogStops)
	}
}

// Stopping mid-turn must not swing the nose: the steering stays put.
func TestLease_ExpiryStopKeepsTheSteering(t *testing.T) {
	t.Parallel()

	_, got := leaseRun(t, boardlink.ExpiryStop)
	if want := []string{"drive 0.000"}; !slices.Equal(got, want) {
		t.Fatalf("writes at the deadline = %v, want only the drive stopped, %v", got, want)
	}
}

// Hold leaves the command running until the watchdog, which still ends it
// on time: a lease never extends CommandTimeoutMS.
func TestLease_ExpiryHoldLeavesItToTheWatchdog(t *testing.T) {
	t.Parallel()

	h, got := leaseRun(t, boardlink.ExpiryHold)
	if len(got) != 0 {
		t.Fatalf("writes at the deadline = %v, want none", got)
	}
	h.run(testTimeoutMS*time.Millisecond - testLease)
	if want := []string{"drive 0.000", wantCenterMsg}; !slices.Equal(h.ev.list, want) {
		t.Errorf("writes by the watchdog timeout = %v, want %v", h.ev.list, want)
	}
}

// An action this firmware does not know is read as the safest one.
func TestLease_UnknownExpiryStopsAndCenters(t *testing.T) {
	t.Parallel()

	_, got := leaseRun(t, boardlink.Expiry(0xEE))
	if want := []string{"drive 0.000", wantCenterMsg}; !slices.Equal(got, want) {
		t.Fatalf("writes at the deadline = %v, want %v", got, want)
	}
}

// A lease past the watchdog does not keep the command alive past it.
func TestLease_CannotExtendTheWatchdog(t *testing.T) {
	t.Parallel()

	h := newHarness(t, 0, false)
	h.configure()
	h.queueLeased(10*time.Second, boardlink.ExpiryHold)
	h.step()
	h.reset()
	h.run(testTimeoutMS * time.Millisecond)
	if want := []string{"drive 0.000", wantCenterMsg}; !slices.Equal(h.ev.list, want) {
		t.Fatalf("writes by the watchdog timeout = %v, want %v", h.ev.list, want)
	}
}

// The stale newest command of a stalled link: it arrives past its deadline
// and is refused, touching nothing and not refreshing the watchdog.
func TestLease_CommandArrivingExpiredIsRefused(t *testing.T) {
	t.Parallel()

	h := newHarness(t, 0, false)
	h.configure()
	h.queueCommand(testSpeedMPS, testSteerRad)
	h.step()
	h.run(testTimeoutMS * time.Millisecond) // the watchdog stops the car
	h.reset()

	h.queue(boardlink.Packet{Type: boardlink.TypeCommand, Command: boardlink.Command{
		DeadlineUS:       uint64((h.now - time.Millisecond).Microseconds()),
		SpeedMPS:         testSpeedMPS,
		SteeringAngleRad: testSteerRad,
	}})
	h.step()
	if len(h.ev.list) != 0 {
		t.Fatalf("an expired command was applied: %v", h.ev.list)
	}
	if c := h.loop.Counters(); c.CommandsExpired != 1 || c.CommandsApplied != 1 {
		t.Errorf("CommandsExpired %d, CommandsApplied %d, want 1 and 1", c.CommandsExpired, c.CommandsApplied)
	}
}

// A newer command replaces the lease, so only the latest deadline counts.
func TestLease_NewCommandReplacesTheLease(t *testing.T) {
	t.Parallel()

	h := newHarness(t, 0, false)
	h.configure()
	h.queueLeased(testLease, boardlink.ExpiryStopCenter)
	h.step()
	h.run(testLease / 2)
	h.queueCommand(testSpeedMPS, testSteerRad) // no lease
	h.step()
	h.reset()
	h.run(testLease)
	if len(h.ev.list) != 0 || h.loop.Counters().LeaseExpiries != 0 {
		t.Errorf("writes %v and %d expiries after the lease was replaced, want none",
			h.ev.list, h.loop.Counters().LeaseExpiries)
	}
}
