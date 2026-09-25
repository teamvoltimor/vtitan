package picolink_test

import (
	"context"
	"math"
	"testing"
	"time"

	"google.golang.org/protobuf/types/known/timestamppb"

	"github.com/teamvoltimor/vtitan/src/go/internal/node/picolink"
	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
	"github.com/teamvoltimor/vtitan/src/go/pkg/boardsim"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/actuation"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
)

// testLeasePolicy gives drivingSpeedMPS a 100 ms lease, a fifth of
// sampleBoard's 500 ms command watchdog, so the two cannot be confused.
var testLeasePolicy = picolink.LeasePolicy{
	BlindDistanceM: 0.05,
	Min:            50 * time.Millisecond,
	Max:            400 * time.Millisecond,
	OnExpiry:       boardlink.ExpiryStop,
}

func TestLeasePolicy_For(t *testing.T) {
	t.Parallel()

	p := picolink.LeasePolicy{BlindDistanceM: 0.1, Min: 150 * time.Millisecond, Max: 400 * time.Millisecond}
	for _, tt := range []struct {
		speed float64
		want  time.Duration
	}{
		{speed: 0.5, want: 200 * time.Millisecond},
		{speed: -0.5, want: 200 * time.Millisecond},
		{speed: 2, want: 150 * time.Millisecond},
		{speed: 0.1, want: 400 * time.Millisecond},
		{speed: 0, want: 400 * time.Millisecond},
	} {
		if got := p.For(tt.speed); got != tt.want {
			t.Errorf("For(%v) = %v, want %v", tt.speed, got, tt.want)
		}
	}
}

func TestLeasePolicy_Validate(t *testing.T) {
	t.Parallel()

	good := testLeasePolicy
	if err := good.Validate(); err != nil {
		t.Errorf("Validate(%+v) = %v, want nil", good, err)
	}
	if err := (picolink.LeasePolicy{}).Validate(); err != nil {
		t.Errorf("Validate(off) = %v, want nil", err)
	}
	for name, bad := range map[string]picolink.LeasePolicy{
		"no min":         {BlindDistanceM: 0.1, Max: time.Second},
		"max below min":  {BlindDistanceM: 0.1, Min: time.Second, Max: time.Millisecond},
		"unknown expiry": {BlindDistanceM: 0.1, Min: 1, Max: 2, OnExpiry: 9},
	} {
		if bad.Validate() == nil {
			t.Errorf("%s: Validate(%+v) = nil, want an error", name, bad)
		}
	}
}

// startLeased drives a virtual board with testLeasePolicy, stamping every
// command when it is sent, until stop is called; it returns once the
// session has a clock estimate and the drive carries the command, so the
// commands in flight carry leases.
func startLeased(t *testing.T, health picolink.HealthPolicy) (v *virtualHarness, stop func()) {
	t.Helper()

	v = startVirtual(t, picolink.SessionConfig{Board: sampleBoard, Lease: testLeasePolicy, Health: health},
		boardsim.Options{})
	v.waitState(t, actuationv1.MotorStatus_STATE_IDLE)
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan struct{})
	go func() {
		defer close(done)
		ticker := time.NewTicker(50 * time.Millisecond)
		defer ticker.Stop()
		for {
			cmd := &actuationv1.AckermannCmd{Stamp: timestamppb.Now(), Speed: drivingSpeedMPS}
			select {
			case v.cmds <- cmd:
			case <-ctx.Done():
				return
			}
			select {
			case <-ticker.C:
			case <-ctx.Done():
				return
			}
		}
	}()
	stop = func() {
		cancel()
		<-done
	}
	t.Cleanup(stop)
	waitFor(t, "a clock estimate", func() bool { _, ok := v.session.Clock(); return ok })
	want := drivingDuty()
	waitFor(t, "the commanded duty", func() bool { return math.Abs(v.board.Drive.Duty()-want) < 1e-9 })
	// Past one command period, so the command in force was sent with a clock.
	time.Sleep(100 * time.Millisecond)
	return v, stop
}

// A link stall stops the car when the lease of the last command through
// runs out, well before the 500 ms command watchdog would.
func TestLease_StallStopsTheCarWithinTheLease(t *testing.T) {
	t.Parallel()

	v, _ := startLeased(t, picolink.HealthPolicy{})
	stalled := time.Now()
	v.board.StallToBoard(1200 * time.Millisecond)
	waitFor(t, "the lease to stop the drive", func() bool { return v.board.Drive.Duty() == 0 })
	stoppedAfter := time.Since(stalled)

	// The lease is 100 ms from the last command through, which left up to
	// one 50 ms period before the stall; allow scheduling slack on top.
	if stoppedAfter > 250*time.Millisecond {
		t.Errorf("drive stopped %v after the stall began, want within the 100 ms lease", stoppedAfter)
	}
	if got := v.board.Counters().LeaseExpiries; got == 0 {
		t.Error("LeaseExpiries = 0, want the lease to have ended the command")
	}
	t.Logf("stopped %v into the stall", stoppedAfter.Round(time.Millisecond))
}

// The 2.6 residual: a host that stops sending during a stall leaves one
// stale command queued. It arrives past its lease and is refused, so the
// car stays stopped instead of lurching forward on seconds-old input.
func TestLease_StaleCommandAfterAStallIsRefused(t *testing.T) {
	t.Parallel()

	v, stop := startLeased(t, picolink.HealthPolicy{})
	v.board.StallToBoard(1 * time.Second)
	time.Sleep(60 * time.Millisecond) // at least one command queues behind the stall
	stop()
	waitFor(t, "the drive stopped", func() bool { return v.board.Drive.Duty() == 0 })
	applied := v.board.Counters().CommandsApplied

	waitFor(t, "the stalled commands to arrive and be refused", func() bool {
		return v.board.Counters().CommandsExpired > 0
	})
	time.Sleep(100 * time.Millisecond)
	if d := v.board.Drive.Duty(); d != 0 {
		t.Errorf("duty %v after the stall released, want 0: a stale command was applied", d)
	}
	if got := v.board.Counters().CommandsApplied; got != applied {
		t.Errorf("CommandsApplied %d -> %d across the release, want no stale command applied", applied, got)
	}
}

// A stall that runs a lease out makes the host cap the speed: the board
// reports the expiry, the host publishes the link as degraded, and the
// commands after the stall reach the drive at the cap, until Hold passes
// with a healthy link.
func TestHealth_DegradedLinkCapsTheSpeed(t *testing.T) {
	t.Parallel()

	const capMPS = 0.25
	v, _ := startLeased(t, picolink.HealthPolicy{
		MarginFloor: 20 * time.Millisecond, Hold: 800 * time.Millisecond, SpeedCapMPS: capMPS,
	})
	v.board.StallToBoard(300 * time.Millisecond)

	capped := actuation.SpeedToNormalized(capMPS, float64(sampleBoard.SpeedScalePctPerMPS))
	if sampleBoard.InvertDrive {
		capped = -capped
	}
	waitFor(t, "the drive at the capped speed", func() bool {
		return math.Abs(v.board.Drive.Duty()-capped) < 1e-9
	})
	deadline := time.After(waitTimeout)
	for degraded := false; !degraded; {
		select {
		case st := <-v.status:
			degraded = st.GetLink().GetDegraded() && st.GetLink().GetLeaseExpiries() > 0
		case <-deadline:
			t.Fatal("no MotorStatus reporting a degraded link")
		}
	}
	waitFor(t, "the cap lifted after the hold", func() bool {
		return math.Abs(v.board.Drive.Duty()-drivingDuty()) < 1e-9
	})
}
