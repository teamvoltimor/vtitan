package profile_test

import (
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/config/profile"
)

// shippedControlHz is control.toml's control_hz, the rate every escape
// duration below was originally measured and tuned at.
const shippedControlHz = 20.0

// TestFrames_MatchesTheHistoricalFrameCounts is the safety net for the
// seconds-not-frames conversion.
//
// Every duration in escape.toml used to be a literal frame count, and every
// one of them was measured at 20 Hz. Storing seconds and deriving ticks is
// only safe if the derivation reproduces those literals exactly at the
// shipped rate -- otherwise the conversion silently retunes the escape
// system, which is the subsystem where a behaviour change is hardest to
// notice and hardest to attribute.
//
// The pairs below are the pre-conversion values, kept here as the thing the
// arithmetic has to agree with rather than as a restatement of it.
func TestFrames_MatchesTheHistoricalFrameCounts(t *testing.T) {
	t.Parallel()

	for name, tc := range map[string]struct {
		seconds float64
		want    int
	}{
		"k_turn_min":          {0.30, 6},
		"k_turn_max":          {0.60, 12},
		"slalom_reverse":      {0.40, 8},
		"slalom_forward":      {0.50, 10},
		"stuck_timeout":       {2.0, 40},
		"side_correction":     {0.20, 4},
		"max_escape":          {1.0, 20},
		"stuck_escalation":    {0.10, 2},
		"stuck_history_floor": {3.0, 60},
	} {
		if got := profile.Frames(tc.seconds, shippedControlHz); got != tc.want {
			t.Errorf("%s: Frames(%v, %v) = %d, want the historical %d",
				name, tc.seconds, shippedControlHz, got, tc.want)
		}
	}
}

// TestFrames_ScalesWithTheLoopRate is the property the conversion exists
// for: the same configured DURATION must survive a change of loop rate.
// Before this, a frame count meant 2 s at 20 Hz and 0.8 s at 50 Hz, so
// raising the rate silently made the robot give up sooner on everything.
func TestFrames_ScalesWithTheLoopRate(t *testing.T) {
	t.Parallel()

	const twoSeconds = 2.0
	for _, hz := range []float64{10, 20, 50, 100} {
		got := profile.Frames(twoSeconds, hz)
		if want := int(twoSeconds * hz); got != want {
			t.Errorf("Frames(%v, %v) = %d, want %d", twoSeconds, hz, got, want)
		}
		// The point of the exercise: whatever the rate, the wall-clock
		// duration those ticks represent is the one configured.
		if elapsed := float64(got) / hz; elapsed != twoSeconds {
			t.Errorf("at %v Hz the derived %d ticks last %v s, want %v", hz, got, elapsed, twoSeconds)
		}
	}
}

// TestFrames_NeverRoundsAManeuverAway checks the floor. A duration shorter
// than one tick is still a maneuver the caller asked for; truncating it to
// zero frames would skip it entirely, turning a short escape into no escape.
func TestFrames_NeverRoundsAManeuverAway(t *testing.T) {
	t.Parallel()

	if got := profile.Frames(0.001, shippedControlHz); got != 1 {
		t.Errorf("Frames(0.001, %v) = %d, want 1 (never zero)", shippedControlHz, got)
	}
}
