package widthbelief_test

import (
	"maps"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/waypoints"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/widthbelief"
)

// narrowPriorM/wideTruthM are the two hypotheses a blind round holds. They
// share the fixed outer wall, so believing the wrong one displaces the whole
// planned line rather than resizing it symmetrically.
const (
	narrowPriorM = 0.6
	wideTruthM   = 1.0
)

// blindPrior is the belief every blind round starts from: narrow everywhere,
// nothing measured.
func blindPrior() map[trackmodel.Section]float64 {
	return map[trackmodel.Section]float64{
		trackmodel.North: narrowPriorM,
		trackmodel.South: narrowPriorM,
		trackmodel.East:  narrowPriorM,
		trackmodel.West:  narrowPriorM,
	}
}

func TestGate_FirstUpdateAdoptsThePriorWithoutClaimingAChange(t *testing.T) {
	t.Parallel()

	gate := widthbelief.New(true)
	widths, unconfirmed, changed := gate.Update(
		blindPrior(), waypoints.AllUnconfirmed(), trackmodel.South,
	)

	// The caller's initial path was already built from exactly these values.
	// Reporting a change would force a redundant replan -- and a re-seek of
	// the waypoint index -- on the first tick of every round.
	if changed {
		t.Error("first Update reported changed=true; the caller already planned from this")
	}
	if !maps.Equal(widths, blindPrior()) {
		t.Errorf("first Update returned %v, want the prior adopted wholesale", widths)
	}
	if unconfirmed != waypoints.AllUnconfirmed() {
		t.Errorf("first Update returned unconfirmed %+v, want all unconfirmed", unconfirmed)
	}
}

func TestGate_HoldsTheCorridorTheRobotIsStandingIn(t *testing.T) {
	t.Parallel()

	gate := widthbelief.New(true)
	gate.Update(blindPrior(), waypoints.AllUnconfirmed(), trackmodel.South)

	// The estimator measures SOUTH wide while the robot is still in SOUTH.
	// This is the one update that would move the line being tracked.
	believed := blindPrior()
	believed[trackmodel.South] = wideTruthM
	observed := waypoints.AllUnconfirmed()
	observed.Set(trackmodel.South, false)

	widths, unconfirmed, changed := gate.Update(believed, observed, trackmodel.South)

	if changed {
		t.Error("changed=true while standing in the corridor that changed")
	}
	if widths[trackmodel.South] != narrowPriorM {
		t.Errorf("planned South width = %g, want the held prior %g",
			widths[trackmodel.South], narrowPriorM)
	}
	if unconfirmed.Confirmed(trackmodel.South) {
		t.Error("South reads as confirmed to the plan while its change is held")
	}
}

func TestGate_ReleasesOnceTheRobotLeaves(t *testing.T) {
	t.Parallel()

	gate := widthbelief.New(true)
	gate.Update(blindPrior(), waypoints.AllUnconfirmed(), trackmodel.South)

	believed := blindPrior()
	believed[trackmodel.South] = wideTruthM
	observed := waypoints.AllUnconfirmed()
	observed.Set(trackmodel.South, false)
	gate.Update(believed, observed, trackmodel.South) // held

	// The robot has moved on. The estimator says nothing new this tick --
	// which is the point: a held belief is released by MOVING, not by a new
	// reading, so the gate must be driven every tick.
	widths, unconfirmed, changed := gate.Update(believed, observed, trackmodel.East)

	if !changed {
		t.Fatal("changed=false after leaving the corridor; the held belief never landed")
	}
	if widths[trackmodel.South] != wideTruthM {
		t.Errorf("planned South width = %g, want the released %g",
			widths[trackmodel.South], wideTruthM)
	}
	if !unconfirmed.Confirmed(trackmodel.South) {
		t.Error("South still reads unconfirmed after its measurement was released")
	}
}

func TestGate_AppliesAChangeToACorridorTheRobotIsNotIn(t *testing.T) {
	t.Parallel()

	gate := widthbelief.New(true)
	gate.Update(blindPrior(), waypoints.AllUnconfirmed(), trackmodel.South)

	// Updating a corridor the robot is not in costs nothing: it arrives on
	// the new line instead of being displaced onto it.
	believed := blindPrior()
	believed[trackmodel.North] = wideTruthM
	observed := waypoints.AllUnconfirmed()
	observed.Set(trackmodel.North, false)

	widths, _, changed := gate.Update(believed, observed, trackmodel.South)

	if !changed {
		t.Fatal("changed=false for a corridor the robot is not standing in")
	}
	if widths[trackmodel.North] != wideTruthM {
		t.Errorf("planned North width = %g, want %g applied immediately",
			widths[trackmodel.North], wideTruthM)
	}
}

func TestGate_ConfirmedNessAloneIsAlsoHeld(t *testing.T) {
	t.Parallel()

	gate := widthbelief.New(true)
	gate.Update(blindPrior(), waypoints.AllUnconfirmed(), trackmodel.South)

	// A corridor that is GENUINELY narrow confirms at the value the prior
	// already held: the width does not move at all. But dropping the
	// unconfirmed inner bias still shifts the planned line -- in the corridor
	// least able to afford a surprise. Gating width alone would let it
	// through, and no width-only assertion would ever see it.
	observed := waypoints.AllUnconfirmed()
	observed.Set(trackmodel.South, false)

	widths, unconfirmed, changed := gate.Update(blindPrior(), observed, trackmodel.South)

	if changed {
		t.Error("changed=true for a confirmed-ness-only change in the current corridor")
	}
	if unconfirmed.Confirmed(trackmodel.South) {
		t.Error("South's confirmation was applied while the robot stands in it")
	}
	if widths[trackmodel.South] != narrowPriorM {
		t.Errorf("South width = %g, want the unchanged prior %g",
			widths[trackmodel.South], narrowPriorM)
	}
}

func TestGate_DisabledAppliesEverythingImmediately(t *testing.T) {
	t.Parallel()

	gate := widthbelief.New(false)
	gate.Update(blindPrior(), waypoints.AllUnconfirmed(), trackmodel.South)

	believed := blindPrior()
	believed[trackmodel.South] = wideTruthM
	observed := waypoints.AllUnconfirmed()
	observed.Set(trackmodel.South, false)

	widths, unconfirmed, changed := gate.Update(believed, observed, trackmodel.South)

	if !changed {
		t.Fatal("disabled gate held a change back")
	}
	if widths[trackmodel.South] != wideTruthM || !unconfirmed.Confirmed(trackmodel.South) {
		t.Errorf("disabled gate did not apply in full: width %g, confirmed %v",
			widths[trackmodel.South], unconfirmed.Confirmed(trackmodel.South))
	}
}

func TestGate_UnchangedBeliefReportsNoChange(t *testing.T) {
	t.Parallel()

	gate := widthbelief.New(true)
	gate.Update(blindPrior(), waypoints.AllUnconfirmed(), trackmodel.South)

	// Driven every tick, the overwhelmingly common case is "nothing moved".
	// It must not churn the path.
	for range 5 {
		if _, _, changed := gate.Update(
			blindPrior(), waypoints.AllUnconfirmed(), trackmodel.South,
		); changed {
			t.Fatal("changed=true with an identical belief")
		}
	}
}

func TestGate_ReturnedWidthsAreACopy(t *testing.T) {
	t.Parallel()

	gate := widthbelief.New(true)
	widths, _, _ := gate.Update(blindPrior(), waypoints.AllUnconfirmed(), trackmodel.South)

	// The caller plans from this map and may hold it. Handing out the gate's
	// own storage would let a caller's edit silently become the gate's
	// belief, so the next tick would compare against a value nothing measured.
	widths[trackmodel.North] = 99.0

	again, _, _ := gate.Update(blindPrior(), waypoints.AllUnconfirmed(), trackmodel.East)
	if again[trackmodel.North] != narrowPriorM {
		t.Fatalf("gate's North width = %g after a caller mutated its return value, want %g",
			again[trackmodel.North], narrowPriorM)
	}
}
