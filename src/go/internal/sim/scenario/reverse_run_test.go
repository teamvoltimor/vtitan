package scenario

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/kinematics"
)

// newTestReverseRun judges a counterclockwise round: South is driven +x,
// and the section behind South is West.
func newTestReverseRun() *reverseRunScorer {
	return &reverseRunScorer{
		direction: trackmodel.Counterclockwise, cornerMinM: 1.0, cornerMaxM: 2.0,
		chassisLen: 0.30, chassisWid: 0.194,
	}
}

// Once the round turns back in South, the chassis may be anywhere in South
// or West; a footprint wholly in North, still against the round, ends it
// with South as the origin.
func TestReverseRun_LeavingBothSectionsEndsTheRound(t *testing.T) {
	t.Parallel()

	r := newTestReverseRun()
	against := []kinematics.AckermannState{
		{X: 1.5, Y: 0.5, Yaw: math.Pi, V: 0.3},       // South, driven -x
		{X: 0.5, Y: 1.5, Yaw: math.Pi / 2, V: 0.3},   // West, driven +y
		{X: 1.2, Y: 0.4, Yaw: math.Pi, V: 0.3},       // back in South
		{X: 0.5, Y: 1.9, Yaw: math.Pi / 2, V: 0.3},   // West again
		{X: 0.55, Y: 2.45, Yaw: math.Pi / 4, V: 0.3}, // corner, a corner still in West
	}
	for i, st := range against {
		if r.check(st, i+1) {
			t.Fatalf("violation at %+v, want legal inside South and West", st)
		}
	}
	if !r.check(kinematics.AckermannState{X: 1.5, Y: 2.5, Yaw: 0, V: 0.3}, len(against)+1) {
		t.Fatal("a footprint wholly in North, against the round, was not a violation")
	}
	var res Result
	res.Success = true
	r.fill(&res)
	if !res.ReverseRunViolation || res.Success || res.ReverseRunOriginSection != "south" ||
		res.ReverseRunOriginStep == nil || *res.ReverseRunOriginStep != 1 {
		t.Errorf("result = %+v, want a failed run with origin south at step 1", res)
	}
}

// Reversing is judged on velocity: a chassis facing against the round but
// moving the round's way is legal, and so is one at rest.
func TestReverseRun_ReversingTheRoundWayIsLegal(t *testing.T) {
	t.Parallel()

	r := newTestReverseRun()
	for i, x := 0, 1.2; x < 2.8; i, x = i+1, x+0.02 {
		if r.check(kinematics.AckermannState{X: x, Y: 0.5, Yaw: math.Pi, V: -0.3}, i) {
			t.Fatalf("reversing +x along South flagged at x=%.2f", x)
		}
	}
	if r.check(kinematics.AckermannState{X: 1.5, Y: 2.5, Yaw: 0, V: 0.01}, 0) {
		t.Error("a chassis at rest was flagged")
	}
}

func TestRingStep(t *testing.T) {
	t.Parallel()

	cases := []struct {
		from trackmodel.Section
		dir  trackmodel.Direction
		want trackmodel.Section
	}{
		{trackmodel.South, trackmodel.Counterclockwise, trackmodel.West},
		{trackmodel.South, trackmodel.Clockwise, trackmodel.East},
		{trackmodel.North, trackmodel.Counterclockwise, trackmodel.East},
		{trackmodel.West, trackmodel.Clockwise, trackmodel.South},
	}
	for _, c := range cases {
		if got := ringStep(c.from, c.dir, -1); got != c.want {
			t.Errorf("behind %v going %v = %v, want %v", c.from, c.dir, got, c.want)
		}
	}
}

// The scorer is off unless the model is: a nil one never fires or fills.
func TestReverseRun_NilIsOff(t *testing.T) {
	t.Parallel()

	var r *reverseRunScorer
	if r.check(kinematics.AckermannState{V: 1}, 1) {
		t.Error("a nil scorer fired")
	}
	res := Result{Success: true}
	r.fill(&res)
	if !res.Success || res.ReverseRunViolation {
		t.Errorf("a nil scorer changed the result: %+v", res)
	}
}
