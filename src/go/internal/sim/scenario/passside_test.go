package scenario

import (
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// A RED sign in the SOUTH corridor driven COUNTERCLOCKWISE: routingTable says
// {AxisY, red -1}, so the permitted side is y BELOW the sign, and travel runs
// +x, so the radius line is x = sign.X and "past" means x greater.
const (
	testSignX = 1.5
	testSignY = 0.5
)

func newTestScorer() *passSideScorer {
	return newPassSideScorer(
		[]signrouter.SignSpec{{X: testSignX, Y: testSignY, Color: signrouter.SignColorRed}},
		trackmodel.Counterclockwise,
		1.0, 2.0, // corner min/max
		0.30, 0.194, // chassis length/width
	)
}

func TestPassSideScorer_WrongSideCrossingEndsTheRound(t *testing.T) {
	t.Parallel()
	s := newTestScorer()

	// Approach on the FORBIDDEN side (y above the sign), not yet past the line.
	if v := s.check(1.0, 0.8, 0); v != nil {
		t.Fatalf("on approach: check = %v, want nil (rules permit fixing the side here)", v)
	}
	// Fully past the radius, still on the forbidden side.
	if v := s.check(2.0, 0.8, 0); len(v) != 1 || v[0] != 0 {
		t.Fatalf("after crossing on the wrong side: check = %v, want [0]", v)
	}
}

func TestPassSideScorer_CorrectSideCrossingIsClean(t *testing.T) {
	t.Parallel()
	s := newTestScorer()

	if v := s.check(1.0, 0.2, 0); v != nil {
		t.Fatalf("on approach: check = %v, want nil", v)
	}
	if v := s.check(2.0, 0.2, 0); v != nil {
		t.Fatalf("after crossing on the PERMITTED side: check = %v, want nil", v)
	}
}

// The recovery window Appendix A section 5 explicitly grants: straying to the
// wrong side and correcting BEFORE the radius is fully crossed is legal, and a
// scorer that punishes it forbids the one recovery the rules go out of their
// way to permit.
func TestPassSideScorer_RecoveryBeforeTheLineIsLegal(t *testing.T) {
	t.Parallel()
	s := newTestScorer()

	if v := s.check(1.2, 0.8, 0); v != nil { // wrong side, not yet past
		t.Fatalf("straying before the line: check = %v, want nil", v)
	}
	if v := s.check(2.0, 0.2, 0); v != nil { // corrected, then crosses
		t.Fatalf("after correcting before the line: check = %v, want nil", v)
	}
}

// Placed beyond the line without ever having been seen on the approach side --
// the in-bay start does this -- is not a crossing and must not be scored, and
// must NOT consume the sign either: the genuine crossing comes later in the lap.
func TestPassSideScorer_PlacedBeyondTheLineIsNotAPass(t *testing.T) {
	t.Parallel()
	s := newTestScorer()

	if v := s.check(2.0, 0.8, 0); v != nil {
		t.Fatalf("placed beyond the line: check = %v, want nil", v)
	}
	// Sign not consumed: drive round to it properly and it still scores.
	if v := s.check(1.0, 0.8, 0); v != nil {
		t.Fatalf("re-approach: check = %v, want nil", v)
	}
	if v := s.check(2.0, 0.8, 0); len(v) != 1 || v[0] != 0 {
		t.Fatalf("genuine later crossing: check = %v, want [0]", v)
	}
}
