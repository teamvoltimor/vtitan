package directionestimator_test

import (
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/directionestimator"
)

func TestEstimator_ObserveSettlesAfterMinVotes(t *testing.T) {
	t.Parallel()

	est := directionestimator.NewEstimator(3)
	cfg := directionestimator.DefaultConfig()
	s := scan(0.3, 3.0) // decisive clockwise reading

	for i := range 2 {
		if settled := est.Observe(s, 0, cfg); settled {
			t.Fatalf("Observe() settled after %d votes, want not yet (min 3)", i+1)
		}
		if est.IsSettled() {
			t.Fatalf("IsSettled() = true after %d votes, want false", i+1)
		}
	}

	if settled := est.Observe(s, 0, cfg); !settled {
		t.Fatal("Observe() on 3rd matching vote: want settled = true")
	}
	if !est.IsSettled() {
		t.Fatal("IsSettled() = false after min votes reached, want true")
	}

	dir, ok := est.Direction()
	if !ok || dir != directionestimator.Clockwise {
		t.Errorf("Direction() = (%v, %v), want (Clockwise, true)", dir, ok)
	}
}

func TestEstimator_ObserveIgnoresAmbiguousScans(t *testing.T) {
	t.Parallel()

	est := directionestimator.NewEstimator(1)
	cfg := directionestimator.DefaultConfig()
	s := scan(0.5, 0.5) // symmetric, ambiguous

	if settled := est.Observe(s, 0, cfg); settled {
		t.Fatal("Observe() on an ambiguous scan settled, want false")
	}
	if est.IsSettled() {
		t.Fatal("IsSettled() = true after an ambiguous scan, want false")
	}
}

func TestEstimator_SettleIsIdempotent(t *testing.T) {
	t.Parallel()

	est := directionestimator.NewEstimator(directionestimator.DefaultMinVotes)
	est.Settle(directionestimator.Counterclockwise)
	est.Settle(directionestimator.Clockwise) // must be ignored -- already committed

	dir, ok := est.Direction()
	if !ok || dir != directionestimator.Counterclockwise {
		t.Errorf(
			"Direction() = (%v, %v), want (Counterclockwise, true) -- second Settle must be a no-op",
			dir,
			ok,
		)
	}
}

func TestEstimator_ObserveAfterSettledIsANoOp(t *testing.T) {
	t.Parallel()

	est := directionestimator.NewEstimator(1)
	est.Settle(directionestimator.Clockwise)

	cfg := directionestimator.DefaultConfig()
	s := scan(3.0, 0.3) // would otherwise vote Counterclockwise

	if settled := est.Observe(s, 0, cfg); settled {
		t.Fatal("Observe() after Settle reported settling again, want false (no-op)")
	}
	dir, _ := est.Direction()
	if dir != directionestimator.Clockwise {
		t.Errorf("Direction() = %v after a post-settle Observe, want unchanged Clockwise", dir)
	}
}

func TestEstimator_VotesTallyPerDirection(t *testing.T) {
	t.Parallel()

	est := directionestimator.NewEstimator(10) // high enough to never settle
	cfg := directionestimator.DefaultConfig()

	cw := scan(0.3, 3.0)
	ccw := scan(3.0, 0.3)

	est.Observe(cw, 0, cfg)
	est.Observe(cw, 0, cfg)
	est.Observe(ccw, 0, cfg)

	votes := est.Votes()
	if votes[directionestimator.Clockwise] != 2 {
		t.Errorf("Votes()[Clockwise] = %d, want 2", votes[directionestimator.Clockwise])
	}
	if votes[directionestimator.Counterclockwise] != 1 {
		t.Errorf(
			"Votes()[Counterclockwise] = %d, want 1",
			votes[directionestimator.Counterclockwise],
		)
	}

	votes[directionestimator.Clockwise] = 99
	if est.Votes()[directionestimator.Clockwise] == 99 {
		t.Error("Votes() returned a live map, want a defensive copy")
	}
}
