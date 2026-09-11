package signrouter_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// TestAdoptDirection_ReKeysTheMirroredPassSideRule matches router.py's own
// adopt_direction docstring: a router built on a stale direction does not
// degrade the lane, it MIRRORS it, because ROUTING_TABLE is keyed on
// (corridor, direction) and every clockwise row is the negation of its
// counterclockwise partner. AdoptDirection must correct that, in place,
// without needing a fresh SignRouter.
func TestAdoptDirection_ReKeysTheMirroredPassSideRule(t *testing.T) {
	t.Parallel()

	cfg := routerTestConfig(t)
	sign := signrouter.SignSpec{X: 1.5, Y: 0.4, Color: signrouter.SignColorRed}
	robotPos := trackmodel.Waypoint{X: sign.X - 0.3, Y: sign.Y}
	waypoint := trackmodel.Waypoint{X: sign.X, Y: sign.Y}

	ccwRouter := newTestRouter(t, []signrouter.SignSpec{sign}, cfg)
	ccwResult := ccwRouter.DeformWaypoint(waypoint, robotPos, 0.0, trackmodel.South, nil)

	cwRouter, err := signrouter.NewSignRouter([]signrouter.SignSpec{sign}, cfg, trackmodel.Clockwise)
	if err != nil {
		t.Fatalf("NewSignRouter() error = %v", err)
	}
	cwResult := cwRouter.DeformWaypoint(waypoint, robotPos, 0.0, trackmodel.South, nil)

	if math.Abs(ccwResult.Y-cwResult.Y) < tolerance {
		t.Fatalf(
			"CCW and CW routers produced the same lane (Y=%v); the fixture must demonstrate the mirror to test the fix",
			ccwResult.Y,
		)
	}

	// A fresh router built on the placeholder direction (CCW here, matching
	// blind construction using the provisional direction from --direction
	// undetermined) must, after AdoptDirection(CW), match the router built
	// directly on CW -- not the router built on CCW.
	staleRouter := newTestRouter(t, []signrouter.SignSpec{sign}, cfg)
	staleRouter.AdoptDirection(trackmodel.Clockwise)
	adoptedResult := staleRouter.DeformWaypoint(waypoint, robotPos, 0.0, trackmodel.South, nil)

	if math.Abs(adoptedResult.Y-cwResult.Y) > tolerance {
		t.Errorf(
			"after AdoptDirection(Clockwise), DeformWaypoint().Y = %v, want %v (the CW lane)",
			adoptedResult.Y, cwResult.Y,
		)
	}
	if math.Abs(adoptedResult.Y-ccwResult.Y) < tolerance {
		t.Error("after AdoptDirection(Clockwise), DeformWaypoint().Y still matches the stale CCW lane")
	}
}

// TestAdoptDirection_NoopWhenUnchanged matches adopt_direction's own early
// return: re-adopting the SAME direction leaves the routed lane exactly as
// it was, including the commit taken on the first DeformWaypoint call (a
// reset would drop the commit and briefly re-evaluate candidates from
// scratch, which is observable as ActiveSignCount changing once the sign is
// later marked passed -- see TestPassedSigns_ActiveSignCountDecrements for
// what that machinery is).
func TestAdoptDirection_NoopWhenUnchanged(t *testing.T) {
	t.Parallel()

	cfg := routerTestConfig(t)
	sign := signrouter.SignSpec{X: 1.5, Y: 0.4, Color: signrouter.SignColorRed}
	robotPos := trackmodel.Waypoint{X: sign.X - 0.3, Y: sign.Y}
	waypoint := trackmodel.Waypoint{X: sign.X, Y: sign.Y}

	direct := newTestRouter(t, []signrouter.SignSpec{sign}, cfg)
	directResult := direct.DeformWaypoint(waypoint, robotPos, 0.0, trackmodel.South, nil)

	reAdopted := newTestRouter(t, []signrouter.SignSpec{sign}, cfg)
	reAdopted.DeformWaypoint(waypoint, robotPos, 0.0, trackmodel.South, nil)
	reAdopted.AdoptDirection(trackmodel.Counterclockwise)
	reAdoptedResult := reAdopted.DeformWaypoint(waypoint, robotPos, 0.0, trackmodel.South, nil)

	if math.Abs(directResult.Y-reAdoptedResult.Y) > tolerance {
		t.Errorf(
			"AdoptDirection with the SAME direction changed the lane: got Y=%v, want %v (unchanged)",
			reAdoptedResult.Y, directResult.Y,
		)
	}
}
