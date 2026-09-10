package signrouter_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// routerTestConfig is router_config from test_sign_router.py: default
// tuning with the settle window disabled, since most of these tests
// exercise engage/pass logic directly over a handful of calls rather than
// the settle-window feature itself (see TestSettleWindow).
func routerTestConfig(t *testing.T) signrouter.Config {
	t.Helper()
	cfg := signrouter.DefaultConfig()
	cfg.SettleTicks = 0
	return cfg
}

func newTestRouter(
	t *testing.T,
	signs []signrouter.SignSpec,
	cfg signrouter.Config,
) *signrouter.SignRouter {
	t.Helper()
	router, err := signrouter.NewSignRouter(signs, cfg, trackmodel.Counterclockwise)
	if err != nil {
		t.Fatalf("NewSignRouter() error = %v", err)
	}
	return router
}

// TestDeformWaypoint_RoutingDecisionPerCorridorAndColor matches
// test_routing_decision_all_grid_positions: routing must produce the
// correct pass side for a sign in every corridor x color combination when
// approached from just outside activation distance.
func TestDeformWaypoint_RoutingDecisionPerCorridorAndColor(t *testing.T) {
	t.Parallel()

	cfg := routerTestConfig(t)
	for _, geo := range deformationSections {
		for _, color := range []signrouter.SignColor{signrouter.SignColorRed, signrouter.SignColorGreen} {
			sign := signrouter.SignSpec{X: geo.sx, Y: geo.sy, Color: color}
			router := newTestRouter(t, []signrouter.SignSpec{sign}, cfg)

			var robotPos trackmodel.Waypoint
			if geo.axisY {
				robotPos = trackmodel.Waypoint{X: geo.sx - 0.3, Y: geo.sy}
			} else {
				robotPos = trackmodel.Waypoint{X: geo.sx, Y: geo.sy - 0.3}
			}

			result := router.DeformWaypoint(
				trackmodel.Waypoint{X: geo.sx, Y: geo.sy}, robotPos, 0.0, geo.section, nil,
			)

			colorSign := 1
			if color == signrouter.SignColorGreen {
				colorSign = -1
			}
			offset := float64(geo.redMult*colorSign) * cfg.LateralOffsetM
			if geo.axisY {
				want := expectedLateral(geo.sy+offset, geo.lowSide, cfg)
				if math.Abs(result.Y-want) > tolerance {
					t.Errorf("%v/%v: Y = %v, want %v", geo.section, color, result.Y, want)
				}
			} else {
				want := expectedLateral(geo.sx+offset, geo.lowSide, cfg)
				if math.Abs(result.X-want) > tolerance {
					t.Errorf("%v/%v: X = %v, want %v", geo.section, color, result.X, want)
				}
			}
		}
	}
}

// TestDeformWaypoint_SignOutsideActivationNotDeformed matches
// TestActivationDistance.test_sign_outside_activation_not_deformed.
func TestDeformWaypoint_SignOutsideActivationNotDeformed(t *testing.T) {
	t.Parallel()

	cfg := routerTestConfig(t)
	sign := signrouter.SignSpec{X: 1.5, Y: 0.4, Color: signrouter.SignColorRed}
	router := newTestRouter(t, []signrouter.SignSpec{sign}, cfg)

	wp := trackmodel.Waypoint{X: 0.5, Y: 0.4}
	gap := cfg.ActivationDistM + 0.2
	robotPos := trackmodel.Waypoint{X: 1.5 - gap, Y: 0.4}
	result := router.DeformWaypoint(wp, robotPos, 0.0, trackmodel.South, nil)

	if result != wp {
		t.Errorf(
			"DeformWaypoint() = %+v, want unchanged %+v (sign outside activation distance)",
			result,
			wp,
		)
	}
}

// TestDeformWaypoint_SignInsideActivationDeformed matches
// TestActivationDistance.test_sign_inside_activation_deformed.
func TestDeformWaypoint_SignInsideActivationDeformed(t *testing.T) {
	t.Parallel()

	cfg := routerTestConfig(t)
	sign := signrouter.SignSpec{X: 1.5, Y: 0.4, Color: signrouter.SignColorRed}
	router := newTestRouter(t, []signrouter.SignSpec{sign}, cfg)

	wp := trackmodel.Waypoint{X: 1.5, Y: 0.4}
	gap := cfg.ActivationDistM / 4
	robotPos := trackmodel.Waypoint{X: 1.5 - gap, Y: 0.4}
	result := router.DeformWaypoint(wp, robotPos, 0.0, trackmodel.South, nil)

	if result == wp {
		t.Error(
			"DeformWaypoint() = unchanged waypoint, want it deformed (sign inside activation distance)",
		)
	}
}

// TestDeformWaypoint_EmptySignListReturnsWaypointUnchanged matches
// test_empty_sign_list_returns_waypoint_unchanged.
func TestDeformWaypoint_EmptySignListReturnsWaypointUnchanged(t *testing.T) {
	t.Parallel()

	cfg := routerTestConfig(t)
	router := newTestRouter(t, nil, cfg)
	wp := trackmodel.Waypoint{X: 1.5, Y: 0.4}
	result := router.DeformWaypoint(
		wp,
		trackmodel.Waypoint{X: 1.4, Y: 0.4},
		0.0,
		trackmodel.South,
		nil,
	)

	if result != wp {
		t.Errorf("DeformWaypoint() with no signs = %+v, want unchanged %+v", result, wp)
	}
}

// TestRoutedSignPositions_ListsEverySignStillBeingRouted matches
// TestRoutedSignPositions.test_lists_every_sign_it_still_intends_to_route_around.
func TestRoutedSignPositions_ListsEverySignStillBeingRouted(t *testing.T) {
	t.Parallel()

	cfg := routerTestConfig(t)
	signs := []signrouter.SignSpec{
		{X: 1.5, Y: 0.4, Color: signrouter.SignColorRed},
		{X: 2.5, Y: 0.4, Color: signrouter.SignColorGreen},
	}
	router := newTestRouter(t, signs, cfg)

	got := router.RoutedSignPositions()
	want := []trackmodel.Waypoint{{X: 1.5, Y: 0.4}, {X: 2.5, Y: 0.4}}
	if len(got) != len(want) || got[0] != want[0] || got[1] != want[1] {
		t.Errorf("RoutedSignPositions() = %v, want %v", got, want)
	}
}

// TestRoutedSignPositions_RetiredSignIsDropped matches
// TestRoutedSignPositions.test_retired_sign_is_dropped_so_its_guard_comes_back:
// the collision layer withholds returns landing on these positions from its
// escape trigger, so this list is a safety-relevant contract.
func TestRoutedSignPositions_RetiredSignIsDropped(t *testing.T) {
	t.Parallel()

	cfg := routerTestConfig(t)
	signs := []signrouter.SignSpec{
		{X: 1.5, Y: 0.4, Color: signrouter.SignColorRed},
		{X: 2.5, Y: 0.4, Color: signrouter.SignColorGreen},
	}
	router := newTestRouter(t, signs, cfg)

	engagedGap := cfg.ActivationDistM / 4
	passedGap := cfg.PassedDistM + 0.2
	router.DeformWaypoint(
		trackmodel.Waypoint{X: 1.5, Y: 0.4},
		trackmodel.Waypoint{X: 1.5 - engagedGap, Y: 0.4},
		0.0,
		trackmodel.South,
		nil,
	)
	router.DeformWaypoint(
		trackmodel.Waypoint{X: 0.5, Y: 0.4},
		trackmodel.Waypoint{X: 1.5 + passedGap, Y: 0.4},
		0.0,
		trackmodel.South,
		nil,
	)

	got := router.RoutedSignPositions()
	want := trackmodel.Waypoint{X: 2.5, Y: 0.4}
	if len(got) != 1 || got[0] != want {
		t.Errorf("RoutedSignPositions() = %v, want [%v]", got, want)
	}
}

// TestRoutedSignPositions_EmptyWhenThereAreNoSigns matches
// TestRoutedSignPositions.test_empty_when_there_are_no_signs.
func TestRoutedSignPositions_EmptyWhenThereAreNoSigns(t *testing.T) {
	t.Parallel()

	router := newTestRouter(t, nil, routerTestConfig(t))
	if got := router.RoutedSignPositions(); len(got) != 0 {
		t.Errorf("RoutedSignPositions() = %v, want empty", got)
	}
}

// TestPassedSigns_NotDeformedOnceRetired matches
// TestPassedSigns.test_passed_sign_not_deformed.
func TestPassedSigns_NotDeformedOnceRetired(t *testing.T) {
	t.Parallel()

	cfg := routerTestConfig(t)
	sign := signrouter.SignSpec{X: 1.5, Y: 0.4, Color: signrouter.SignColorRed}
	router := newTestRouter(t, []signrouter.SignSpec{sign}, cfg)

	engagedGap := cfg.ActivationDistM / 4
	passedGap := cfg.PassedDistM + 0.2
	router.DeformWaypoint(
		trackmodel.Waypoint{X: 1.5, Y: 0.4},
		trackmodel.Waypoint{X: 1.5 - engagedGap, Y: 0.4},
		0.0,
		trackmodel.South,
		nil,
	)
	router.DeformWaypoint(
		trackmodel.Waypoint{X: 0.5, Y: 0.4},
		trackmodel.Waypoint{X: 1.5 + passedGap, Y: 0.4},
		0.0,
		trackmodel.South,
		nil,
	)

	wp := trackmodel.Waypoint{X: 1.5, Y: 0.4}
	result := router.DeformWaypoint(
		wp,
		trackmodel.Waypoint{X: 1.5 - engagedGap, Y: 0.4},
		0.0,
		trackmodel.South,
		nil,
	)
	if result != wp {
		t.Errorf("DeformWaypoint() on a retired sign = %+v, want unchanged %+v", result, wp)
	}
}

// TestPassedSigns_ActiveSignCountDecrements matches
// TestPassedSigns.test_active_sign_count_decrements.
func TestPassedSigns_ActiveSignCountDecrements(t *testing.T) {
	t.Parallel()

	cfg := routerTestConfig(t)
	// The two signs are spaced against the passed threshold, not a meter
	// apart: the point that retires the first has to still be short of the
	// second.
	firstX := 1.0
	passedGap := cfg.PassedDistM + 0.4
	secondX := firstX + passedGap
	signs := []signrouter.SignSpec{
		{X: firstX, Y: 0.4, Color: signrouter.SignColorRed},
		{X: secondX, Y: 0.4, Color: signrouter.SignColorGreen},
	}
	router := newTestRouter(t, signs, cfg)
	if router.ActiveSignCount() != 2 {
		t.Fatalf("ActiveSignCount() = %v, want 2", router.ActiveSignCount())
	}

	engagedGap := cfg.ActivationDistM / 4
	router.DeformWaypoint(
		trackmodel.Waypoint{X: firstX, Y: 0.4},
		trackmodel.Waypoint{X: firstX - engagedGap, Y: 0.4},
		0.0,
		trackmodel.South,
		nil,
	)
	router.DeformWaypoint(
		trackmodel.Waypoint{
			X: secondX,
			Y: 0.4,
		},
		trackmodel.Waypoint{X: firstX + cfg.PassedDistM + 0.2, Y: 0.4},
		0.0,
		trackmodel.South,
		nil,
	)

	if router.ActiveSignCount() != 1 {
		t.Errorf("ActiveSignCount() = %v, want 1", router.ActiveSignCount())
	}
}

// TestResetForNewLap_ReArmsPassedSigns matches
// TestPassedSigns.test_reset_for_new_lap_re_arms_passed_signs: every sign
// must route again each lap -- the Obstacles Challenge runs 3.
func TestResetForNewLap_ReArmsPassedSigns(t *testing.T) {
	t.Parallel()

	cfg := routerTestConfig(t)
	sign := signrouter.SignSpec{X: 1.5, Y: 0.4, Color: signrouter.SignColorRed}
	router := newTestRouter(t, []signrouter.SignSpec{sign}, cfg)

	engagedGap := cfg.ActivationDistM / 4
	passedGap := cfg.PassedDistM + 0.2
	router.DeformWaypoint(
		trackmodel.Waypoint{X: 1.5, Y: 0.4},
		trackmodel.Waypoint{X: 1.5 - engagedGap, Y: 0.4},
		0.0,
		trackmodel.South,
		nil,
	)
	router.DeformWaypoint(
		trackmodel.Waypoint{X: 0.5, Y: 0.4},
		trackmodel.Waypoint{X: 1.5 + passedGap, Y: 0.4},
		0.0,
		trackmodel.South,
		nil,
	)
	if router.ActiveSignCount() != 0 {
		t.Fatalf("ActiveSignCount() = %v, want 0 before reset", router.ActiveSignCount())
	}

	router.ResetForNewLap()
	if router.ActiveSignCount() != 1 {
		t.Fatalf("ActiveSignCount() = %v, want 1 after ResetForNewLap", router.ActiveSignCount())
	}

	wp := trackmodel.Waypoint{X: 1.5, Y: 0.4}
	result := router.DeformWaypoint(
		wp,
		trackmodel.Waypoint{X: 1.5 - engagedGap, Y: 0.4},
		0.0,
		trackmodel.South,
		nil,
	)
	if result == wp {
		t.Error(
			"DeformWaypoint() after ResetForNewLap = unchanged, want the re-armed sign to deform it again",
		)
	}
}

// TestSettleWindow_EngageAndPassSuppressedWithinWindow matches
// TestSettleWindow.test_engage_and_pass_suppressed_within_settle_window:
// right after spawn (or a lap boundary), the robot can briefly swing toward
// a corridor it hasn't actually reached yet. If that swing grazes a
// not-yet-really-encountered sign's activation radius, engage+pass
// bookkeeping with no settle window would retire the sign before its
// genuine pass.
func TestSettleWindow_EngageAndPassSuppressedWithinWindow(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	cfg.SettleTicks = 3
	sign := signrouter.SignSpec{X: 1.5, Y: 0.4, Color: signrouter.SignColorRed}
	router := newTestRouter(t, []signrouter.SignSpec{sign}, cfg)

	engagedGap := cfg.ActivationDistM / 4
	passedGap := cfg.PassedDistM + 0.2

	// Ticks 1-2: engage-then-leave, still within the settle window.
	router.DeformWaypoint(
		trackmodel.Waypoint{X: 1.5, Y: 0.4},
		trackmodel.Waypoint{X: 1.5 - engagedGap, Y: 0.4},
		0.0,
		trackmodel.South,
		nil,
	)
	router.DeformWaypoint(
		trackmodel.Waypoint{X: 0.5, Y: 0.4},
		trackmodel.Waypoint{X: 1.5 + passedGap, Y: 0.4},
		0.0,
		trackmodel.South,
		nil,
	)
	if router.ActiveSignCount() != 1 {
		t.Fatalf(
			"ActiveSignCount() = %v within the settle window, want 1 (not retired)",
			router.ActiveSignCount(),
		)
	}

	// Tick 3: burn the remaining unsettled tick.
	router.DeformWaypoint(
		trackmodel.Waypoint{X: 1.5, Y: 0.4},
		trackmodel.Waypoint{X: 10.0, Y: 10.0},
		0.0,
		trackmodel.South,
		nil,
	)

	// Ticks 4-5: past the settle window, the same sequence retires it.
	router.DeformWaypoint(
		trackmodel.Waypoint{X: 1.5, Y: 0.4},
		trackmodel.Waypoint{X: 1.5 - engagedGap, Y: 0.4},
		0.0,
		trackmodel.South,
		nil,
	)
	router.DeformWaypoint(
		trackmodel.Waypoint{X: 0.5, Y: 0.4},
		trackmodel.Waypoint{X: 1.5 + passedGap, Y: 0.4},
		0.0,
		trackmodel.South,
		nil,
	)
	if router.ActiveSignCount() != 0 {
		t.Errorf(
			"ActiveSignCount() = %v past the settle window, want 0 (retired)",
			router.ActiveSignCount(),
		)
	}
}

// TestSettleWindow_CandidateSelectionNotSuppressed matches
// TestSettleWindow.test_candidate_selection_not_suppressed_within_settle_window:
// a sign in the robot's real corridor still deforms during the settle
// window -- only the engage/pass bookkeeping is deferred.
func TestSettleWindow_CandidateSelectionNotSuppressed(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	const largeSettleTicks = 100
	cfg.SettleTicks = largeSettleTicks
	sign := signrouter.SignSpec{X: 1.5, Y: 0.4, Color: signrouter.SignColorRed}
	router := newTestRouter(t, []signrouter.SignSpec{sign}, cfg)

	wp := trackmodel.Waypoint{X: 1.5, Y: 0.4}
	engagedGap := cfg.ActivationDistM / 4
	result := router.DeformWaypoint(
		wp,
		trackmodel.Waypoint{X: 1.5 - engagedGap, Y: 0.4},
		0.0,
		trackmodel.South,
		nil,
	)
	if result == wp {
		t.Error("DeformWaypoint() within a long settle window = unchanged, want it still deformed")
	}
}

// TestSettleWindow_ResetForNewLapRestartsTheWindow matches
// TestSettleWindow.test_reset_for_new_lap_restarts_the_settle_window.
func TestSettleWindow_ResetForNewLapRestartsTheWindow(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	cfg.SettleTicks = 3
	sign := signrouter.SignSpec{X: 1.5, Y: 0.4, Color: signrouter.SignColorRed}
	router := newTestRouter(t, []signrouter.SignSpec{sign}, cfg)

	engagedGap := cfg.ActivationDistM / 4
	passedGap := cfg.PassedDistM + 0.2
	const settleBurnTicks = 4
	for range settleBurnTicks {
		router.DeformWaypoint(
			trackmodel.Waypoint{X: 1.5, Y: 0.4}, trackmodel.Waypoint{X: 1.5 - engagedGap, Y: 0.4},
			0.0, trackmodel.South, nil,
		)
	}
	router.DeformWaypoint(
		trackmodel.Waypoint{X: 0.5, Y: 0.4},
		trackmodel.Waypoint{X: 1.5 + passedGap, Y: 0.4},
		0.0,
		trackmodel.South,
		nil,
	)
	if router.ActiveSignCount() != 0 {
		t.Fatalf(
			"ActiveSignCount() = %v, want 0 (settled already, so this retired it)",
			router.ActiveSignCount(),
		)
	}

	router.ResetForNewLap()
	// Immediately after reset, back inside a fresh settle window.
	router.DeformWaypoint(
		trackmodel.Waypoint{X: 1.5, Y: 0.4},
		trackmodel.Waypoint{X: 1.5 - engagedGap, Y: 0.4},
		0.0,
		trackmodel.South,
		nil,
	)
	router.DeformWaypoint(
		trackmodel.Waypoint{X: 0.5, Y: 0.4},
		trackmodel.Waypoint{X: 1.5 + passedGap, Y: 0.4},
		0.0,
		trackmodel.South,
		nil,
	)
	if router.ActiveSignCount() != 1 {
		t.Errorf(
			"ActiveSignCount() = %v, want 1 (fresh settle window suppressed the retirement)",
			router.ActiveSignCount(),
		)
	}
}

// TestEngagementGating_DistantSignAtSpawnNotPrematurelyPassed matches
// TestEngagementGating.test_distant_sign_at_spawn_not_prematurely_passed: a
// sign is only retired once approached -- never discarded from afar. The
// buggy behavior marked a far-away sign passed on the first tick, silently
// disabling routing.
func TestEngagementGating_DistantSignAtSpawnNotPrematurelyPassed(t *testing.T) {
	t.Parallel()

	cfg := routerTestConfig(t)
	sign := signrouter.SignSpec{X: 2.0, Y: 0.4, Color: signrouter.SignColorRed}
	router := newTestRouter(t, []signrouter.SignSpec{sign}, cfg)

	spawnWp := trackmodel.Waypoint{X: 0.4, Y: 0.4}
	result := router.DeformWaypoint(
		spawnWp,
		spawnWp,
		0.0,
		trackmodel.South,
		nil,
	) // d = 1.6m > passed_dist
	if result != spawnWp {
		t.Errorf(
			"DeformWaypoint() at spawn = %+v, want unchanged %+v (too far to deform yet)",
			result,
			spawnWp,
		)
	}
	if router.ActiveSignCount() != 1 {
		t.Fatalf(
			"ActiveSignCount() = %v, want 1 (still active, not retired)",
			router.ActiveSignCount(),
		)
	}

	approachWp := trackmodel.Waypoint{X: 2.0, Y: 0.4}
	approached := router.DeformWaypoint(
		approachWp,
		trackmodel.Waypoint{X: 1.7, Y: 0.4},
		0.0,
		trackmodel.South,
		nil,
	)
	if approached == approachWp {
		t.Error("DeformWaypoint() once approached = unchanged, want it deformed")
	}
}
