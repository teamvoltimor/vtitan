package controllers_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// TestComputeSteering_LargeAngleErrorIsRateLimitedAcrossTicks ports
// test_large_angle_error_is_rate_limited_across_ticks: a target directly
// behind the robot is outside the curvature formula's valid range (x_local
// <= 0), so it saturates to full lock -- same end result as the old
// P-controller's clamp, but via the explicit target-behind fallback rather
// than an oversized gain.
func TestComputeSteering_LargeAngleErrorIsRateLimitedAcrossTicks(t *testing.T) {
	t.Parallel()

	controller := newDefaultWaypointController()
	controller.MaxSteeringRate = 2.0
	const dt = 0.05
	maxDeltaNorm := (2.0 * dt) / controller.MaxSteeringAngle
	target := trackmodel.Waypoint{X: -1.0, Y: 0.0}

	first, _, _ := controller.ComputeSteering(trackmodel.Waypoint{}, 0.0, target, 0.0, dt)
	if math.Abs(first-maxDeltaNorm) > 1e-9 {
		t.Errorf("first = %v, want %v", first, maxDeltaNorm)
	}

	second, _, _ := controller.ComputeSteering(trackmodel.Waypoint{}, 0.0, target, 0.0, dt)
	if math.Abs(math.Abs(second-first)-maxDeltaNorm) > 1e-9 {
		t.Errorf("abs(second-first) = %v, want %v", math.Abs(second-first), maxDeltaNorm)
	}
}

// TestComputeSteering_SmallAngleErrorIsNotRateLimited ports
// test_small_angle_error_is_not_rate_limited.
func TestComputeSteering_SmallAngleErrorIsNotRateLimited(t *testing.T) {
	t.Parallel()

	controller := newDefaultWaypointController()
	controller.MaxSteeringRate = 2.0

	steering, _, angleError := controller.ComputeSteering(
		trackmodel.Waypoint{}, 0.0, trackmodel.Waypoint{X: 1.0, Y: 0.01}, 0.0, 0.05,
	)
	if !(steering > 0.0 && steering < 0.05) {
		t.Errorf("steering = %v, want in (0, 0.05)", steering)
	}
	wantAngleError := math.Atan2(0.01, 1.0)
	if math.Abs(angleError-wantAngleError) > 1e-9 {
		t.Errorf("angleError = %v, want %v", angleError, wantAngleError)
	}
}

// TestReset_ClearsRateLimitMemory ports test_reset_clears_rate_limit_memory.
func TestReset_ClearsRateLimitMemory(t *testing.T) {
	t.Parallel()

	controller := newDefaultWaypointController()
	controller.MaxSteeringRate = 2.0
	const dt = 0.05
	target := trackmodel.Waypoint{X: -1.0, Y: 0.0}

	controller.ComputeSteering(trackmodel.Waypoint{}, 0.0, target, 0.0, dt)
	controller.Reset()

	steering, _, _ := controller.ComputeSteering(trackmodel.Waypoint{}, 0.0, target, 0.0, dt)
	maxDeltaNorm := (2.0 * dt) / controller.MaxSteeringAngle
	if math.Abs(steering-maxDeltaNorm) > 1e-9 {
		t.Errorf("steering after reset = %v, want %v", steering, maxDeltaNorm)
	}
}

// TestComputeSteering_ForwardTargetUsesCurvatureNotGain ports
// test_forward_target_uses_curvature_not_gain: a target ahead and to the
// left should steer left (positive), with a magnitude set by chassis
// geometry (wheelbase/2), not an arbitrary gain.
func TestComputeSteering_ForwardTargetUsesCurvatureNotGain(t *testing.T) {
	t.Parallel()

	controller := newDefaultWaypointController()
	controller.MaxSteeringRate = 100.0 // effectively unrated for this check

	steering, _, angleError := controller.ComputeSteering(
		trackmodel.Waypoint{}, 0.0, trackmodel.Waypoint{X: 0.40, Y: 0.20}, 0.0, 0.05,
	)
	if angleError <= 0 {
		t.Errorf("angleError = %v, want > 0", angleError)
	}
	if !(steering > 0 && steering < 1.0) {
		t.Errorf("steering = %v, want in (0, 1.0)", steering)
	}
}

// TestSelectTargetPoint_WrapsPastTheEndOfTheLap ports
// TestSelectTargetPointWrapsAndStaysAhead.test_wraps_past_the_end_of_the_lap_instead_of_going_dry:
// 2026-08-03, select_target_point used to be handed a pre-sliced remainder
// and searched it by distance alone, ignoring heading. Near the end of a lap
// that slice could run dry instead of continuing around the loop --
// measured on real hardware as steering pinned near zero for tens of
// seconds while heading drifted 85+ degrees.
func TestSelectTargetPoint_WrapsPastTheEndOfTheLap(t *testing.T) {
	t.Parallel()

	controller := newDefaultWaypointController()
	// A small closed loop. Starting the search at the last index, nothing
	// from there to the literal end of the slice reaches the lookahead
	// distance -- only continuing around to index 1 does.
	waypoints := []trackmodel.Waypoint{{X: 0.0, Y: 0.0}, {X: 1.0, Y: 0.0}, {X: 1.0, Y: 1.0}, {X: 0.0, Y: 1.0}}

	target := controller.SelectTargetPoint(trackmodel.Waypoint{X: 0.0, Y: 1.0}, 0.0, waypoints, 3, 1.0)
	want := trackmodel.Waypoint{X: 1.0, Y: 0.0}
	if target != want {
		t.Errorf("SelectTargetPoint() = %+v, want %+v", target, want)
	}
}

// TestSelectTargetPoint_SkipsABehindCandidateEvenWhenFartherByDistance
// ports test_skips_a_behind_candidate_even_when_it_is_farther_by_distance:
// index 1 is far enough by pure distance but behind the chassis; index 2 is
// closer but ahead. A distance-only, array-order search would have returned
// index 1 first -- measured as a wrong-direction turn on real hardware.
func TestSelectTargetPoint_SkipsABehindCandidateEvenWhenFartherByDistance(t *testing.T) {
	t.Parallel()

	controller := newDefaultWaypointController()
	waypoints := []trackmodel.Waypoint{{X: 0.0, Y: 0.0}, {X: -5.0, Y: 0.0}, {X: 3.0, Y: 0.0}}

	target := controller.SelectTargetPoint(trackmodel.Waypoint{}, 0.0, waypoints, 0, 1.0)
	want := trackmodel.Waypoint{X: 3.0, Y: 0.0}
	if target != want {
		t.Errorf("SelectTargetPoint() = %+v, want %+v", target, want)
	}
}

// TestSelectTargetPoint_FallsBackToNearestAheadWhenNothingReachesLookahead
// ports test_falls_back_to_nearest_ahead_when_nothing_reaches_lookahead:
// 2026-08-04, was "farthest ahead", which starved the curvature formula on
// real hardware and got reversed to nearest.
func TestSelectTargetPoint_FallsBackToNearestAheadWhenNothingReachesLookahead(t *testing.T) {
	t.Parallel()

	controller := newDefaultWaypointController()
	waypoints := []trackmodel.Waypoint{{X: 0.0, Y: 0.0}, {X: 0.1, Y: 0.0}, {X: 0.2, Y: 0.0}}

	// lookahead 5.0 is farther than anything on this tiny loop.
	target := controller.SelectTargetPoint(trackmodel.Waypoint{}, 0.0, waypoints, 0, 5.0)
	want := trackmodel.Waypoint{X: 0.1, Y: 0.0} // nearest point that is still ahead
	if target != want {
		t.Errorf("SelectTargetPoint() = %+v, want %+v", target, want)
	}
}

// -- TestCrosstrackBudgetFromWallDistance: the crosstrack threshold must not
// exceed what the path can afford.

// TestEffectiveTransition_DefaultsToTheConfiguredTransition ports
// test_defaults_to_the_configured_transition.
func TestEffectiveTransition_DefaultsToTheConfiguredTransition(t *testing.T) {
	t.Parallel()

	controller := newDefaultWaypointController()
	controller.LookaheadTransition = 0.30
	if got := controller.EffectiveTransition(); math.Abs(got-0.30) > 1e-9 {
		t.Errorf("EffectiveTransition() = %v, want 0.30", got)
	}
}

// TestEffectiveTransition_ATightBudgetLowersTheThreshold ports
// test_a_tight_budget_lowers_the_threshold.
func TestEffectiveTransition_ATightBudgetLowersTheThreshold(t *testing.T) {
	t.Parallel()

	controller := newDefaultWaypointController()
	controller.LookaheadTransition = 0.30
	budget := 0.123
	controller.SetCrosstrackBudget(&budget)

	if got := controller.EffectiveTransition(); math.Abs(got-0.123) > 1e-9 {
		t.Errorf("EffectiveTransition() = %v, want 0.123", got)
	}
}

// TestEffectiveTransition_AGenerousBudgetDoesNotRaiseIt ports
// test_a_generous_budget_does_not_raise_it: a wide corridor must behave
// exactly as before, not steer twitchier.
func TestEffectiveTransition_AGenerousBudgetDoesNotRaiseIt(t *testing.T) {
	t.Parallel()

	controller := newDefaultWaypointController()
	controller.LookaheadTransition = 0.30
	budget := 0.373
	controller.SetCrosstrackBudget(&budget)

	if got := controller.EffectiveTransition(); math.Abs(got-0.30) > 1e-9 {
		t.Errorf("EffectiveTransition() = %v, want 0.30 (a generous budget must not raise it)", got)
	}
}

// TestEffectiveTransition_ClearingTheBudgetRestoresTheConfiguredValue ports
// test_clearing_the_budget_restores_the_configured_value.
func TestEffectiveTransition_ClearingTheBudgetRestoresTheConfiguredValue(t *testing.T) {
	t.Parallel()

	controller := newDefaultWaypointController()
	controller.LookaheadTransition = 0.30
	budget := 0.10
	controller.SetCrosstrackBudget(&budget)
	controller.SetCrosstrackBudget(nil)

	if got := controller.EffectiveTransition(); math.Abs(got-0.30) > 1e-9 {
		t.Errorf("EffectiveTransition() = %v, want 0.30", got)
	}
}

// TestSelectLookahead_ShortLookaheadEngagesWithinTheBudget ports
// test_short_lookahead_engages_within_the_budget: the point of the whole
// crosstrack-budget mechanism -- 0.15m of crosstrack must arm the correction
// on a wall-hugging path, where it previously did not.
func TestSelectLookahead_ShortLookaheadEngagesWithinTheBudget(t *testing.T) {
	t.Parallel()

	controller := newDefaultWaypointController()
	controller.LookaheadTransition = 0.30
	controller.LookaheadShort = 0.20
	controller.LookaheadLong = 0.40

	if got := controller.SelectLookahead(0.15, 0.0, false); math.Abs(got-0.40) > 1e-9 {
		t.Errorf("SelectLookahead(0.15) before a budget = %v, want 0.40", got)
	}

	budget := 0.123
	controller.SetCrosstrackBudget(&budget)

	if got := controller.SelectLookahead(0.15, 0.0, false); math.Abs(got-0.20) > 1e-9 {
		t.Errorf("SelectLookahead(0.15) within the budget = %v, want 0.20", got)
	}
}

// -- TestUpcomingTurnArmsTheShortLookahead: crosstrack error cannot rise
// until a corner has already been missed, so gating on it alone lands the
// sharp correction after the corner. Previewing the planned path's own turn
// arms the short lookahead on entry instead.

func upcomingTurnController(t *testing.T) *controllers.WaypointController {
	t.Helper()
	controller := newDefaultWaypointController()
	controller.LookaheadTransition = 0.30
	controller.LookaheadShort = 0.20
	controller.LookaheadLong = 0.40
	controller.CornerTurnThresholdRad = 0.35
	return controller
}

// TestSelectLookahead_StraightAheadKeepsTheLongLookahead ports
// test_straight_ahead_keeps_the_long_lookahead: a straight must behave
// exactly as before -- a short lookahead there is twitchy, which is the
// failure this whole mechanism trades against.
func TestSelectLookahead_StraightAheadKeepsTheLongLookahead(t *testing.T) {
	t.Parallel()

	controller := upcomingTurnController(t)
	if got := controller.SelectLookahead(0.01, 0.0, false); math.Abs(got-0.40) > 1e-9 {
		t.Errorf("SelectLookahead(0.01, 0.0) = %v, want 0.40", got)
	}
}

// TestSelectLookahead_CornerAheadShortensItWhileStillOnPath ports
// test_corner_ahead_shortens_it_while_still_on_path: 0.40m of preview on the
// default 0.45m corner arc turns ~0.89 rad.
func TestSelectLookahead_CornerAheadShortensItWhileStillOnPath(t *testing.T) {
	t.Parallel()

	controller := upcomingTurnController(t)
	if got := controller.SelectLookahead(0.01, 0.89, false); math.Abs(got-0.20) > 1e-9 {
		t.Errorf("SelectLookahead(0.01, 0.89) = %v, want 0.20", got)
	}
}

// TestSelectLookahead_AGentleBendIsNotTreatedAsACorner ports
// test_a_gentle_bend_is_not_treated_as_a_corner.
func TestSelectLookahead_AGentleBendIsNotTreatedAsACorner(t *testing.T) {
	t.Parallel()

	controller := upcomingTurnController(t)
	if got := controller.SelectLookahead(0.01, 0.20, false); math.Abs(got-0.40) > 1e-9 {
		t.Errorf("SelectLookahead(0.01, 0.20) = %v, want 0.40", got)
	}
}

// TestSelectLookahead_CrosstrackStillArmsItWithNoTurnAhead ports
// test_crosstrack_still_arms_it_with_no_turn_ahead: the lagging crosstrack
// signal stays wired -- the turn-preview signal is added, not swapped.
func TestSelectLookahead_CrosstrackStillArmsItWithNoTurnAhead(t *testing.T) {
	t.Parallel()

	controller := upcomingTurnController(t)
	if got := controller.SelectLookahead(0.35, 0.0, false); math.Abs(got-0.20) > 1e-9 {
		t.Errorf("SelectLookahead(0.35, 0.0) = %v, want 0.20", got)
	}
}

// TestSelectLookahead_OmittingTheTurnPreservesTheOldBehaviour ports
// test_omitting_the_turn_preserves_the_old_behavior.
func TestSelectLookahead_OmittingTheTurnPreservesTheOldBehaviour(t *testing.T) {
	t.Parallel()

	controller := upcomingTurnController(t)
	if got := controller.SelectLookahead(0.01, 0.0, false); math.Abs(got-0.40) > 1e-9 {
		t.Errorf("SelectLookahead(0.01, 0.0) = %v, want 0.40", got)
	}
	if got := controller.SelectLookahead(0.35, 0.0, false); math.Abs(got-0.20) > 1e-9 {
		t.Errorf("SelectLookahead(0.35, 0.0) = %v, want 0.20", got)
	}
}

// -- TestLookaheadRampsRatherThanSwitching: the switch used to be a step,
// and both arming signals sit near their thresholds in normal driving, so
// it flipped long/short on consecutive ticks (hardware
// run_20260829_104641: 0.320, 0.160, 0.320, 0.160 at ~2.5Hz). Curvature is
// 2y/L**2, so each flip swung the command by 4x and the chassis drew a
// visible zigzag.

func rampController(t *testing.T) *controllers.WaypointController {
	t.Helper()
	controller := newDefaultWaypointController()
	controller.LookaheadTransition = 0.30
	controller.LookaheadShort = 0.20
	controller.LookaheadLong = 0.40
	controller.CornerTurnThresholdRad = 0.35
	controller.LookaheadBlendStart = 0.70
	return controller
}

// TestSelectLookahead_NoSingleStepMovesTheLookaheadFar ports
// test_no_single_step_moves_the_lookahead_far: the actual anti-zigzag
// property -- a small change in crosstrack cannot produce a large change in
// lookahead. Under the old hard step this failed by construction: one tick
// either side of the threshold spanned the whole 0.20m range.
func TestSelectLookahead_NoSingleStepMovesTheLookaheadFar(t *testing.T) {
	t.Parallel()

	controller := rampController(t)
	const steps = 81
	looks := make([]float64, steps)
	for i := range steps {
		looks[i] = controller.SelectLookahead(float64(i)*0.005, 0.0, false) // 0 -> 0.40m in 5mm steps
	}
	biggestJump := 0.0
	for i := 1; i < steps; i++ {
		if jump := math.Abs(looks[i] - looks[i-1]); jump > biggestJump {
			biggestJump = jump
		}
	}
	span := controller.LookaheadLong - controller.LookaheadShort
	if biggestJump > span/4 {
		t.Errorf("biggestJump = %v, want <= %v (span/4)", biggestJump, span/4)
	}
}

// TestSelectLookahead_NeverIncreasesAsTheRobotStrays ports
// test_lookahead_never_increases_as_the_robot_strays: monotone -- straying
// further may only tighten tracking, never loosen it. A non-monotone blend
// would let a worsening error relax the correction, which is the failure
// mode this whole path exists to avoid.
func TestSelectLookahead_NeverIncreasesAsTheRobotStrays(t *testing.T) {
	t.Parallel()

	controller := rampController(t)
	const steps = 41
	looks := make([]float64, steps)
	for i := range steps {
		looks[i] = controller.SelectLookahead(float64(i)*0.01, 0.0, false)
	}
	for i := 1; i < steps; i++ {
		if looks[i] > looks[i-1]+1e-12 {
			t.Fatalf("looks[%d] (%v) > looks[%d] (%v), want non-increasing", i, looks[i], i-1, looks[i-1])
		}
	}
}

// TestSelectLookahead_TheDeadbandLeavesStraightsUntouched ports
// test_the_deadband_leaves_straights_untouched: below the blend start the
// long lookahead must be exactly unchanged, so this cannot make calm
// driving twitchier than it already was.
func TestSelectLookahead_TheDeadbandLeavesStraightsUntouched(t *testing.T) {
	t.Parallel()

	controller := rampController(t)
	below := 0.70 * controller.EffectiveTransition() * 0.99
	if got := controller.SelectLookahead(below, 0.0, false); math.Abs(got-controller.LookaheadLong) > 1e-9 {
		t.Errorf("SelectLookahead(below deadband) = %v, want %v", got, controller.LookaheadLong)
	}
}

// TestSelectLookahead_BlendStartOfOneRestoresTheHardSwitch ports
// test_blend_start_of_one_restores_the_hard_switch: the escape hatch -- 1.0
// must reproduce the original step exactly, so the ramp can be disabled
// from config without a code change.
func TestSelectLookahead_BlendStartOfOneRestoresTheHardSwitch(t *testing.T) {
	t.Parallel()

	controller := rampController(t)
	controller.LookaheadBlendStart = 1.0
	transition := controller.EffectiveTransition()

	if got := controller.SelectLookahead(transition*0.99, 0.0, false); math.Abs(got-controller.LookaheadLong) > 1e-9 {
		t.Errorf("SelectLookahead(just under threshold) = %v, want %v", got, controller.LookaheadLong)
	}
	if got := controller.SelectLookahead(transition*1.01, 0.0, false); math.Abs(got-controller.LookaheadShort) > 1e-9 {
		t.Errorf("SelectLookahead(just over threshold) = %v, want %v", got, controller.LookaheadShort)
	}
}
