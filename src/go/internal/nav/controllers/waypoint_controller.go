package controllers

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// WaypointController is a pure pursuit steering controller for waypoint
// following, matching waypoint_controller.WaypointController.
//
// Computes steering angle to intercept a lookahead point on the planned
// path. Adapts lookahead distance based on crosstrack error and upcoming
// turn to handle corners (short lookahead) vs. straights (long lookahead).
type WaypointController struct {
	// MaxSteeringAngle is the physical steering limit in radians.
	MaxSteeringAngle float64
	// WheelbaseM is the chassis wheelbase (m), fed to navutil.PurePursuitSteer.
	WheelbaseM float64
	// YawGainCompensation is the fraction of predicted yaw the chassis
	// delivers; see controllers.Config.YawGainCompensation.
	YawGainCompensation float64
	// LookaheadShort/LookaheadLong are the lookahead distances for sharp
	// corners / straight sections (m).
	LookaheadShort, LookaheadLong float64
	// LookaheadTransition is the crosstrack error threshold (m) above
	// which the short lookahead engages.
	LookaheadTransition float64
	// MaxSteeringRate is the maximum steering command rate (rad/s).
	MaxSteeringRate float64
	// WaypointReachedDistanceM is the distance below which the current
	// target waypoint is considered reached (m).
	WaypointReachedDistanceM float64
	// CornerTurnThresholdRad is the upcoming-turn threshold above which
	// the short lookahead engages regardless of crosstrack error.
	CornerTurnThresholdRad float64
	// LookaheadBlendStart is the fraction of either threshold at which
	// the lookahead starts sliding from long toward short. 1.0 (the
	// default) reproduces a hard switch exactly.
	LookaheadBlendStart float64
	// TargetSearchSpanM is how far ALONG THE PATH SelectTargetPoint may walk,
	// in meters. 0 means a whole lap -- see the span comment in
	// SelectTargetPoint. Mirrors TARGET_SEARCH_SPAN_M.
	TargetSearchSpanM float64
	// TargetSenseGate rejects a candidate the chassis would reach by going
	// round the loop the WRONG WAY. Mirrors TARGET_SENSE_GATE (ships false).
	TargetSenseGate bool

	prevSteeringRad   float64
	crosstrackBudgetM *float64
}

// minSearchCandidates is how many waypoints SelectTargetPoint always
// examines, whatever the span says. A path can legitimately be coarser than
// the span (the bay's first few points, any synthetic path), and below a
// handful of candidates the search stops being a search -- so the floor wins
// over the span. Mirrors waypoint_controller._MIN_SEARCH_CANDIDATES.
const minSearchCandidates = 5

// minNormLen is the shortest (dx, dy) vector unitOrNone will normalise. Below
// it the vector is too short to have a reliable bearing, matching
// waypoint_controller._UNIT_EPSILON.
const minNormLen = 1e-12

// EffectiveTransition is the crosstrack threshold actually in force, after
// the wall budget, matching WaypointController.effective_transition.
func (w *WaypointController) EffectiveTransition() float64 {
	if w.crosstrackBudgetM == nil {
		return w.LookaheadTransition
	}
	return min(w.LookaheadTransition, *w.crosstrackBudgetM)
}

// SetCrosstrackBudget declares how far this path may be strayed from
// before a wall is hit, matching WaypointController.set_crosstrack_budget.
// Only ever tightens the threshold. Pass nil to drop back to the
// configured value.
func (w *WaypointController) SetCrosstrackBudget(budgetM *float64) {
	w.crosstrackBudgetM = budgetM
}

// SelectLookahead selects the lookahead distance from off-path distance
// and upcoming turn, matching WaypointController.select_lookahead.
func (w *WaypointController) SelectLookahead(
	crosstrackError, turnAheadRad float64,
	signAhead bool,
) float64 {
	signDemand := 0.0
	if signAhead {
		signDemand = 1.0
	}
	demand := max(
		w.demand(crosstrackError, w.EffectiveTransition()),
		w.demand(turnAheadRad, w.CornerTurnThresholdRad),
		signDemand,
	)
	return w.LookaheadLong + (w.LookaheadShort-w.LookaheadLong)*demand
}

// SelectTargetPoint finds the path point at least lookaheadDistance ahead
// of the chassis, matching WaypointController.select_target_point.
//
// Searches forward from waypointIndex, wrapping around the end of the list
// back to the start (at most one full lap). waypoints is the full
// canonical-lap path, not a pre-sliced remainder. Also skips any candidate
// that is behind the chassis in its current local frame. pose is the
// chassis pose in the world frame.
func (w *WaypointController) SelectTargetPoint(
	pose trackmodel.Pose,
	waypointsPath []trackmodel.Waypoint,
	waypointIndex int,
	lookaheadDistance float64,
) trackmodel.Waypoint {
	n := len(waypointsPath)
	cosYaw, sinYaw := math.Cos(pose.Yaw), math.Sin(pose.Yaw)

	var nearestAhead *trackmodel.Waypoint
	nearestAheadDist := math.Inf(1)
	nearestAny := waypointsPath[((waypointIndex%n)+n)%n]
	nearestAnyDist := math.Inf(1)

	// How far ALONG THE PATH the scan may walk. Without a bound this loop
	// wraps a whole lap and returns the first waypoint merely geometrically
	// in front of the chassis -- which, once the chassis has turned toward
	// the way it came, is on the FAR SIDE OF THE RING. The bound removes
	// every such pathological pick and is raised to at least
	// lookaheadDistance so it can never starve the search. The modulo stays,
	// so the seam fix is untouched. See adr:0052-pursuit-target-selection.
	spanM := w.TargetSearchSpanM
	walked := 0.0
	for offset := range n {
		if spanM > 0.0 && offset >= minSearchCandidates && walked > max(spanM, lookaheadDistance) {
			break
		}
		index := safeMod(waypointIndex+offset, n)
		if offset > 0 {
			prev := waypointsPath[safeMod(waypointIndex+offset-1, n)]
			walked += math.Hypot(waypointsPath[index].X-prev.X, waypointsPath[index].Y-prev.Y)
		}
		wp := waypointsPath[index]
		dx, dy := wp.X-pose.X, wp.Y-pose.Y
		dist := math.Hypot(dx, dy)
		if dist < nearestAnyDist {
			nearestAnyDist = dist
			nearestAny = wp
		}
		xLocal := dx*cosYaw + dy*sinYaw
		if xLocal <= 0 {
			continue
		}
		if w.TargetSenseGate && !agreesWithPathSense(dx, dy, dist, waypointsPath, waypointIndex+offset) {
			continue
		}
		if dist >= lookaheadDistance {
			return wp
		}
		if dist < nearestAheadDist {
			nearestAheadDist = dist
			wpCopy := wp
			nearestAhead = &wpCopy
		}
	}

	if nearestAhead != nil {
		return *nearestAhead
	}
	return nearestAny
}

// Reset clears the steering-rate-limit memory, matching
// WaypointController.reset. Call this whenever something other than this
// controller has just driven the steering command.
func (w *WaypointController) Reset() {
	w.prevSteeringRad = 0.0
}

// ComputeSteering computes steering angle, lookahead, and heading error
// for the next control step, matching WaypointController.compute_steering.
//
// Curvature-based pure pursuit (see navutil.PurePursuitSteer), not a gain
// on heading error. Returns (steeringNormalized, lookaheadDistanceM,
// angleErrorRad): steeringNormalized is the command in [-1, 1] after rate
// limiting; angleErrorRad is the signed bearing error before rate
// limiting, letting the caller slow down for a sharp turn.
func (w *WaypointController) ComputeSteering(
	pose trackmodel.Pose, targetWaypoint trackmodel.Waypoint,
	crosstrackError, dt float64,
) (steeringNormalized, lookaheadDistanceM, angleErrorRad float64) {
	lookahead := w.SelectLookahead(crosstrackError, 0.0, false)

	xLocal, yLocal := pose.ToLocalFrame(targetWaypoint)
	distance := math.Hypot(xLocal, yLocal)

	if distance < w.WaypointReachedDistanceM {
		return 0.0, lookahead, 0.0
	}

	angleError := math.Atan2(yLocal, xLocal)

	var steeringNormalizedRaw float64
	if xLocal > 0 {
		steeringNormalizedRaw = navutil.PurePursuitSteer(
			xLocal, yLocal, w.WaypointReachedDistanceM, w.WheelbaseM, w.MaxSteeringAngle,
			w.YawGainCompensation,
		)
	} else {
		// Target behind the robot: the curvature formula is only valid
		// for a roughly-forward target. Saturate toward whichever side
		// it's on instead of trusting a formula that can look plausible
		// while actually steering away from the target.
		steeringNormalizedRaw = 1.0
		if yLocal < 0 {
			steeringNormalizedRaw = -1.0
		}
	}

	steeringRad := steeringNormalizedRaw * w.MaxSteeringAngle

	// Rate-limit against the previous tick's command.
	maxDelta := w.MaxSteeringRate * dt
	steeringRad = max(w.prevSteeringRad-maxDelta, min(w.prevSteeringRad+maxDelta, steeringRad))
	w.prevSteeringRad = steeringRad

	return steeringRad / w.MaxSteeringAngle, lookahead, angleError
}

// agreesWithPathSense reports whether the chassis would reach a candidate
// travelling the path's own way, matching
// waypoint_controller._agrees_with_path_sense. The bearing from the pose to
// the candidate is projected on the path's direction of travel AT the
// candidate; a negative projection means the approach runs against the path
// -- the candidate is reached by going round the loop the wrong way.
//
// This covers a different quantity from TargetSearchSpanM: a closed loop has
// two tangent directions at every point, and a point one meter along the path
// in the wrong sense is still one meter away and still in the forward
// half-plane of a rotated chassis. The span bound alone converted the
// reversal rather than closing it. See adr:0052-pursuit-target-selection.
func agreesWithPathSense(dx, dy, dist float64, path []trackmodel.Waypoint, index int) bool {
	if dist <= 0.0 {
		return true
	}
	n := len(path)
	wp := path[safeMod(index, n)]
	next := path[safeMod(index+1, n)]
	// A duplicated waypoint has no outgoing direction, so it cannot disagree.
	px, py, ok := unitOrNone(next.X-wp.X, next.Y-wp.Y)
	if !ok {
		return true
	}
	return (dx/dist)*px+(dy/dist)*py > 0.0
}

// unitOrNone normalises (dx, dy), reporting false when it is too short to
// have a bearing. Mirrors waypoint_controller._unit_or_none.
func unitOrNone(dx, dy float64) (nx, ny float64, ok bool) {
	n := math.Hypot(dx, dy)
	if n <= minNormLen {
		return 0.0, 0.0, false
	}
	return dx / n, dy / n, true
}

// safeMod returns i mod n in [0, n), matching Python's % for negative i.
func safeMod(i, n int) int {
	return ((i % n) + n) % n
}

// demand is how far toward the short lookahead one signal asks to go, in
// [0, 1], matching WaypointController._demand.
func (w *WaypointController) demand(value, threshold float64) float64 {
	if threshold <= 0.0 {
		return 0.0
	}
	ratio := value / threshold
	if w.LookaheadBlendStart >= 1.0 {
		if ratio > 1.0 {
			return 1.0
		}
		return 0.0
	}
	span := 1.0 - w.LookaheadBlendStart
	return navutil.Clamp((ratio-w.LookaheadBlendStart)/span, 0.0, 1.0)
}
