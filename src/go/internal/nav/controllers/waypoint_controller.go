package controllers

import (
	"math"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
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

	prevSteeringRad   float64
	crosstrackBudgetM *float64
}

// NewWaypointController builds a WaypointController with
// LookaheadBlendStart defaulted to 1.0 (a hard switch), matching
// WaypointController.__init__'s default.
func NewWaypointController(
	maxSteeringAngle, wheelbaseM, lookaheadShort, lookaheadLong, lookaheadTransition,
	maxSteeringRate, waypointReachedDistanceM, cornerTurnThresholdRad,
	yawGainCompensation float64,
) *WaypointController {
	const defaultLookaheadBlendStart = 1.0
	return &WaypointController{
		MaxSteeringAngle:         maxSteeringAngle,
		WheelbaseM:               wheelbaseM,
		LookaheadShort:           lookaheadShort,
		LookaheadLong:            lookaheadLong,
		LookaheadTransition:      lookaheadTransition,
		MaxSteeringRate:          maxSteeringRate,
		WaypointReachedDistanceM: waypointReachedDistanceM,
		CornerTurnThresholdRad:   cornerTurnThresholdRad,
		YawGainCompensation:      yawGainCompensation,
		LookaheadBlendStart:      defaultLookaheadBlendStart,
	}
}

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
// that is behind the chassis in its current local frame.
func (w *WaypointController) SelectTargetPoint(
	currentPos trackmodel.Waypoint,
	currentYaw float64,
	waypointsPath []trackmodel.Waypoint,
	waypointIndex int,
	lookaheadDistance float64,
) trackmodel.Waypoint {
	n := len(waypointsPath)
	cosYaw, sinYaw := math.Cos(currentYaw), math.Sin(currentYaw)

	var nearestAhead *trackmodel.Waypoint
	nearestAheadDist := math.Inf(1)
	nearestAny := waypointsPath[((waypointIndex%n)+n)%n]
	nearestAnyDist := math.Inf(1)

	for offset := range n {
		wp := waypointsPath[(waypointIndex+offset)%n]
		dx, dy := wp.X-currentPos.X, wp.Y-currentPos.Y
		dist := math.Hypot(dx, dy)
		if dist < nearestAnyDist {
			nearestAnyDist = dist
			nearestAny = wp
		}
		xLocal := dx*cosYaw + dy*sinYaw
		if xLocal <= 0 {
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
	currentPos trackmodel.Waypoint, currentYaw float64, targetWaypoint trackmodel.Waypoint,
	crosstrackError, dt float64,
) (steeringNormalized, lookaheadDistanceM, angleErrorRad float64) {
	lookahead := w.SelectLookahead(crosstrackError, 0.0, false)

	pose := trackmodel.Pose{X: currentPos.X, Y: currentPos.Y, Yaw: currentYaw}
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
