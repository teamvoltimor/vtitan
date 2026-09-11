package navigator

import (
	"math"
	"slices"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// signAhead reports whether a routed sign is within activation distance and
// ahead of the chassis, the third preview signal alongside crosstrack and
// turn-ahead.
//
// Crosstrack is measured against the raw path, so it never rises during a
// sign pass (the deformation biases the SEARCH's output, not the path the
// search is judged against). RoutedSignPositions is read-only, so this is
// safe to query before DeformWaypoint runs later this tick.
//
// Restricted to signs actually AHEAD along the chassis heading:
// RoutedSignPositions has no direction filter, so without this a
// not-yet-passed sign still alongside or just behind the chassis forces the
// short lookahead just as readily as a genuinely upcoming one.
func (n *Navigator) signAhead(robotX, robotY, robotYaw float64) bool {
	if !n.cfg.SignAwareLookahead || n.signRouter == nil {
		return false
	}
	cosYaw, sinYaw := math.Cos(robotYaw), math.Sin(robotYaw)
	for _, wp := range n.signRouter.RoutedSignPositions() {
		dx, dy := wp.X-robotX, wp.Y-robotY
		if math.Hypot(dx, dy) < n.cfg.ActivationDistM && dx*cosYaw+dy*sinYaw > 0 {
			return true
		}
	}
	return false
}

// applySignRouting runs the sign router over the point steering will
// actually chase, matching step()'s deform block.
//
// Deforming a raw-path CANDIDATE before the lookahead search picked from it
// meant the search, not the sign, decided whether the nudge ever reached
// steering (it almost never did: waypoints are spaced well under the
// lookahead, so the search kept skipping past a single deformed candidate).
// Deforming the search's own output guarantees the bias is exactly what
// gets steered toward.
//
// The router is CALLED even when SignLaneSuppressDeform discards its
// output: it owns engage/pass bookkeeping and RoutedSignPositions (which
// the escape mask reads), none of which the lane transform replaces.
func (n *Navigator) applySignRouting(
	steerTarget, here trackmodel.Waypoint, robotYaw float64,
) (target trackmodel.Waypoint, deformMagnitude *float64, activeSigns *int) {
	if n.signRouter == nil || n.currentCorridor == nil {
		return steerTarget, nil, nil
	}
	observations := n.visionDetections()

	// DISCOVERY FIRST, matching deform_waypoint's own ordering: "in blind mode
	// this frame may be what reveals the sign about to be routed around, so it
	// has to land before candidate selection rather than after it."
	//
	// Until 2026-09-06 this ran ONLY inside blindCreep, so Go stopped looking
	// at the camera the moment the travel direction settled -- a few seconds
	// into the round. A genuinely blind run therefore confirmed 0-1 of its 4-6
	// signs and drove into the rest. Python ingests every tick, for the whole
	// round, from inside the router.
	if n.discovery != nil {
		n.discovery.Observe(observations, here)
		n.discovery.Publish()
	}

	deformed := n.signRouter.DeformWaypoint(
		steerTarget,
		here,
		robotYaw,
		*n.currentCorridor,
		observations,
	)
	magnitude := math.Hypot(deformed.X-steerTarget.X, deformed.Y-steerTarget.Y)

	suppress := n.cfg.SignLanePlanner && n.cfg.SignLaneSuppressDeform
	if !suppress {
		steerTarget = deformed
	}
	return steerTarget, new(magnitude), new(n.signRouter.ActiveSignCount())
}

// visionDetections returns this tick's traffic-sign observations, or an
// empty slice when there is no vision port or it has nothing to report --
// which is what Python's gateway returns in the same situation.
func (n *Navigator) visionDetections() []signrouter.TrafficSignObservation {
	if n.vision == nil {
		return nil
	}
	observations, ok := n.vision.GetVisionDetections()
	if !ok {
		return nil
	}
	return observations
}

// refreshSignLanes rebuilds the planned path onto its pass-side lanes when
// the sign layout changes, matching _refresh_sign_lanes.
//
// No-op unless a SignRouter exists and SignLanePlanner is set, so the Open
// Challenge's path is never rewritten -- it has no router at all, and the
// early return here is what makes that structural rather than a matter of
// the flag's value.
//
// Lanes are always recomputed from laneBaseWaypoints (the path as planned)
// rather than from waypoints: re-laning an already-laned path would stack
// one offset on the next every time the layout was refreshed.
//
// waypointIndex is deliberately NOT re-seeked the way ReplacePath does.
// This transform is 1:1 and order-preserving -- waypoint i of the lane path
// is waypoint i of the base path moved sideways -- so the index still
// denotes the same point on the same lap.
func (n *Navigator) refreshSignLanes() {
	if n.signRouter == nil || !n.cfg.SignLanePlanner {
		return
	}
	fingerprint := laneFingerprintOf(n.signRouter)
	if n.laneFingerprintSet && slices.Equal(fingerprint, n.laneFingerprint) {
		return
	}
	n.laneFingerprint, n.laneFingerprintSet = fingerprint, true

	previous := n.waypoints
	// The ROUTER's direction, not the navigator's: the lane must be built
	// on the same one the pass-side decision was made under.
	direction := n.signRouter.Direction()
	n.waypoints = signrouter.ApplySignLanes(
		n.laneBaseWaypoints,
		n.signRouter.LaneSpecs(),
		signrouter.SignLaneParams{
			LateralOffsetM:    n.cfg.LaneLateralOffsetM(n.signRouterCfg.ChassisHalfDiagonalM),
			RampM:             n.cfg.SignLaneRampM,
			HoldM:             n.cfg.SignLaneHoldM,
			SplitOverlap:      n.cfg.SignLaneSplitOverlap,
			SkipUnsatisfiable: n.cfg.SignLaneSkipUnsatisfiable,
			CornerEntryM:      n.cfg.SignLaneCornerEntryM,
		},
		&direction,
		n.signRouterCfg,
	)
	n.holdCommittedPath(previous, n.cfg.SignLaneCommitAheadM)
	n.applyPathWallBudget()
	n.logger.Info("sign lanes replanned", "signs", len(fingerprint))
}

// laneFingerprintOf derives the sign layout a lane path would be built for,
// standing in for SignRouter.lane_fingerprint (see laneFingerprintEntry).
func laneFingerprintOf(router *signrouter.SignRouter) []laneFingerprintEntry {
	specs := router.LaneSpecs()
	fingerprint := make([]laneFingerprintEntry, 0, len(specs))
	for _, spec := range specs {
		fingerprint = append(fingerprint, laneFingerprintEntry{
			X: spec.Spec.X, Y: spec.Spec.Y, Corridor: spec.Corridor,
		})
	}
	return fingerprint
}

// holdCommittedPath keeps a lane rebuild from moving the path the chassis
// is already on, matching _hold_committed_path.
//
// A lane ramps onto its offset over the approach, which assumes the rebuild
// happens before the robot reaches that stretch. Sighted runs satisfy that
// trivially -- the layout is known at t=0 and the path is built once. A
// discovering run does not, and is instantly off a path it has no runway to
// rejoin. So the near field is pinned to what it already was; new
// information still bends the path, just ahead of the robot rather than
// underneath it.
//
// No-op at commitAheadM 0.0 (the shipped default), and no-op for a sighted
// run either way, since nothing rebuilds after the first tick there.
func (n *Navigator) holdCommittedPath(previous []trackmodel.Waypoint, commitAheadM float64) {
	if commitAheadM <= 0.0 || len(previous) == 0 || len(previous) != len(n.waypoints) {
		return
	}
	pose, ok := n.gateway.GetCurrentPose()
	if !ok {
		return
	}
	cosYaw, sinYaw := math.Cos(pose.Yaw), math.Sin(pose.Yaw)
	held := slices.Clone(n.waypoints)
	for i, old := range previous {
		next := n.waypoints[i]
		if old == next {
			continue
		}
		// Along-track distance in the chassis frame: negative is behind.
		if (next.X-pose.X)*cosYaw+(next.Y-pose.Y)*sinYaw < commitAheadM {
			held[i] = old
		}
	}
	n.waypoints = held
}
