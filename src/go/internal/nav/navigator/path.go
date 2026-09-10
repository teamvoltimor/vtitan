package navigator

import (
	"math"
	"slices"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/parking"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// ReplacePath swaps in a new planned path mid-run, resuming at the nearest
// point, matching replace_path.
//
// The waypoint index cannot carry over: the new path has its own indexing
// and the old index would point somewhere arbitrary on it. Re-seeking to
// the nearest waypoint keeps progress instead of restarting the lap, and
// matters because the paths differ by centimeters, not corridors.
//
// Nearest-by-position alone can go wrong near a corner, where several
// waypoints sit at almost the same distance while pointing in very
// different directions, and the robot's heading at that instant does not
// always match the path's local direction there yet. Picking purely by
// position can then hand WaypointController a point past the turn,
// demanding a correction far larger than finishing the corner needs --
// measured on real hardware as a ~193 deg swing where ~90 deg would do.
// Passing a non-nil robotYaw re-ranks the near-tied-by-distance candidates
// (ReplanHeadingTieMarginM) by heading agreement instead. Pass nil where
// the robot has been tracking a path very similar to the new one, where
// nearest-by-position alone is already safe.
func (n *Navigator) ReplacePath(
	path []trackmodel.Waypoint,
	robotXY trackmodel.Waypoint,
	robotYaw *float64,
) {
	if len(path) == 0 {
		return
	}
	previousIndex := n.waypointIndex
	n.waypoints = slices.Clone(path)
	// A replanned path is a new centerline, so the lanes have to be laid
	// over it again -- and the fingerprint cleared, or the unchanged sign
	// layout would read as "already applied" and leave the new path bare.
	n.laneBaseWaypoints = slices.Clone(path)
	n.laneFingerprint, n.laneFingerprintSet = nil, false
	// A corner held open by the latch was previewed on the OLD centerline
	// and need not exist on this one, so holding it would keep the short
	// lookahead armed against a turn the robot is no longer going to make.
	n.cornerLatch.Reset()
	n.applyPathWallBudget()

	distances := make([]float64, len(path))
	nearestIndex := 0
	for i, wp := range path {
		distances[i] = wp.DistanceTo(robotXY)
		if distances[i] < distances[nearestIndex] {
			nearestIndex = i
		}
	}

	if robotYaw != nil {
		margin := distances[nearestIndex] + n.cfg.ReplanHeadingTieMarginM
		bestError := math.Inf(1)
		best := nearestIndex
		for i, d := range distances {
			if d > margin {
				continue
			}
			headingError := math.Abs(navutil.WrapAngle(outgoingBearing(path, i) - *robotYaw))
			if headingError < bestError {
				best, bestError = i, headingError
			}
		}
		nearestIndex = best
	}

	n.waypointIndex = nearestIndex

	// A large forward jump is never earned progress -- the robot cannot
	// skip most of a lap between two ticks. It means the re-seek landed on
	// the far side of the start/finish seam: the path was rebuilt running
	// the other way, leaving the robot just BEHIND the new waypoint 0,
	// which on a closed loop is also the tail of the list. The tail it is
	// about to drive belongs to a lap it never ran, so the wrap at the end
	// of it is the entry into lap 1, not the completion of it.
	if n.waypointIndex-previousIndex > len(path)/2 {
		n.suppressNextWrap = true
	}
}

// ReplaceSignRouter swaps in a sign router built for a new race, matching
// replace_sign_router.
//
// The state machine can cycle FINISHED -> BOOT_CHECK -> READY -> RACING
// purely from the button, so a process built once as Open (nil router) must
// be able to pick up a switch to Obstacles, and vice versa, without a
// restart. The previous router's own committed state does not carry over --
// the caller builds a fresh one from the current section/direction/tuning.
func (n *Navigator) ReplaceSignRouter(router *signrouter.SignRouter) {
	n.signRouter = router
	// Whatever lanes the previous router's layout produced belong to that
	// race. Drop back to the planned centerline and let the next tick
	// re-lane from the new router, if there is one.
	n.waypoints = slices.Clone(n.laneBaseWaypoints)
	n.laneFingerprint, n.laneFingerprintSet = nil, false
}

// ReplaceParkController swaps in a parking maneuver built for a new race,
// matching replace_park_controller. Exists for the same reason as
// ReplaceSignRouter: the state machine can cycle FINISHED -> BOOT_CHECK ->
// READY -> RACING purely from the button, so a process built once for one
// scenario's parking lot (or none) must be able to pick up a different one
// without a restart. nil is valid -- the Open Challenge, or any scenario
// with no parking lot.
func (n *Navigator) ReplaceParkController(pc *parking.ParkController) {
	n.parkController = pc
	n.parkingEngaged = false
}

// SetTravelDirection adopts the (re-)inferred travel direction, matching
// set_travel_direction. Consumed by the collision-avoidance escape maneuver
// as the fallback side when a LIDAR-only clearance comparison cannot decide
// one.
func (n *Navigator) SetTravelDirection(direction trackmodel.Direction) {
	n.direction = &direction
}

// Reset clears per-race state so a new race starts as if this were the
// first, matching reset.
//
// Needed because the state machine can cycle FINISHED -> BOOT_CHECK ->
// READY -> RACING purely from the physical button, with no process restart
// -- so nothing else re-creates this object between races. Without this,
// LapsCompleted alone would stay at its previous value and the very first
// tick of the new race would immediately read as already finished.
func (n *Navigator) Reset() {
	n.waypointIndex = 0
	n.lapsCompleted = 0
	n.suppressNextWrap = false
	n.cornerLatch.Reset()
	n.parkingEngaged = false
	n.activeManeuver = nil
	n.maneuverFramesLeft = 0
	n.escapeCount = 0
	n.escapeSteerSign = 1.0
	n.escapeSequenceStartXY = nil
	n.stuckDetector.Reset()
	n.waypointController.Reset()
	if n.signRouter != nil {
		n.signRouter.ResetForNewLap()
	}
}

// handleFinish handles the post-final-lap phase, matching _handle_finish.
// Returns true if a command was issued (caller should stop this tick);
// false if the robot should keep navigating toward the parking corridor.
func (n *Navigator) handleFinish(robotX, robotY, robotYaw float64) bool {
	pc := n.parkController
	if pc == nil {
		// Open Challenge: no parking maneuver -- hold position.
		n.holdFinished(robotX, robotY, robotYaw)
		return true
	}

	if !n.parkingEngaged && n.shouldEngageParking(robotX, robotY) {
		// currentCorridor is set every tick before this branch is reachable
		// (Step's very first assignment), so it is never nil here.
		n.logger.Info("parking engaged", "corridor", *n.currentCorridor)
		n.parkingEngaged = true
	}
	if !n.parkingEngaged {
		return false // Keep navigating until at the staging point.
	}

	if pc.IsDone() {
		n.holdFinished(robotX, robotY, robotYaw)
		return true
	}

	cmd := pc.Update(trackmodel.Pose{X: robotX, Y: robotY, Yaw: robotYaw})
	linear := cmd.LinearMPS
	if scan, ok := n.gateway.GetLidarScan(); ok {
		// One call for both parking stop-check clearances (narrow-forward
		// min + full 360deg sweep min) over the same scan. Not colliding
		// takes priority over completing the maneuver -- see
		// ParkingClearances' own doc comment for the side-margin padding
		// rationale.
		gate := n.collisionController.ParkingClearances(scan.RangesM, scan.AnglesRad)
		sideMargin := n.cfg.ContactDistM + n.cfg.ChassisWidthM/2
		if gate.ForwardM < n.cfg.ContactDistM || gate.SweepM < sideMargin {
			linear = 0.0
		}
	}

	n.gateway.PublishDrive(controllers.DriveCommand{SpeedMPS: linear, SteeringNorm: cmd.SteeringNorm})
	debug := n.baseDebug(robotX, robotY, robotYaw)
	debug.Phase = PhaseParking
	parkPhase := cmd.Phase
	debug.ParkPhase = &parkPhase
	debug.CommandedSpeedMPS = new(linear)
	debug.CommandedSteerNorm = new(cmd.SteeringNorm)
	n.debug = debug
	return true
}

// holdFinished publishes a zero drive command and the FINISHED_HOLD
// snapshot, matching the two identical branches of _handle_finish (no
// ParkController, and a done one) that both do exactly this.
func (n *Navigator) holdFinished(robotX, robotY, robotYaw float64) {
	n.gateway.PublishDrive(controllers.DriveCommand{})
	debug := n.baseDebug(robotX, robotY, robotYaw)
	debug.Phase = PhaseFinishedHold
	debug.CommandedSpeedMPS = new(0.0)
	debug.CommandedSteerNorm = new(0.0)
	n.debug = debug
}

// shouldEngageParking reports whether the parking handoff should engage:
// only once in the parking corridor and near the staging point, matching
// _should_engage_parking.
func (n *Navigator) shouldEngageParking(robotX, robotY float64) bool {
	pc := n.parkController
	if pc == nil {
		return false
	}
	if n.currentCorridor != nil && *n.currentCorridor != pc.Section() {
		return false
	}
	staging := pc.Staging()
	return math.Hypot(staging.X-robotX, staging.Y-robotY) < n.cfg.ParkEngageDistM
}

// handleWaypointWrap detects the index running off the end of the lap and
// counts the lap, matching step()'s waypoint-wrap branch. Returns true when
// the tick is finished.
//
// This is the ONLY lap-counting mechanism in this port. Python prefers a
// LapDetector's geometric confirmation and falls back to counting wraps
// directly; LapDetector has no Go equivalent (see doc.go), so the fallback
// branch is always the one taken.
func (n *Navigator) handleWaypointWrap(robotX, robotY, robotYaw float64) bool {
	if n.waypointIndex < len(n.waypoints) {
		return false
	}
	n.waypointIndex = 0
	n.stuckDetector.Reset()
	if n.suppressNextWrap {
		// Seeded past the seam by ReplacePath, not driven -- see there.
		n.suppressNextWrap = false
		return false
	}

	n.lapsCompleted++
	n.logger.Info("lap complete (waypoint-only fallback)", "laps_completed", n.lapsCompleted)
	if n.signRouter != nil {
		n.signRouter.ResetForNewLap()
	}
	debug := n.baseDebug(robotX, robotY, robotYaw)
	debug.Phase = PhaseWaypointWrapFallback
	n.debug = debug
	return true
}

// advancePastPassedWaypoints walks the index past any waypoint the robot
// has already gone by and returns the waypoint now being driven toward,
// matching step()'s advance loop.
//
// Not just one the robot happens to pass within the threshold of: a robot
// that starts somewhere other than exactly on the planned centerline can
// curve past a waypoint without ever entering that radius; left
// un-advanced, the lookahead search keeps re-targeting that same,
// increasingly stale point long after the robot has passed it -- and once
// far enough away, that stale point can itself satisfy the search's
// lookahead-distance test and be selected as the steering target even
// though it is now behind the robot.
//
// The comparison wraps around the seam, so the LAST waypoint gets the same
// pass-by rescue as every other one. Stopping at index+1 < len left
// entering the reached-distance circle as the only way past the final
// point, and a robot running wider than that radius never gets past it,
// never wraps, and so never completes a lap: measured on the 2026-08-06
// counterclockwise round as a robot circling the mat for seven minutes with
// the lap count stuck at zero.
//
// The BEHIND test is an Obstacles-only extension (gated on the router's
// presence and StaleTargetRescue): a robot cutting a corner sharply enough
// can leave both the current and next waypoint reading farther away every
// tick, even though local-frame ahead/behind already shows the chassis has
// swept past them.
func (n *Navigator) advancePastPassedWaypoints(
	robotX, robotY, robotYaw float64,
) trackmodel.Waypoint {
	here := trackmodel.Waypoint{X: robotX, Y: robotY}
	rawWP := n.waypoints[n.waypointIndex]

	rescueBehind := n.signRouter != nil && n.cfg.StaleTargetRescue
	cosYaw, sinYaw := 0.0, 0.0
	if rescueBehind {
		cosYaw, sinYaw = math.Cos(robotYaw), math.Sin(robotYaw)
	}

	count := len(n.waypoints)
	for range count {
		nextIndex := n.waypointIndex + 1
		nextWP := n.waypoints[nextIndex%count]
		nextCloser := nextWP.DistanceTo(here) < rawWP.DistanceTo(here)
		rawBehind := rescueBehind && (rawWP.X-robotX)*cosYaw+(rawWP.Y-robotY)*sinYaw <= 0
		if !nextCloser && !rawBehind {
			break
		}
		n.waypointIndex = nextIndex
		if nextIndex >= count {
			// Seam crossed. Leave rawWP on the final waypoint and let the
			// wrap branch count the lap next tick -- walking on into the
			// new lap here would skip waypoints the wrap is about to
			// rewind to.
			break
		}
		rawWP = nextWP
	}
	return rawWP
}
