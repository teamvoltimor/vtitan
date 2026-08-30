package navigator

import (
	"math"
	"slices"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// This file ports core_navigator/escape_recovery.py's EscapeRecovery mixin:
// the retrace-reverse, K-turn escalation, pivot-out-of-wedge and
// masked-scan escape arbitration. Python extracts them into a mixin that is
// flattened onto CoreNavigator at runtime; Go has no need for that
// indirection, so they are plain unexported methods on *Navigator, kept in
// their own file for the same readability reason the mixin exists.

// retraceSteer is the steering that reverses the chassis back along ground
// it just occupied, matching _retrace_steer. ok=false when the trail is too
// short to aim at, leaving the caller on its ordinary reverse.
//
// A generic reverse escape backs along an ARC into space the robot has
// never been and, on this chassis, largely cannot see: the rear sector is
// masked by mount occlusion except for a narrow slot straight back. Two
// consequences, both arguing for retracing instead:
//
//   - That slot may not exist on the next chassis at all. If it goes,
//     ComputeRearClearance reports the same no-data sentinel it reports for
//     open road, and reversingIntoUnseenWall refuses outright -- but a
//     refused reverse is a robot that is not escaping.
//   - The arc is what produces the wall strikes. Measured blind with the
//     escape mask off, sign collisions fall 57 -> 41 but wall collisions
//     rise 0 -> 13, in a corridor only 1.0 m wide.
//
// Retracing needs no rear sensor by construction: the chassis was
// physically standing on this ground seconds ago, so it is free unless
// something moved into it, and nothing on this track does. It also cannot
// swing into a wall, because it follows a path already driven.
//
// Reverse pure pursuit: curvature is the NEGATIVE of the forward case,
// since the vehicle rotates the other way for a given steer angle when
// traveling backwards.
func (n *Navigator) retraceSteer(robotX, robotY, robotYaw float64) (steer float64, ok bool) {
	target, found := n.trailPointBehind(robotX, robotY)
	if !found {
		return 0, false
	}
	cosYaw, sinYaw := math.Cos(robotYaw), math.Sin(robotYaw)
	dx, dy := target.X-robotX, target.Y-robotY
	along := dx*cosYaw + dy*sinYaw
	lateral := -dx*sinYaw + dy*cosYaw
	distance := target.ToWaypoint().DistanceTo(trackmodel.Waypoint{X: robotX, Y: robotY})
	if distance < n.cfg.PoseTrailMinStepM || along > 0.0 {
		// Target is not actually behind the chassis -- nothing to retrace.
		return 0, false
	}
	return n.cfg.RetraceSteerGainNorm(-lateral / distance), true
}

// trailPointBehind is the breadcrumb roughly RetraceDistM back along the
// trail, matching _trail_point_behind.
func (n *Navigator) trailPointBehind(robotX, robotY float64) (point trackmodel.Pose, ok bool) {
	travelled := 0.0
	previous := trackmodel.Waypoint{X: robotX, Y: robotY}
	for _, candidate := range slices.Backward(n.poseTrail) {
		travelled += candidate.ToWaypoint().DistanceTo(previous)
		previous = candidate.ToWaypoint()
		if travelled >= n.cfg.RetraceDistM {
			return candidate, true
		}
	}
	return trackmodel.Pose{}, false
}

// reversingIntoUnseenWall reports whether executing maneuver would back
// into a wall behind the robot, matching _reversing_into_unseen_wall.
//
// Skipped while retracing: that maneuver reverses along ground the chassis
// just occupied, so it is known free without consulting a rear sector this
// hardware barely covers.
//
// A rear sector with no valid rays counts as blocked, not clear. Reading
// the clearance alone fails open there, because it reports the same no-data
// sentinel for "nothing behind me" and "I cannot see behind me" -- the gate
// would wave the reverse through exactly when it is blindest. Refusing
// costs little: the caller falls through to a capped forward creep, with
// the stuck detector as the backstop.
func (n *Navigator) reversingIntoUnseenWall(maneuver controllers.EscapeManeuver, scan controllers.LidarScan) bool {
	if maneuver.Speed >= 0 || n.retracing {
		return false
	}
	rear := n.collisionController.RearSector(scan.RangesM, scan.AnglesRad)
	if !rear.Measured() {
		// No rear vision on this mount, but the pose trail records ground
		// the chassis physically occupied -- the same argument the retrace
		// exemption above already accepts, reached by escapes that are not
		// retraces. Evidence rather than a sensor: it cannot know what
		// moved in since, so it has to cover the WHOLE maneuver with the
		// contact distance to spare before it counts, and an empty trail
		// still refuses. Without this the gate is unreachable on a chassis
		// with no rear slot, and a scenario needing one escape-reverse hits
		// the wall instead (measured on go_open #85, 2026-08-22).
		reverseDistance := math.Abs(maneuver.Speed) * float64(maneuver.DurationFrames) / n.cfg.ControlHz
		if n.trailConfirmsReverse(reverseDistance) {
			return false
		}
		n.logger.Warn("reverse escape refused: rear sector measured nothing")
		return true
	}
	// As a gap from the REAR bumper. Compared raw until 2026-08-22, which
	// made this gate unreachable: the sensor is at the front, so an
	// obstacle touching the rear bumper reports ~0.272 m against a 0.10 m
	// threshold and the reverse was authorized right up to impact.
	return controllers.BumperGapBehind(rear.MinRangeM, n.cfg.LidarToRearBumperM) < n.cfg.ContactDistM
}

// trailConfirmsReverse reports whether the pose trail vouches for a reverse
// of reverseDistance, matching _trail_confirms_reverse.
//
// The trail records ground the chassis physically occupied, so it is the
// one statement about the space behind that needs no rear sensor. It is
// evidence, not a reading: it cannot know what moved in since, so the
// ground must cover the WHOLE maneuver with the contact distance to spare.
// An empty trail refuses -- which is exactly the told-direction wedge
// behavior wanted on a chassis with no rear slot.
func (n *Navigator) trailConfirmsReverse(reverseDistance float64) bool {
	if len(n.poseTrail) == 0 {
		return false
	}
	newest := n.poseTrail[len(n.poseTrail)-1]
	covered, ok := trailClearanceBehind(n.poseTrail, newest.X, newest.Y, newest.Yaw, n.cfg.ChassisWidthM/2)
	return ok && covered >= reverseDistance+n.cfg.ContactDistM
}

// trailClearanceBehind is how far the chassis may reverse over ground it
// has already occupied, matching utils.trail_clearance_behind. Ported here
// rather than into navutil because the navigator is its only consumer and
// navutil's existing clearance helpers are all scan-derived, where this
// deliberately is not.
//
// This is not a sensor reading and does not pretend to be one: it is a
// record of where the chassis physically was, which is the one statement
// about the space behind it that needs no rear vision at all.
//
// Walks back from the newest breadcrumb and stops at the first one that
// leaves a corridor of the chassis's own width -- the trail is only a
// promise about ground the footprint actually covered, so a trail that
// curves away is no longer describing the path a reverse would take.
// Points still ahead of the chassis are skipped rather than terminating the
// walk: the newest breadcrumbs sit within centimeters of the current pose
// and their sign is noise.
//
// Deliberately conservative in two ways. The trail records the chassis
// CENTER, so ground occupied by the rear half of the footprint is not
// counted. And it says nothing about anything that MOVED into that space
// since.
//
// ok=false means NO EVIDENCE, which is not the same as no room.
func trailClearanceBehind(
	trail []trackmodel.Pose, robotX, robotY, robotYaw, halfWidthM float64,
) (reachableM float64, ok bool) {
	cosYaw, sinYaw := math.Cos(robotYaw), math.Sin(robotYaw)
	reachable := 0.0
	for _, t := range slices.Backward(trail) {
		deltaX, deltaY := t.X-robotX, t.Y-robotY
		along := deltaX*cosYaw + deltaY*sinYaw
		if along > 0.0 {
			continue
		}
		if math.Abs(-deltaX*sinYaw+deltaY*cosYaw) > halfWidthM {
			break
		}
		reachable = math.Max(reachable, -along)
	}
	// Python returns `reachable or None`, so a zero-length result is "no
	// evidence" rather than "zero room" -- preserved here as ok=false.
	if reachable == 0.0 {
		return 0, false
	}
	return reachable, true
}

// beginManeuver latches an escape maneuver so it executes for its full
// duration, matching _begin_maneuver.
func (n *Navigator) beginManeuver(maneuver controllers.EscapeManeuver) {
	n.activeManeuver = &maneuver
	n.maneuverFramesLeft = max(1, maneuver.DurationFrames)
	// An escape maneuver drives steering directly, bypassing pure pursuit.
	// Clear the rate-limit memory so pure pursuit does not rate-limit its
	// first post-maneuver command against a stale pre-maneuver angle.
	n.waypointController.Reset()
}

// driveActiveManeuver publishes the active escape command and counts down
// its latched duration, matching _drive_active_maneuver.
func (n *Navigator) driveActiveManeuver(robotX, robotY, robotYaw float64, phase Phase) {
	if n.activeManeuver == nil {
		return
	}
	maneuver := *n.activeManeuver

	n.maneuverFramesLeft--
	if n.maneuverFramesLeft <= 0 {
		n.activeManeuver = nil
		n.retracing = false
	}

	// A retrace is re-aimed every tick, unlike a latched arc: the whole
	// point is to follow a path, and a single steering value fixed at
	// trigger time would describe an arc again after the first few
	// centimeters. Falls back to the latched steering the moment the trail
	// runs out, so this can only ever be as bad as the ordinary reverse.
	if n.retracing {
		if retrace, ok := n.retraceSteer(robotX, robotY, robotYaw); ok {
			maneuver.Steering = retrace
		}
	}

	n.gateway.PublishDrive(controllers.DriveCommand{
		SpeedMPS: maneuver.Speed, SteeringNorm: maneuver.Steering,
	})
	debug := n.baseDebug(robotX, robotY, robotYaw)
	debug.Phase = phase
	debug.ActiveManeuverType = ptr(maneuver.Type)
	debug.ManeuverSteering = ptr(maneuver.Steering)
	debug.ManeuverSpeedMPS = ptr(maneuver.Speed)
	debug.ManeuverFramesLeft = ptr(n.maneuverFramesLeft)
	debug.EscapeCount = ptr(n.escapeCount)
	debug.CommandedSpeedMPS = ptr(maneuver.Speed)
	debug.CommandedSteerNorm = ptr(maneuver.Steering)
	n.debug = debug
}

// escapeSteerSignForAttempt is which side this escape attempt swings
// toward, matching _escape_steer_sign_for_attempt.
//
// Derived from escapeCount rather than flipped in place, so a side is held
// for EscapeSideCommitAttempts consecutive attempts before the other is
// tried. Flipping on every attempt means consecutive attempts rotate the
// chassis in opposite directions and undo each other: measured on real
// hardware 2026-08-05 as four escalating escapes over 40 s that rocked the
// yaw between -0.4 and -0.8 rad and translated the robot exactly nowhere.
//
// firstAttempt is the escapeCount at which this caller's sequence begins,
// so its blocks line up with it -- anchoring every caller at 1 leaves
// whichever attempt a caller actually starts on stranded mid-block, and a
// block of one is the alternating behavior this exists to stop. startSign
// is the side for the sequence's first block, defaulting (nil) to the base;
// escalation passes the opposite, since switching sides is the point of
// escalating.
func (n *Navigator) escapeSteerSignForAttempt(firstAttempt int, startSign *float64) float64 {
	base := n.escapeSteerSign
	if startSign != nil {
		base = *startSign
	}
	commit := max(1, n.cfg.EscapeSideCommitAttempts)
	block := max(0, n.escapeCount-firstAttempt) / commit
	if block%2 == 0 {
		return base
	}
	return -base
}

// pivotSteerSign is the forward-travel steer sign that swings the nose
// toward the open side, matching _pivot_steer_sign.
//
// Used by the rear-free stop-and-steer pivot, where the chassis is wedged
// front-and-back with no rear sensor to authorize a reverse. Forward travel
// swings the nose RIGHT for a positive command, the opposite of reverse, so
// the sign must point the nose toward the WIDER side clearance rather than
// mirroring the reverse K-turn rule.
//
// A side with no valid return is the clearest possible "open" reading -- a
// wall-pinned chassis reads a close valid return on the jammed side and
// nothing on the free side -- so a missing side is treated as maximally
// open, never as a tie. Falls back to the committed escape side when LIDAR
// says nothing at all, so a pivot still happens rather than stalling.
func (n *Navigator) pivotSteerSign(scan controllers.LidarScan, haveScan bool) float64 {
	if !haveScan || len(scan.RangesM) == 0 {
		return n.escapeSteerSignForAttempt(1, nil)
	}
	const sideHalfFovRad = math.Pi / 4
	left := n.collisionController.ComputeMinClearance(scan.RangesM, scan.AnglesRad, math.Pi/2, sideHalfFovRad)
	right := n.collisionController.ComputeMinClearance(scan.RangesM, scan.AnglesRad, -math.Pi/2, sideHalfFovRad)
	if left >= n.cfg.NoDataRangeM && right >= n.cfg.NoDataRangeM {
		return n.escapeSteerSignForAttempt(1, nil)
	}
	// More open side wins; forward positive steer = nose right.
	switch {
	case left > right:
		return -1.0
	case right > left:
		return 1.0
	default:
		return n.escapeSteerSignForAttempt(1, nil)
	}
}

// maybeEscalate escalates a repeated escape instead of repeating an
// identical pulse, matching _maybe_escalate.
//
// After a few consecutive escapes that clearly are not working, reverse for
// longer and swing toward the opposite side, so the robot stops slamming
// the same failing maneuver into the same wall.
func (n *Navigator) maybeEscalate(maneuver controllers.EscapeManeuver) controllers.EscapeManeuver {
	if n.escapeCount <= n.cfg.EscalateAfterAttempts {
		return maneuver
	}
	oppositeBase := -n.escapeSteerSign
	side := n.escapeSteerSignForAttempt(n.cfg.EscalateAfterAttempts+1, &oppositeBase)

	escalated := maneuver
	escalated.Steering = 0.0
	if maneuver.Steering != 0 {
		escalated.Steering = math.Abs(maneuver.Steering) * side
	}
	escalated.DurationFrames = min(maneuver.DurationFrames*2, n.cfg.MaxEscapeFrames)
	return escalated
}

// stuckEscapeFrames is the latched duration of this stuck-escape attempt,
// escalating with repeated attempts up to the hard cap.
func (n *Navigator) stuckEscapeFrames() int {
	return min(
		n.cfg.KTurnMinFrames+n.cfg.StuckEscalationFramesPerAttempt*(n.escapeCount-1),
		n.cfg.MaxEscapeFrames,
	)
}

// beginStuckEscape is the common tail of all three stuck-escape branches:
// count the attempt, latch the maneuver, re-arm the detector, publish, and
// annotate the snapshot with the diagnostics that named the robot stuck.
func (n *Navigator) beginStuckEscape(
	robotX, robotY, robotYaw float64,
	maneuverType controllers.ManeuverType,
	steering, speed float64,
	diag controllers.StuckDiagnostics,
	rearClear, forwardClear float64,
	reportForwardClearance bool,
) {
	if n.escapeCount == 0 {
		n.escapeSequenceStartXY = &trackmodel.Waypoint{X: robotX, Y: robotY}
	}
	n.escapeCount++
	n.beginManeuver(controllers.EscapeManeuver{
		Type:           maneuverType,
		Steering:       steering,
		Speed:          speed,
		DurationFrames: n.stuckEscapeFrames(),
	})
	n.stuckDetector.Reset()
	n.driveActiveManeuver(robotX, robotY, robotYaw, PhaseStuckEscapeManeuver)
	n.debug.IsStuck = ptr(diag.IsStuck)
	n.debug.StuckCount = ptr(diag.StuckCount)
	n.debug.RecentMovementM = ptr(diag.RecentMovementM)
	n.debug.RearClearanceM = ptr(rearClear)
	if reportForwardClearance {
		n.debug.ForwardClearanceM = ptr(forwardClear)
	}
}

// handleStuckEscape reverses out of a stuck state, but never back into an
// unseen wall, matching _handle_stuck_escape.
//
// The reverse is latched for several frames (escalating with repeated
// attempts) and switches steering side only after committing to one for
// several attempts, so a wall-pinned robot actually backs away instead of
// twitching one centimeter every few seconds forever.
//
// When reverse itself is blocked (wedged both front and rear), this used to
// hold and reset the detector, over and over, forever: confirmed on real
// hardware 2026-08-04 as a robot frozen at the same position for 27 s.
// Holding is only the safe choice when forward is ALSO blocked; when it is
// not, a forward creep at full steering lock gives the robot a real chance
// to walk itself clear using more decisive steering than normal drive's
// pure-pursuit curvature was willing to command for this geometry.
func (n *Navigator) handleStuckEscape(robotX, robotY, robotYaw float64) {
	n.logger.Warn("robot stuck - triggering escape")
	diag := n.stuckDetector.GetDiagnostics()

	// "Not blocked" sentinel for the no-scan-yet case, reusing
	// NoDataRangeM rather than a second independent magic 10.0 -- both mean
	// the same thing: no valid reading, so assume clear rather than blocked.
	rearClear, forwardClear := n.cfg.NoDataRangeM, n.cfg.NoDataRangeM
	rearBlind := false
	scan, haveScan := n.gateway.GetLidarScan()
	if haveScan {
		// A rear sector that measured nothing reports the same sentinel as
		// a genuinely empty one, so the distance alone cannot tell them
		// apart. Tracked separately so the two stay distinguishable below.
		rear := n.collisionController.RearSector(scan.RangesM, scan.AnglesRad)
		rearBlind = !rear.Measured()
		// Both ends as BUMPER gaps, so the single contact distance below
		// means the same thing in each direction. The sentinel survives the
		// conversion -- 10 m less either datum is still open road -- so the
		// no-scan branch keeps its "assume clear" meaning.
		rearClear = controllers.BumperGapBehind(rear.MinRangeM, n.cfg.LidarToRearBumperM)
		forwardClear = controllers.BumperGapAhead(
			n.collisionController.ComputeForwardClearance(scan.RangesM, scan.AnglesRad),
			n.cfg.LidarToFrontBumperM,
		)
	}

	// Blind behind is a reason to prefer forward, but only when forward is
	// actually open. Treating it as flatly "blocked" would leave a chassis
	// with no rear vision at all frozen in every corner where both ends read
	// blocked; there, an unseen reverse is still the better of two bad
	// options -- but only when the pose trail vouches for it. A blind rear
	// with NO trail is the exact "cannot see behind" case, and the
	// fall-through below would reverse into whatever moved in since.
	stuckReverseDistance := math.Abs(n.cfg.RevSpeed) * float64(n.cfg.MaxEscapeFrames) / n.cfg.ControlHz
	blindRearUnconfirmed := rearBlind && !n.trailConfirmsReverse(stuckReverseDistance)

	rearBlocked := rearClear < n.cfg.ContactDistM
	preferForward := rearBlind && forwardClear >= n.cfg.ContactDistM
	if rearBlocked || preferForward || blindRearUnconfirmed {
		if forwardClear >= n.cfg.ContactDistM {
			rearState := "blocked"
			if rearBlind {
				rearState = "unseen"
			}
			n.logger.Warn("stuck escape: forcing forward escape",
				"rear_state", rearState, "rear_clearance_m", rearClear, "forward_clearance_m", forwardClear)
			n.beginStuckEscape(
				robotX, robotY, robotYaw, controllers.ManeuverStuckForward,
				n.cfg.RevSteerNorm()*n.escapeSteerSignForAttempt(1, nil), n.cfg.CreepSpeedMPS(),
				diag, rearClear, forwardClear, true,
			)
			return
		}

		// Both ends blocked and rear unmeasurable (the current build carries
		// no rear slot): a frozen hold used to deadlock here, re-arming the
		// same failed command every stuck window until the run timed out.
		// The safe, rear-free recovery is a LOW-SPEED PIVOT forward -- never
		// a reverse, since the rear gate cannot authorize one without a
		// sensor -- steering toward the more open side so the chassis
		// reorients out of the wedge instead of sitting in it.
		steerSign := n.pivotSteerSign(scan, haveScan)
		n.logger.Warn("stuck escape both-blocked: stop-and-steer pivot",
			"rear_clearance_m", rearClear, "forward_clearance_m", forwardClear, "steer_sign", steerSign)
		n.beginStuckEscape(
			robotX, robotY, robotYaw, controllers.ManeuverStuckForward,
			n.cfg.RevSteerNorm()*steerSign, n.cfg.CreepSpeedMPS(),
			diag, rearClear, forwardClear, true,
		)
		return
	}

	n.beginStuckEscape(
		robotX, robotY, robotYaw, controllers.ManeuverStuckReverse,
		n.cfg.RevSteerNorm()*n.escapeSteerSignForAttempt(1, nil), n.cfg.RevSpeed,
		diag, rearClear, forwardClear, false,
	)
}
