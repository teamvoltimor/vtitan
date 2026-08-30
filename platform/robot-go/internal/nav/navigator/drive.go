package navigator

import (
	"math"
	"slices"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// assessPerception reads the LIDAR and derives forward clearance and both
// risk levels, matching step()'s scan block.
func (n *Navigator) assessPerception(pose trackmodel.Pose) perception {
	scan, haveScan := n.gateway.GetLidarScan()
	p := perception{scan: scan, haveScan: haveScan}

	if haveScan {
		// Converted to a BUMPER gap once, here, rather than at each of the
		// comparisons below: the no-LIDAR fallback assigns a threshold
		// value to this same variable, so the two branches have to leave
		// it in one frame or the degraded path means something different
		// from the measured one.
		p.forwardClearance = controllers.BumperGapAhead(
			n.collisionController.ComputeForwardClearance(scan.RangesM, scan.AnglesRad),
			n.cfg.LidarToFrontBumperM,
		)
		p.risk = n.collisionController.AssessRisk(scan.RangesM, scan.AnglesRad)
		p.escapeRanges = scan.RangesM
		if len(scan.RangesM) > 0 {
			p.minRange = new(slices.Min(scan.RangesM))
		}
	} else {
		// No LIDAR: a degraded sensor is not open road. Drive cautiously
		// (slow zone + non-SAFE risk) instead of blasting forward blind.
		p.forwardClearance = n.cfg.SlowDistM
		p.risk = controllers.RiskObstacle
	}
	p.escapeRisk = p.risk

	if haveScan && n.signRouter != nil {
		mapped := make([]controllers.MappedObstacle, 0, len(n.signRouter.RoutedSignPositionsByCorridor()))
		for _, sign := range n.signRouter.RoutedSignPositionsByCorridor() {
			mapped = append(mapped, controllers.MappedObstacle{Position: sign.Waypoint, Corridor: sign.Corridor})
		}
		p.escapeRanges = controllers.MaskMappedObstacles(
			scan.RangesM, scan.AnglesRad, pose, mapped,
			n.cfg.EscapeMaskRadiusM, n.cfg.CornerMinM, n.cfg.CornerMaxM,
		)
		p.escapeRisk = n.collisionController.AssessRisk(p.escapeRanges, scan.AnglesRad)
	}
	return p
}

// driveNormally is step()'s tail: steering selection, the speed ladder, the
// sign-contact evade, the escape trigger, and the normal publish.
func (n *Navigator) driveNormally(pose trackmodel.Pose, p perception) {
	robotX, robotY, robotYaw := pose.X, pose.Y, pose.Yaw
	here := trackmodel.Waypoint{X: robotX, Y: robotY}

	// Steer at a lookahead point, not directly at the (often much closer)
	// next waypoint -- otherwise the lookahead distance is computed but
	// discarded, producing weave on straights and corner cutting.
	//
	// Lookahead selection is gated on crosstrack error (how far off the
	// planned path the robot actually is), not forward LIDAR clearance.
	crosstrack := trackmodel.CrossTrackError(n.waypoints, robotX, robotY)
	// Crosstrack alone arms the short lookahead only after a corner has
	// been missed; the path's own upcoming turn arms it on entry.
	turnAhead := trackmodel.PathTurnAhead(n.waypoints, n.waypointIndex, n.cfg.CornerPreviewDistanceM)
	signAhead := n.signAhead(robotX, robotY, robotYaw)
	lookahead := n.waypointController.SelectLookahead(crosstrack, turnAhead, signAhead)
	// Full waypoint list, not a slice from waypointIndex -- SelectTargetPoint
	// wraps the search around the lap itself; slicing here would cut that
	// wraparound off again.
	steerTarget := n.waypointController.SelectTargetPoint(here, robotYaw, n.waypoints, n.waypointIndex, lookahead)

	steerTarget, signDeformMagnitude, activeSignCount := n.applySignRouting(steerTarget, here, robotYaw)

	dt := 0.0
	if n.cfg.ControlHz > 0 {
		dt = 1.0 / n.cfg.ControlHz
	}
	steering, _, angleError := n.waypointController.ComputeSteering(here, robotYaw, steerTarget, crosstrack, dt)

	speed, clearanceSpeed, headingSpeed := n.selectSpeed(p, angleError, turnAhead, signDeformMagnitude)

	debug := n.baseDebug(robotX, robotY, robotYaw)
	debug.ForwardClearanceM = new(p.forwardClearance)
	debug.MinLidarRangeM = p.minRange
	debug.Risk = new(p.risk)
	debug.EscapeRisk = new(p.escapeRisk)
	debug.CrosstrackErrorM = new(crosstrack)
	debug.LookaheadDistance = new(lookahead)
	debug.PathTurnAheadRad = new(turnAhead)
	debug.SteerTargetX = new(steerTarget.X)
	debug.SteerTargetY = new(steerTarget.Y)
	debug.AngleErrorRad = new(angleError)
	debug.ClearanceSpeedMPS = new(clearanceSpeed)
	debug.HeadingSpeedMPS = new(headingSpeed)
	debug.SignDeformMagnitudeM = signDeformMagnitude
	debug.ActiveSignCount = activeSignCount

	// Last-resort geometric guard against clipping a routed sign. The two
	// responses that already exist both assume the planner has the sign
	// handled -- the escape mask suppresses any reaction to it, and without
	// that mask the generic escape reverses and swings, which in a 1.0 m
	// corridor trades sign strikes for wall strikes. That assumption holds
	// sighted, where the lane is placed a corridor ahead.
	//
	// Deliberately NOT gated on LIDAR risk: a return reads CRITICAL only at
	// contact range, too late for any steering command to matter, which is
	// why a risk-gated version of this measured flat.
	if n.cfg.SignContactEvade && n.signRouter != nil {
		if evade, evading := n.signEvadeSteer(robotX, robotY, robotYaw); evading {
			steering = navutil.Clamp(steering+evade, -1.0, 1.0)
			speed = math.Min(speed, n.cfg.CreepSpeedMPS())
		}
	}

	if n.tryEscape(pose, p, debug) {
		return
	}

	// Normal publish -- clear the escape escalation, but only once the
	// robot has actually moved since the sequence started. A single
	// normal-drive tick between escape attempts does not mean the escape
	// worked: confirmed on real hardware 2026-08-04, normal_drive ->
	// escape_triggered alternated for 34+ seconds with the robot pinned in
	// place, and an unconditional reset zeroed the count every cycle so it
	// never reached EscalateAfterAttempts.
	if n.escapeSequenceStartXY == nil ||
		math.Hypot(robotX-n.escapeSequenceStartXY.X, robotY-n.escapeSequenceStartXY.Y) >= n.cfg.StuckMoveThreshold {
		n.escapeCount = 0
		n.escapeSequenceStartXY = nil
	}
	n.gateway.PublishDrive(controllers.DriveCommand{SpeedMPS: speed, SteeringNorm: steering})
	debug.Phase = PhaseNormalDrive
	debug.CommandedSpeedMPS = new(speed)
	debug.CommandedSteerNorm = new(steering)
	debug.EscapeCount = new(n.escapeCount)
	n.debug = debug
}

// selectSpeed runs the speed ladder, matching step()'s speed block, and
// returns the commanded speed alongside the two attribution values the
// debug snapshot reports (the clearance-only and heading-only choices).
//
// Python also caps the first lap of a DISCOVERING run at
// EXPLORE_LAP_SPEED_FRAC of the ceiling. That branch is gated on
// SignRouter.is_discovering, which this port does not have (blind discovery
// is out of scope -- see signrouter's doc.go), so it is omitted rather than
// approximated by a condition that would fire on sighted runs too. It is
// inert by default in any case: EXPLORE_LAP_SPEED_FRAC ships at 1.0.
func (n *Navigator) selectSpeed(
	p perception, angleError, turnAhead float64, signDeformMagnitude *float64,
) (speed, clearanceSpeed, headingSpeed float64) {
	switch {
	case p.forwardClearance < n.cfg.ContactDistM:
		speed = n.cfg.CreepSpeedMPS()
	case p.forwardClearance < n.cfg.SlowDistM:
		speed = n.cfg.SlowSpeedMPS()
	case p.forwardClearance < n.cfg.MediumDistM:
		speed = n.cfg.MediumSpeedMPS()
	default:
		speed = n.cfg.FastSpeedMPS()
	}
	// Captured before the heading limiter, the envelope clamp and the risk
	// cap all fold into speed. Reporting the post-min value under this name
	// made the two debug fields satisfy final <= heading by construction, so
	// the heading limiter looked innocent on 100% of ticks while it was in
	// fact the binding constraint on most of them.
	clearanceSpeed = speed

	// Never take a sharp turn at a speed the steering actuator cannot keep
	// up with: the servo has a fixed slew rate independent of chassis
	// speed, so a big required correction taken at full speed demands a yaw
	// rate it cannot track -- it saturates, overshoots and oscillates.
	// Clearance alone never catches this: a corner can have 0.50 m+ of open
	// space ahead while still demanding a 90-180 deg correction.
	//
	// Only the CRAWL threshold reduces speed. This ladder briefly mirrored
	// the clearance one; measured on hardware 2026-08-09 that cost 33% of
	// lap time for nothing, since ordinary cornering sits at 23-45 deg of
	// heading error, so the middle rungs taxed every corner on the track
	// rather than catching a dangerous case.
	headingSpeed = n.cfg.FastSpeedMPS()
	if math.Abs(angleError) >= n.cfg.CrawlRad {
		headingSpeed = n.cfg.CreepSpeedMPS()
	}
	speed = math.Min(speed, headingSpeed)

	// Bound the selected cruise speed by the configured envelope. Applied
	// HERE, to a zone speed that is always positive, and not to the final
	// command: clamping that up to the floor would turn every legitimate
	// stop (escape handoff, blocked at both ends) into a crawl the robot
	// cannot be commanded out of.
	speed = math.Min(math.Max(speed, n.cfg.MinSpeedMPS()), n.cfg.MaxSpeedMPS())

	// Never blast past a non-forward obstacle (e.g. a sign alongside the
	// robot) just because the path ahead is clear.
	if p.risk != controllers.RiskSafe {
		speed = math.Min(speed, n.cfg.SlowSpeedMPS())
	}

	// Give the pursuit controller more time to close a sign-avoidance
	// offset. Neither clearance nor heading-error speed reacts to one: a
	// deformation biases the STEERING TARGET sideways without necessarily
	// shrinking forward clearance or growing heading error. Gated on the
	// deformation the router actually applied THIS tick, not proximity to a
	// sign, so it only fires while a correction is genuinely in flight.
	if n.cfg.SignAwareSpeed && signDeformMagnitude != nil &&
		*signDeformMagnitude > n.cfg.SignDeformSpeedThresholdM {
		speed = math.Min(speed, n.cfg.SlowSpeedMPS())
	}

	// First-lap corner caution. A mixed-width corner's PLANNED arc is safe
	// by construction (verified 2026-08-28: clearance to both outer walls
	// never drops below what the straights already have), but real hardware
	// wedged at exactly this kind of corner anyway, which points at CONTROL
	// tracking error eating the plan's margin, not the plan itself. Only
	// the first lap has never actually been driven.
	if n.lapsCompleted == 0 && turnAhead != 0 {
		speed = math.Min(speed, n.cfg.SlowSpeedMPS())
	}
	return speed, clearanceSpeed, headingSpeed
}
