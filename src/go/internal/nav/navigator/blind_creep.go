package navigator

import (
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/bayexit"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/corridorfollower"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/directionestimator"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/startmeasurement"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// blindCreep drives the BLIND_CREEP phase: creep along the corridor centred
// between visible walls, infer the travel direction from LIDAR, and accumulate
// camera sign detections, matching CoreNavigator.step's blind bootstrap. Once
// the direction settles (parking-bay read or enough agreeing scans), the
// navigator adopts it and the next tick follows the planned path.
func (n *Navigator) blindCreep(pose trackmodel.Pose) {
	debug := n.baseDebug(pose)
	debug.Phase = PhaseBlindCreep

	scan, haveScan := n.gateway.GetLidarScan()

	// Accumulate camera sign detections into the discovery map (discover
	// mode), so signs are published to the router as they confirm.
	if n.discovery != nil {
		n.discovery.Observe(n.visionDetections(), trackmodel.Waypoint{X: pose.X, Y: pose.Y})
		n.discovery.Publish()
	}

	// Take width readings during the creep as well. They cannot be filed
	// under a corridor yet -- that needs the direction -- but they are the
	// cleanest readings of the whole round. See recordCreepWidth.
	if haveScan {
		n.recordCreepWidth(scan, pose.Yaw)
	}

	// Resolve the direction; a settled one hands the tick back to the caller.
	if n.dirEstimator != nil && n.resolveBlindDirection(pose, scan, haveScan, debug) {
		return
	}

	// No direction yet: creep along the corridor. Without a scan there is
	// nothing to react to, so hold still rather than guess.
	if !haveScan {
		n.publishHold(debug)
		return
	}
	n.creepFollow(pose, scan, debug)
}

// resolveBlindDirection runs the direction-resolution half of the blind
// bootstrap, returning true when it published a command and the caller should
// stop. A boxed-in parking bay names the direction outright; otherwise scans
// are voted on.
func (n *Navigator) resolveBlindDirection(
	pose trackmodel.Pose, scan controllers.LidarScan, haveScan bool, debug DebugSnapshot,
) bool {
	boxed := n.checkBayStart(scan, haveScan)
	if !boxed && !n.exitingBay && haveScan {
		n.dirEstimator.Observe(scan, pose.Yaw, n.dirEstCfg)
	}
	if n.updateBayExit(scan, haveScan, pose, debug) {
		return true
	}
	if dir, ok := n.dirEstimator.Direction(); ok {
		n.adoptDirection(dir, pose, scan, haveScan, debug)
		return true
	}
	return false
}

// checkBayStart tests the parking-bay read ONCE (the first tick a scan is
// available). Re-testing every tick lets it fire mid-creep at a corner and
// settle the direction off geometry that is not a bay at all. It reports
// whether the bay named the direction outright.
func (n *Navigator) checkBayStart(scan controllers.LidarScan, haveScan bool) bool {
	if n.bayStartChecked || !haveScan {
		return false
	}
	n.bayStartChecked = true
	if dir, ok := directionestimator.DirectionFromParkingBay(scan, n.dirEstCfg); ok {
		n.dirEstimator.Settle(dir)
		n.exitingBay = true
		return true
	}
	if n.signRouter != nil && n.followerCfg.AssumeBayStart && !bayexit.IsClear(scan, n.bayExitCfg) {
		// The in-bay start is the one Obstacles intends to use, so believe it
		// rather than requiring the scan to prove it. Only the DIRECTION half
		// of the test failed, and the exit does not need one; boxed stays
		// false so the estimator resumes voting once the pocket is behind us.
		//
		// IsClear is tested HERE rather than left to the unlatch below because
		// the vote at !boxed && !exitingBay runs first: latching and
		// unlatching around it would cost a parallel start one direction vote,
		// which the Python node does not pay (its unlatch precedes its vote).
		// Same threshold either way -- a parallel start is a start with
		// forward clearance. See AssumeBayStart.
		n.exitingBay = true
	}
	return false
}

// updateBayExit drives the bay-exit maneuver while it is engaged, returning
// true when it published a command for this tick.
func (n *Navigator) updateBayExit(
	scan controllers.LidarScan, haveScan bool, pose trackmodel.Pose, debug DebugSnapshot,
) bool {
	// Out of the pocket. Falls through to the settle block rather than
	// returning, so the path is rebuilt for the committed direction once the
	// maneuver ends -- see bayexit.IsClear.
	if n.exitingBay && haveScan && bayexit.IsClear(scan, n.bayExitCfg) {
		n.exitingBay = false
	}
	if !n.exitingBay {
		return false
	}
	odom, odomOK := n.gateway.GetWheelOdometry()
	if !odomOK {
		// No odometry means the reverse leg cannot be bounded, and this
		// maneuver reverses toward a fin. Hold rather than guess.
		n.publishHold(debug)
		return true
	}
	if n.bayExit == nil {
		n.bayExit = bayexit.New()
	}
	cmd := n.bayExit.Command(scan, odom.DistanceM, n.cfg.CreepSpeedMPS(), n.bayExitCfg, &pose.Yaw)
	n.gateway.PublishDrive(cmd)
	debug.CommandedSpeedMPS = new(cmd.SpeedMPS)
	debug.CommandedSteerNorm = new(cmd.SteeringNorm)
	n.debug = debug
	return true
}

// adoptDirection commits a settled direction, measures the start pose so the
// map frame is corrected before the path is followed, and resyncs the path.
func (n *Navigator) adoptDirection(
	dir trackmodel.Direction, pose trackmodel.Pose, scan controllers.LidarScan, haveScan bool, debug DebugSnapshot,
) {
	n.direction = &dir
	if n.signRouter != nil {
		n.signRouter.AdoptDirection(dir)
	}
	if haveScan {
		if measured, measuredOK := startmeasurement.MeasureStartPose(
			scan, dir, trackmodel.South, n.startMeasCfg,
		); measuredOK {
			n.ApplyBelievedStart(
				trackmodel.Pose{X: measured.X, Y: measured.Y, Yaw: pose.Yaw},
				pose,
			)
		}
	}
	// Resync the path to where the chassis actually is, UNCONDITIONALLY --
	// including when the inferred direction agreed with the provisional one
	// and the path is unchanged. The navigator did not follow the path during
	// the creep, so its waypoint index is still 0 while the robot has driven a
	// meter past it: it would resume by chasing a waypoint behind itself.
	// Measured in Python: this alone cost fixtures that had inferred the
	// direction perfectly.
	//
	// The yaw is passed so the nearest-waypoint search breaks ties by heading
	// agreement -- at the end of a corridor the waypoint behind and the one
	// ahead are near-equidistant, and position alone picks between them
	// arbitrarily.
	n.ReplacePath(
		n.waypoints,
		trackmodel.Waypoint{X: pose.X, Y: pose.Y},
		&pose.Yaw,
	)
	n.debug = debug
}

// creepFollow steers along the corridor while the direction is still unknown.
func (n *Navigator) creepFollow(pose trackmodel.Pose, scan controllers.LidarScan, debug DebugSnapshot) {
	yaw := pose.Yaw
	followParams := corridorfollower.Params{
		SpeedMPS:       n.cfg.CreepSpeedMPS(),
		Yaw:            &yaw,
		ForcedTurnSide: n.signDodgeSide(pose),
	}
	if believed, ok := n.BelievedCreepWidthM(); ok {
		followParams.BelievedWidthM = &believed
	}
	cmd := corridorfollower.FollowCorridor(scan, followParams, n.followerCfg)
	n.gateway.PublishDrive(cmd)
	debug.CommandedSpeedMPS = new(cmd.SpeedMPS)
	debug.CommandedSteerNorm = new(cmd.SteeringNorm)
	n.debug = debug
}

// publishHold commands zero and records it, for a tick that cannot safely act.
func (n *Navigator) publishHold(debug DebugSnapshot) {
	n.gateway.PublishDrive(controllers.DriveCommand{})
	debug.CommandedSpeedMPS = new(0.0)
	debug.CommandedSteerNorm = new(0.0)
	n.debug = debug
}
