// Package navigator is the composition root of the navigation stack: the Go
// port of platform/robot/src/navigation/core_navigator/navigator.py
// (CoreNavigator) and escape_recovery.py (the EscapeRecovery mixin, flattened
// onto Navigator here as it is onto CoreNavigator at runtime).
//
// Navigator.Step is one control tick: fetch pose, classify the corridor,
// record the pose trail, continue or trigger an escape maneuver, detect
// stuck, count laps, refresh the sign lanes, advance past passed waypoints,
// read the LIDAR, pick a lookahead and steering target, deform it through
// SignRouter, then run the speed ladder and publish. It composes
// internal/nav/controllers (pure pursuit, collision avoidance, stuck
// detection), internal/nav/signrouter (traffic-sign routing and pass-side
// lanes), internal/nav/trackmodel (path frame), internal/nav/waypoints
// (corridor classification) and internal/nav/navutil; it owns no geometry of
// its own beyond the trail-clearance helper in escape_recovery.go.
//
// # Scope and deviations from the Python source
//
// Three of CoreNavigator's collaborators have no Go port, so this first cut
// is deliberately narrower than the Python original. None of the three is
// stubbed: the corresponding parameters are absent from Params, so a caller
// cannot ask for behavior that is not here.
//
//   - LapDetector (race_tracker.py) is not ported. Lap counting is therefore
//     always Python's waypoint-wrap fallback branch -- the index running off
//     the end of the canonical lap IS the lap -- with no geometric
//     finish-line confirmation.
//   - ParkController (internal/nav/parking) IS ported and wired: once
//     LapsCompleted reaches NumLaps, handleFinish (path.go) mirrors
//     _handle_finish faithfully -- a nil ParkController (Open Challenge)
//     holds at zero speed/steering as before; a non-nil one defers the
//     handoff until the robot is in the parking corridor and within
//     Config.ParkEngageDistM of the staging point (shouldEngageParking,
//     matching _should_engage_parking), then drives ParkController.Update
//     each tick with the same forward/side LIDAR-clearance safety clamp
//     (controllers.ParkingClearances) that overrides the maneuver rather
//     than let it complete into a wall. IsRepositioning suspends and resets
//     the stuck detector during ParkController's own reverse-and-reorient
//     recovery, matching Python's suspension for the same reason (the
//     generic escape is blind to the bay's keep-out geometry).
//   - Blind-mode bootstrap IS ported: when Params.Direction is nil the
//     navigator runs the BLIND_CREEP phase (corridor_follower.FollowCorridor
//     creep centred between visible walls + directionestimator.InferDirection
//     / DirectionFromParkingBay settling the travel direction), accumulating
//     camera signs through signrouter.ObservedSignMap (discover mode) and
//     applying the start_measurement believed-offset (BelievedYawOffset /
//     ApplyBelievedStart) once the direction resolves. A boxed-in parking-bay
//     start additionally hands off to internal/nav/bayexit's BayExit for the
//     ticks it takes to clear the pocket (see blindCreep's exitingBay
//     handling), matching track_navigator_node.py's `_exiting_bay` state
//     machine; on real hardware this holds rather than exits, since the
//     Gateway has no encoder topic to read wheel odometry from yet.
//     corridor_estimator.py (running corridor-width averaging) is the one
//     blind piece still not wired here -- the creep uses the plain
//     TurnClearanceM; a sighted Direction (non-nil) skips the whole phase,
//     preserving the original behavior.
//
// Everything else -- the full Step control flow, the whole escape/stuck
// recovery subsystem, SignRouter integration including the pass-side lane
// planner, ReplacePath's nearest-waypoint reseek, ReplaceSignRouter,
// SetTravelDirection and Reset -- is ported faithfully.
package navigator
