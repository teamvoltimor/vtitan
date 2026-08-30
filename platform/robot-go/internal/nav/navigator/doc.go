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
//   - ParkController (maneuvers/parking.py) is not ported. Finish handling
//     collapses to _handle_finish's `pc is None` branch: once LapsCompleted
//     reaches NumLaps the robot holds at zero speed and zero steering
//     forever. There is no parking maneuver, no engage distance, and no
//     is_repositioning suspension of stuck detection.
//   - Blind-mode bootstrap (corridor_follower.py, corridor_estimator.py,
//     start_measurement.py) is not ported, matching internal/nav/signrouter's
//     own sighted-only scope. Params.Direction is a required, already-known
//     trackmodel.Direction rather than Python's Direction | None, so there is
//     no blind-creep phase, no _resolve_direction/_commit_direction, and the
//     BLIND_CREEP phase is never emitted. SignRouter.is_discovering has no
//     counterpart either, which makes the explore-lap speed cap unreachable
//     (see selectSpeed).
//
// Everything else -- the full Step control flow, the whole escape/stuck
// recovery subsystem, SignRouter integration including the pass-side lane
// planner, ReplacePath's nearest-waypoint reseek, ReplaceSignRouter,
// SetTravelDirection and Reset -- is ported faithfully.
package navigator
