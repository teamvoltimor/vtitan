// Package bayexit ports
// platform/robot/src/navigation/maneuvers/bay_exit.py: backing the chassis
// out of the parking pocket at the start of a round.
//
// The WRO rules allow two legal starts: inside the parking lot, or parallel
// to it in the same section. Getting OUT of the pocket is a different
// problem from getting in -- internal/nav/parking drives the entry, and
// this drives the exit.
//
// Three exit strategies are ported, selected by Config (matching Python's
// tuning-driven selection): the legacy reverse-then-swing exit (Command
// falls through to it when both BayExitCycle and BayExitClearanceGuard are
// false), the alternating arc/straight-reverse "cycle" exit
// (BayExitCycle, the shipped default), and the clearance-guarded cycle exit
// (BayExitClearanceGuard, the legal replacement that bounds each leg by
// PREDICTED fin clearance instead of by contact -- see BayExitClearanceGuard's
// doc comment for why the contact-bounded exits violate rule 9.24.7).
// BayExitFallbackFrames can additionally switch from the configured exit to
// the other one mid-round, matching Python's fallback mechanism.
//
// track_navigator_node.py's `_exiting_bay` state machine (the caller that
// decides WHEN to invoke BayExit -- boxed-in-bay detection via
// directionestimator.DirectionFromParkingBay, calling IsClear each tick to
// know when to stop, and reading wheel odometry to pass as Command's
// travelledM) IS wired in now, in internal/nav/navigator's blindCreep.
//
// The wheel-odometry accessor (controllers.HardwareGateway.GetWheelOdometry)
// already existed on the port; it only lacked a real producer.
// internal/sim/harness's SimHardwareGateway now accumulates a genuine
// signed distance (matching the Python oracle's heading-projected
// _wheel_distance_m update) instead of reporting zero. Real hardware
// (internal/adapters/natsgw) still has no encoder topic to read, so
// GetWheelOdometry there continues to report ok=false -- the navigator
// holds (publishes zero drive) rather than run the exit blind, exactly as
// the Python node does when its gateway returns None.
package bayexit
