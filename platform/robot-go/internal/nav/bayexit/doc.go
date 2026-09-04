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
// travelledM) is NOT wired in yet. internal/nav/navigator's Gateway port has
// no wheel-odometry accessor at all currently (Python's
// get_wheel_odometry() has no Go counterpart), so wiring this in is a
// separate change that also touches the Gateway interface and both its
// implementations (internal/adapters/natsgw, internal/sim/harness), not
// just this package.
package bayexit
