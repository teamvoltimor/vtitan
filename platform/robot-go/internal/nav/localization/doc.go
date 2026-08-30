// Package localization estimates absolute position by matching a LIDAR sweep
// against known wall geometry, porting src/navigation/localization.py.
//
// The robot has no wheel odometry -- nothing publishes nav_msgs/Odometry --
// and a general-purpose scan-matching or SLAM library is unwarranted for a
// track this constrained. The wall layout for the round is already known from
// scenario metadata, so position is recovered directly: find the pose whose
// PREDICTED scan against that known geometry best matches the REAL scan.
//
// # The search is local, not a relocalization
//
// The robot starts at a known position and moves only centimeters between
// 20 Hz ticks, so each estimate is a tight seed for the next. That is also
// why corners and straights need no special-casing: a direct geometric
// wall-distance approach would have to decide "which wall is my nearest
// wall", and this does not.
//
// # Two guards, and why the obvious third one does not exist
//
// A free-space check rejects any match landing outside the outer walls OR
// inside the inner block -- both are equally impossible, so both get the same
// guard rather than a narrower bounds-only check.
//
// A speed bound rejects a candidate implying motion the drivetrain cannot
// produce, holding it until the SAME candidate wins again on the next tick. A
// real correction reconverges to nearly the same position from an independent
// scan; the ambiguous flip this guards against does not typically repeat.
//
// The obvious third guard -- rejecting a winner whose cost margin over the
// runner-up is thin -- was tried, committed, and REVERTED on 2026-08-05 after
// replaying 22 real hardware runs (846 sampled ticks). Confirmed-bad and
// genuinely correct matches had statistically indistinguishable cost and
// margin distributions on real noisy scans. The signal it depended on exists
// only in the clean simulator. Do not re-add it without new evidence.
package localization
