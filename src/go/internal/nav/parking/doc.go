// Package parking drives the robot into the bay between the two magenta
// parking blocks, the final maneuver of the WRO 2026 Obstacles challenge.
// Ports platform/robot/src/navigation/maneuvers/parking.
//
// The challenge adds a parallel-park after completing the laps. WRO's rule is
// exact, not approximate: the robot's whole projection on the mat must lie
// inside the rectangle between the two markers AND it must be parallel to the
// field wall -- "parallel" meaning the two wheels on one side differ by no
// more than 2 cm in their distance to that wall. A center-in-box test passes
// a robot sitting mostly in the corridor or with its nose through the wall,
// because neither the footprint nor the heading enters into it; that shortcut
// is why the Python predecessor "succeeded" 0/240 times it actually contained.
// So the stop condition here is genuine footprint containment, not a proxy.
//
// The maneuver is two phases:
//
//   - STAGE: pure-pursuit toward a staging point in front of the bay opening,
//     on the track side. This leaves the robot aimed at the gap with room to
//     swing, instead of arriving at the lot already committed.
//   - ENTER: pure-pursuit toward the bay center. It stops the moment the whole
//     footprint is inside the lot AND the heading is wall-parallel within
//     tolerance. If the footprint would reach the field wall or a marker fin
//     first, it gives up (holding position) rather than colliding -- not
//     colliding beats parking.
//
// ENTER still pure-pursues a single point, which steers for position without
// controlling the final heading. With the honest containment stop condition,
// that is not enough to actually park this chassis: the controller will
// time out rather than falsely report success. The real entry maneuver is a
// separate piece of work (see platform/docs/internal/2026-07-25-parking-review.md);
// this port preserves that known limitation rather than papering over it with
// the old center-in-box test.
//
// # Do not re-add a center-in-box containment test
//
// Dropping footprint_inside for a center-of-chassis-in-rectangle check would
// restore the 0/240 "success" rate the honest test replaced: a robot parked
// across the bay has its center well inside the rectangle while most of the
// chassis sits out in the corridor, and a nose-through-the-wall pose reports
// contained too. The old check could not see either, so it passed the
// maneuver at the exact moment it had failed it. Keep footprint_inside.
//
// # Deliberate drops from the Python original
//
// A few Python no-ops are not replicated here; flagged so a future reader
// does not "fix" them back in:
//
//   - ParkingContext/TuningContext container and the `from_tuning` overrides:
//     the Python context exists only to inject tuning that would otherwise
//     freeze as module-level globals. Go threads Config explicitly through
//     every call instead (ConfigFor loads the TOML tree), so there is no
//     equivalent indirection to port.
//   - park_controller_from_metadata's DictKeys/parking-lot-from-scenario
//     dict unpacking: that is the ROS2/metadata boundary. The Go equivalent
//     (building a ParkingLot from a generated scenario) is the caller's job;
//     ParkControllerFromMetadata instead takes typed parking-lot geometry
//     directly, which is what the caller already holds after parsing.
//   - DEFAULT_PARKING_CONTEXT module singleton: replaced by passing Config
//     (a value, not a pointer) into footprint_inside/breaches_* and
//     pure_pursuit_steer, matching how sibling nav packages carry tuning.
package parking
