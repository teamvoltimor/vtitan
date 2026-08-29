// Package directionestimator infers which way round the WRO track loop the
// robot is traveling, from LIDAR alone -- the Go port of
// platform/robot/src/navigation/direction_estimator.py.
//
// Every corridor has the outer wall on one side and the inner block on the
// other. The block is finite and the outer wall is not, so driving toward
// the end of a corridor the block ends while the wall continues -- and the
// sideways ray on that side stops coming back at corridor width and instead
// runs off down the next corridor. That side is where the track turns:
// opening on the right means the block is on the right means clockwise;
// opening on the left means counterclockwise. See InferDirection's doc
// comment for the full discrimination.
//
// Config's field defaults mirror
// shared.config.navigation_tuning.blind_nav.DirectionEstimatorParams and
// (for DirectionFromParkingBay's forward-clearance/wall-clearance checks)
// CorridorFollowerParams -- restated here as literals rather than loaded
// from platform/shared/config/navigation/**'s TOML files because
// internal/config/profile doesn't cover the navigation-tuning tree yet
// (only robot.toml/track.toml and per-driver hardware TOMLs); migrate to a
// profile-loaded Config the same way internal/telemetry/diag.Config did
// once that tree is ported.
package directionestimator
