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
// DefaultConfig's field values mirror
// shared.config.navigation_tuning.blind_nav.DirectionEstimatorParams and
// (for DirectionFromParkingBay's forward-clearance/wall-clearance checks)
// CorridorFollowerParams/LidarSectorParams; ConfigFor loads the real
// values from platform/config/navigation/**'s TOML files via
// internal/config/profile, falling back to DefaultConfig's literals when
// no config root is supplied or loading fails.
package directionestimator
