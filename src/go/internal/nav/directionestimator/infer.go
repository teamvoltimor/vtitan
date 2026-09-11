package directionestimator

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"
)

// InferDirection reports which way round the loop rangesM/anglesRad implies,
// or ok=false if it cannot say -- the normal state until the robot nears the
// end of its corridor.
//
// Decides on the SPAN (left+right), not on either range alone: two walls
// span the corridor wherever the chassis sits between them, so left+right
// stays at the corridor width until one side stops being a wall, at which
// point it jumps by a corridor length. That makes the test immune to being
// off-center. Comparing the two ranges directly does not work: drifted
// toward the inner block, a robot reads 0.27m to the block on its left and
// 0.72m to the outer wall on its right, and "the larger side is open"
// would then pick the outer wall -- exactly the wrong answer, since that is
// which wall is nearer, not which side is open.
//
// yaw is only used to check the chassis is roughly aligned with a corridor
// (off-axis the side rays cut a diagonal and read long for no good reason);
// no map is consulted.
func InferDirection(
	scan controllers.LidarScan,
	yaw float64,
	cfg Config,
) (dir Direction, ok bool) {
	if navutil.AxisErrorRad(yaw) > cfg.AlignmentToleranceRad {
		return 0, false
	}

	left := navutil.NearestRay(scan, math.Pi/2)
	right := navutil.NearestRay(scan, -math.Pi/2)

	// A dropout carries no information about whether a side is open, and
	// the span test below cannot tell one from a corridor running away:
	// both read long. Reject rather than guess.
	if left > cfg.MaxInTrackRangeM || right > cfg.MaxInTrackRangeM {
		return 0, false
	}

	if left+right <= cfg.PlausibleSpanThresholdM {
		return 0, false
	}
	if math.Abs(left-right) < cfg.MinAsymmetryM {
		return 0, false
	}

	// The inner block is on the side that opened, and the block's side
	// fixes the rotational sense: block on the right means clockwise.
	if right > left {
		return Clockwise, true
	}
	return Counterclockwise, true
}

// DirectionFromParkingBay reads travel direction straight off a start
// inside the parking bay, without moving -- ok=false unless the scan really
// is the boxed-in case (forward blocked, hard against one side, wide open
// on the other), so an ordinary start on the centreline never matches and
// InferDirection is left to do its normal job.
//
// The lot is always against the outer wall, so its opening necessarily
// faces the inner block, and a lap always turns toward the inner block:
// open side, inner side, and corner-turn side are the same side by track
// design. That inverts InferDirection's usual difficulty -- from the
// corridor centerline both sides read comparable ranges, which is why it
// has a history of failing to settle, but boxed in the bay the robot sits
// ~0.10m off the outer wall with meters of open corridor opposite, so the
// asymmetry is enormous and names the answer outright. Worth having
// because the alternative is a deadlock, not a delay: nothing settles
// until the robot moves and nothing moves until the direction settles.
func DirectionFromParkingBay(scan controllers.LidarScan, cfg Config) (dir Direction, ok bool) {
	arcRad := cfg.DirectionArcHalfFovDeg * math.Pi / navutil.DegreesPerHalfTurn
	if navutil.ForwardClearance(
		scan,
		arcRad,
		cfg.MinValidRangeM,
	) >= cfg.MinForwardClearanceM {
		return 0, false
	}

	left := navutil.NearestRay(scan, math.Pi/2)
	right := navutil.NearestRay(scan, -math.Pi/2)

	if min(left, right) > cfg.BayWallClearanceM || max(left, right) <= cfg.TurnClearanceM {
		return 0, false
	}
	if left > right {
		return Counterclockwise, true
	}
	return Clockwise, true
}
