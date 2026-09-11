package controllers_test

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
)

// numRays/forwardSectorIndices/rearSectorIndices/lidarCloseThreat/
// lidarDefaultFar/minForwardClearance/minRearClearance mirror
// tests/test_constants.py's NUM_RAYS/FORWARD_SECTOR_INDICES/
// REAR_SECTOR_INDICES/LIDAR_CLOSE_THREAT/LIDAR_DEFAULT_FAR/
// MIN_FORWARD_CLEARANCE/MIN_REAR_CLEARANCE, the values the Python oracle
// suite builds every collision-avoidance fixture scan from.
const (
	numRays              = 360
	forwardSectorIndices = 4
	rearSectorIndices    = 8
	lidarCloseThreat     = 0.3
	lidarDefaultFar      = 10.0
	minForwardClearance  = 5.0
	minRearClearance     = 5.0

	// cornerMinM/cornerMaxM mirror track.toml's [track] corner_min/corner_max
	// (1.0/2.0) -- shared.config.constants.TrackDimensions.CORNER_MIN/MAX on
	// the Python side. This is what fixes which corridor a given (x, y) test
	// position classifies into for MaskMappedObstacles' corridor-match gate.
	cornerMinM = 1.0
	cornerMaxM = 2.0
)

// anglesFullRotation mirrors tests/test_constants.py's ANGLES_FULL_ROTATION:
// np.linspace(-pi, pi, NUM_RAYS), a CLOSED interval (both endpoints included,
// 359 even steps). This is deliberately NOT this package's own angle
// fallback for a nil angles slice (an OPEN [-pi, pi) sweep, see
// sectors.go's synthesizeAngles) -- that fallback only matters for the one
// test that omits angles entirely and relies on it instead of this fixture.
func anglesFullRotation() []float64 {
	return navutil.AngleFanClosed(numRays)
}

// newScan returns a uniform-range scan, matching fixtures.create_numpy_scan.
func newScan(defaultRangeM float64) []float64 {
	ranges := make([]float64, numRays)
	for i := range ranges {
		ranges[i] = defaultRangeM
	}
	return ranges
}

// scanObj packages a loose ranges/angles pair as the LidarScan value type the
// controller APIs take.
func scanObj(rangesM, anglesRad []float64) controllers.LidarScan {
	return controllers.LidarScan{RangesM: rangesM, AnglesRad: anglesRad}
}

// angleToIndex finds the ray index closest to bearingRad, matching
// fixtures.angle_to_index (np.argmin over |angles - bearing|, which returns
// the first minimal index on a tie).
func angleToIndex(angles []float64, bearingRad float64) int {
	best, bestDiff := 0, math.Inf(1)
	for i, a := range angles {
		diff := math.Abs(a - bearingRad)
		if diff < bestDiff {
			best, bestDiff = i, diff
		}
	}
	return best
}

// normalizeSliceIndex reproduces numpy/Python slice-index normalization
// (negative wraps from the end, everything clamps into [0, length]), needed
// because setSector below ports the oracle's `ranges[i-n:i+n] = value`
// idiom, and near the array's ends that expression can go negative or past
// the length exactly as a Python slice silently tolerates.
func normalizeSliceIndex(idx, length int) int {
	if idx < 0 {
		idx += length
	}
	if idx < 0 {
		idx = 0
	}
	if idx > length {
		idx = length
	}
	return idx
}

// setSector sets ranges[i-halfWidth : i+halfWidth] to value, matching the
// slicing idiom every Python oracle test uses to close a sector.
func setSector(ranges []float64, i, halfWidth int, value float64) {
	lo := normalizeSliceIndex(i-halfWidth, len(ranges))
	hi := normalizeSliceIndex(i+halfWidth, len(ranges))
	for k := lo; k < hi; k++ {
		ranges[k] = value
	}
}

// newDefaultCollisionAvoidanceController mirrors the Python oracle suite's
// `controller` fixture: CollisionAvoidanceController.from_tuning(
// NavigationTuning.load_default()).
func newDefaultCollisionAvoidanceController() *controllers.CollisionAvoidanceController {
	return controllers.DefaultConfig().NewCollisionAvoidanceController()
}

// newDefaultWaypointController mirrors WaypointController.from_tuning(
// NavigationTuning.load_default()).
//
// Config.NewWaypointController does not currently wire LookaheadBlendStart
// through from Config (see config.go's NewWaypointController), unlike
// Python's from_tuning, which always passes
// tuning.pursuit.LOOKAHEAD_BLEND_START explicitly. Set it here too, so these
// tests exercise WaypointController's own ramp logic against the same
// parameters the Python oracle uses rather than tripping over that separate
// Config-wiring gap.
func newDefaultWaypointController() *controllers.WaypointController {
	wc := controllers.DefaultConfig().NewWaypointController()
	wc.LookaheadBlendStart = controllers.DefaultLookaheadBlendStart
	return wc
}
