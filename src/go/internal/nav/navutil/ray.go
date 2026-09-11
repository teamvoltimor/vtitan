package navutil

import "math"

// NearestRay returns the range whose bearing is closest to target, matching
// utils._nearest_ray. Panics if rangesM/anglesRad are empty or of mismatched
// length -- callers always pass a real LIDAR scan, where that would already
// be a bug upstream.
func NearestRay(scan LidarScan, target float64) float64 {
	bestIndex := 0
	bestDiff := math.Inf(1)
	for i, angle := range scan.AnglesRad {
		diff := math.Abs(WrapAngle(angle - target))
		if diff < bestDiff {
			bestDiff = diff
			bestIndex = i
		}
	}
	return scan.RangesM[bestIndex]
}

// ForwardClearance is the minimum valid range within arcRad of dead ahead,
// or +Inf if no ray in that arc is valid, matching utils._forward_clearance
// (lidar_sectors.DIRECTION_ARC_HALF_FOV_DEG, MIN_VALID_RANGE_M).
func ForwardClearance(scan LidarScan, arcRad, minValidRangeM float64) float64 {
	clearance := math.Inf(1)
	for i, angle := range scan.AnglesRad {
		if math.Abs(WrapAngle(angle)) > arcRad || scan.RangesM[i] <= minValidRangeM {
			continue
		}
		clearance = min(clearance, scan.RangesM[i])
	}
	return clearance
}
