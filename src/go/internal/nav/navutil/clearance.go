package navutil

import (
	"math"
	"sort"
)

// BlindWedges bounds the mount's two occlusion wedges (radians, wrapped
// range), where rays self-collide by geometry regardless of measured range.
type BlindWedges struct {
	LeftMinRad, LeftMaxRad   float64
	RightMinRad, RightMaxRad float64
}

// medianEvenDivisor splits a sorted slice into left/right halves when
// averaging the two middle values.
const medianEvenDivisor = 2

// RearClearance is the minimum clearance in the rear sector, or ok=false
// when the mount cannot see it (matches utils._rear_clearance).
//
// false does not mean clear -- it means NO INFORMATION, and the two must
// not collapse into one number: a caller gating a reverse on this must
// refuse when ok is false, not treat it as an all-clear.
//
// Excludes, each for a different reason: the mount's occlusion wedges (see
// BlindWedges), readings at or below minValidRangeM (not measurements), and
// self-detection returns at or below selfDetectionThresholdM (the chassis
// and its own cabling).
func RearClearance(
	scan LidarScan,
	threatHalfFovRad, minValidRangeM, selfDetectionThresholdM float64,
	blindWedges BlindWedges,
) (clearanceM float64, ok bool) {
	best := math.Inf(1)
	found := false
	for i, a := range scan.AnglesRad {
		r := scan.RangesM[i]
		if math.Abs(WrapAngle(a-math.Pi)) > threatHalfFovRad {
			continue
		}
		if r <= minValidRangeM || r <= selfDetectionThresholdM {
			continue
		}
		wrapped := WrapAngle(a)
		if wrapped >= blindWedges.LeftMinRad && wrapped <= blindWedges.LeftMaxRad {
			continue
		}
		if wrapped >= blindWedges.RightMinRad && wrapped <= blindWedges.RightMaxRad {
			continue
		}
		found = true
		best = min(best, r)
	}
	if !found {
		return 0, false
	}
	return best, true
}

// WedgeMedian is the median valid range in a wedge about centerRad, or
// ok=false if none, matching utils._wedge_median. Median rather than mean
// or minimum: a mean is dragged by the occasional max-range no-return, and
// a minimum reports whatever speck is nearest rather than the wall the
// wedge is pointed at.
//
// Ranges outside [minValidRangeM, maxValidRangeM] are excluded; pass
// maxValidRangeM <= 0 to leave that bound uncapped (matching the Python
// default of None), and selfDetectionThresholdM <= 0 to skip that filter
// (also matching a None default).
func WedgeMedian(
	scan LidarScan,
	centerRad, halfWidthRad, minValidRangeM float64,
	selfDetectionThresholdM, maxValidRangeM float64,
) (medianM float64, ok bool) {
	valid := make([]float64, 0, len(scan.RangesM))
	for i, a := range scan.AnglesRad {
		r := scan.RangesM[i]
		if math.Abs(WrapAngle(a-centerRad)) > halfWidthRad {
			continue
		}
		if r <= minValidRangeM {
			continue
		}
		if maxValidRangeM > 0 && r >= maxValidRangeM {
			continue
		}
		if selfDetectionThresholdM > 0 && r <= selfDetectionThresholdM {
			continue
		}
		valid = append(valid, r)
	}
	if len(valid) == 0 {
		return 0, false
	}
	return median(valid), true
}

// median computes the median of values, which is mutated (sorted) in place.
func median(values []float64) float64 {
	sort.Float64s(values)
	n := len(values)
	if n%medianEvenDivisor == 1 {
		return values[n/medianEvenDivisor]
	}
	return (values[n/medianEvenDivisor-1] + values[n/medianEvenDivisor]) / medianEvenDivisor
}
