package bayexit

import "math"

// point is a 2D coordinate in the BAY FRAME: along runs down the wall along
// the start heading, out runs away from the wall toward the corridor.
type point struct {
	along, out float64
}

// rectCorners returns the corners of a length x width rectangle centred at
// (along, out), rotated by yaw, matching _rect_corners.
func rectCorners(along, out, yaw, length, width float64) []point {
	ca, sa := math.Cos(yaw), math.Sin(yaw)
	hl, hw := length/2.0, width/2.0
	signs := [4][2]float64{{1, 1}, {1, -1}, {-1, -1}, {-1, 1}}
	corners := make([]point, 4)
	for i, s := range signs {
		sl, sw := s[0], s[1]
		corners[i] = point{
			along: along + sl*hl*ca - sw*hw*sa,
			out:   out + sl*hl*sa + sw*hw*ca,
		}
	}
	return corners
}

// gap is the separating-axis gap between two convex polygons; negative
// means overlap, matching _gap.
//
// Under-estimates vertex-to-vertex gaps, which is the safe direction for a
// guard whose job is to never touch.
func gap(polyA, polyB []point) float64 {
	best := math.Inf(-1)
	for _, poly := range [][]point{polyA, polyB} {
		count := len(poly)
		for i := range count {
			p1, p2 := poly[i], poly[(i+1)%count]
			ax, ay := p2.out-p1.out, p1.along-p2.along
			norm := math.Hypot(ax, ay)
			if norm == 0.0 {
				continue
			}
			ax, ay = ax/norm, ay/norm

			aLo, aHi := projectExtent(polyA, ax, ay)
			bLo, bHi := projectExtent(polyB, ax, ay)
			best = max(best, bLo-aHi, aLo-bHi)
		}
	}
	return best
}

// projectExtent is the [min, max] projection of poly's vertices onto the
// unit axis (ax, ay), the inner reduction _gap's Python comprehensions
// perform inline.
func projectExtent(poly []point, ax, ay float64) (lo, hi float64) {
	lo, hi = math.Inf(1), math.Inf(-1)
	for _, p := range poly {
		v := p.along*ax + p.out*ay
		lo = min(lo, v)
		hi = max(hi, v)
	}
	return lo, hi
}

// finRects returns the two marker fins in the BAY FRAME, from the rulebook
// geometry, matching _fin_rects.
//
// Origin is where the chassis was placed -- the pocket centre, on the lot's
// axis, ParkingLot.WallOffsetM out from the wall. The fins stand at +-half
// the block spacing, are Width thick, and span the lot's full depth from the
// wall to their tips.
func finRects(lot Config) [2][]point {
	halfSpacing := lot.ParkingLot.BlockSpacingFactor * lot.ChassisLengthM / 2.0
	inner := halfSpacing - lot.ParkingLot.Width/2.0
	outer := halfSpacing + lot.ParkingLot.Width/2.0
	wall := -lot.ParkingLot.WallOffsetM
	tip := wall + lot.ParkingLot.Length
	return [2][]point{
		{{along: -outer, out: wall}, {along: -inner, out: wall}, {along: -inner, out: tip}, {along: -outer, out: tip}},
		{{along: inner, out: wall}, {along: outer, out: wall}, {along: outer, out: tip}, {along: inner, out: tip}},
	}
}
