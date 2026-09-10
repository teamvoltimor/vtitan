package collision

import (
	"math"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// box is an axis-aligned keep-out rectangle, matching track_model.py's
// private _Box.
type box struct {
	xMin, yMin, xMax, yMax float64
}

// corners returns the four box corners, CCW from bottom-left, matching
// _Box.corners.
func (b box) corners() []trackmodel.Waypoint {
	return []trackmodel.Waypoint{
		{X: b.xMin, Y: b.yMin},
		{X: b.xMax, Y: b.yMin},
		{X: b.xMax, Y: b.yMax},
		{X: b.xMin, Y: b.yMax},
	}
}

// axesForYaw are the four separating-axis-test axes for an oriented
// rectangle vs. an axis-aligned box: the rectangle's two edge normals
// (which rotate with yaw) plus the world X/Y axes (the box's own normals).
// Exact for rectangle-vs-box overlap. Matches the identical `axes` tuple
// literal in both _convex_overlap and _convex_penetration.
func axesForYaw(yaw float64) [4][2]float64 {
	cosY, sinY := math.Cos(yaw), math.Sin(yaw)
	return [4][2]float64{
		{cosY, sinY},
		{-sinY, cosY},
		{1.0, 0.0},
		{0.0, 1.0},
	}
}

// rectCorners returns the four corners of an oriented rectangle centered at
// (cx, cy), matching track_model.py's _rect_corners.
func rectCorners(cx, cy, yaw, length, width float64) []trackmodel.Waypoint {
	hl, hw := length/navutil.Half, width/navutil.Half

	cosY, sinY := math.Cos(yaw), math.Sin(yaw)
	local := [4][2]float64{{hl, hw}, {hl, -hw}, {-hl, -hw}, {-hl, hw}}

	corners := make([]trackmodel.Waypoint, len(local))
	for i, l := range local {
		lx, ly := l[0], l[1]
		corners[i] = trackmodel.Waypoint{X: cx + lx*cosY - ly*sinY, Y: cy + lx*sinY + ly*cosY}
	}
	return corners
}

// projExtent returns the [min, max] projection of poly onto axis (ax, ay).
func projExtent(poly []trackmodel.Waypoint, ax, ay float64) (lo, hi float64) {
	lo, hi = math.Inf(1), math.Inf(-1)
	for _, p := range poly {
		proj := p.X*ax + p.Y*ay
		if proj < lo {
			lo = proj
		}
		if proj > hi {
			hi = proj
		}
	}
	return lo, hi
}

// convexOverlap is a separating-axis test between two convex polygons
// (rect vs AABB), matching track_model.py's _convex_overlap.
func convexOverlap(polyA, polyB []trackmodel.Waypoint, yaw float64) bool {
	for _, axis := range axesForYaw(yaw) {
		aMin, aMax := projExtent(polyA, axis[0], axis[1])
		bMin, bMax := projExtent(polyB, axis[0], axis[1])
		if aMax < bMin || bMax < aMin {
			return false // found a separating axis -> no overlap
		}
	}
	return true
}

// convexPenetration returns how deeply two overlapping convex polygons
// intrude, in meters: the minimum overlap across the same separating axes
// convexOverlap tests -- the magnitude of the translation that would just
// separate them. Zero when they do not overlap. Matches track_model.py's
// _convex_penetration.
//
// Used as the model for how far a touched pillar gets shoved: the chassis
// cannot occupy the pillar's space, so in reality the pillar is pushed
// ahead by roughly the distance the chassis has intruded.
func convexPenetration(polyA, polyB []trackmodel.Waypoint, yaw float64) float64 {
	smallest := math.Inf(1)
	for _, axis := range axesForYaw(yaw) {
		aMin, aMax := projExtent(polyA, axis[0], axis[1])
		bMin, bMax := projExtent(polyB, axis[0], axis[1])
		overlap := math.Min(aMax, bMax) - math.Max(aMin, bMin)
		if overlap <= 0.0 {
			return 0.0
		}
		if overlap < smallest {
			smallest = overlap
		}
	}
	if math.IsInf(smallest, 1) {
		return 0.0
	}
	return smallest
}

// raycastBox returns the distance from (x, y) to box b along direction
// (dx, dy), clamped to maxRange for a miss, matching track_model.py's
// _raycast_box (the scalar, per-ray equivalent of its vectorised form --
// Go has no numpy, so RaycastScan calls this once per ray per obstacle
// instead of once per obstacle for all rays at once).
//
// Standard slab method: intersect the ray against the box's x- and
// y-bounded strips and keep the overlap. A ray exactly parallel to an axis
// divides by zero here, which Go's float64 division defines the same way
// IEEE754/numpy does (+-Inf), so no special-casing is needed to match the
// Python original's `errstate(divide="ignore")`.
func raycastBox(x, y, dx, dy float64, b box, maxRange float64) float64 {
	tx1, tx2 := (b.xMin-x)/dx, (b.xMax-x)/dx
	ty1, ty2 := (b.yMin-y)/dy, (b.yMax-y)/dy

	// math.Min/math.Max carry NaN and signed-zero semantics that the Go
	// compiler cannot inline into a bare comparison, so each one is a real
	// call into math.archMin/archMax -- together ~24% of a sweep's runtime,
	// since this runs once per ray per obstacle. A NaN can only arise here
	// from 0/0, which needs the ray exactly parallel to an axis (dx or dy
	// exactly 0) AND the sensor exactly on that slab's plane. When both
	// components are non-zero every t is Inf-or-finite, comparisons order
	// them exactly as math.Min/Max would, and the branch form is equivalent.
	// The parallel-ray case keeps the original calls so the NaN path that
	// isFinite below depends on still behaves identically.
	var tNear, tFar float64
	if dx != 0 && dy != 0 {
		txNear, txFar := tx1, tx2
		if txNear > txFar {
			txNear, txFar = txFar, txNear
		}
		tyNear, tyFar := ty1, ty2
		if tyNear > tyFar {
			tyNear, tyFar = tyFar, tyNear
		}
		tNear, tFar = txNear, txFar
		if tyNear > tNear {
			tNear = tyNear
		}
		if tyFar < tFar {
			tFar = tyFar
		}
	} else {
		tNear = math.Max(math.Min(tx1, tx2), math.Min(ty1, ty2))
		tFar = math.Min(math.Max(tx1, tx2), math.Max(ty1, ty2))
	}

	// A hit needs the slabs to overlap and the exit point to be in front of
	// the sensor. Starting inside the box yields t_near < 0, reported as
	// range 0.
	// Computed once rather than twice, and by comparison for the same reason
	// as above. A NaN tNear lands on 0 here instead of propagating, which
	// changes nothing: isFinite is false in that case, so distance is never
	// the returned value.
	distance := tNear
	if !(distance > 0.0) {
		distance = 0.0
	}
	isFinite := !math.IsInf(tNear, 0) && !math.IsNaN(tNear)
	if isFinite && tFar >= distance {
		return distance
	}
	return maxRange
}
