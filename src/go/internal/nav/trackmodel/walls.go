package trackmodel

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"
)

// segment is an axis-aligned wall face as a line segment (for LIDAR
// raycasting).
type segment struct {
	x1, y1, x2, y2 float64
}

// TrackWalls holds the axis-aligned wall segments for one Open Challenge
// layout, and raycasts against them.
type TrackWalls struct {
	InnerBlock InnerBlock

	minCoord, maxCoord float64
	segments           []segment
}

// NewTrackWalls builds the wall layout from geometry. minCoord/maxCoord are
// the track's outer boundary (profile.TrackConfig.Track.MinCoord/MaxCoord).
func NewTrackWalls(geometry CorridorGeometry, minCoord, maxCoord float64) *TrackWalls {
	inner := geometry.InnerBlock
	return &TrackWalls{
		InnerBlock: inner,
		minCoord:   minCoord,
		maxCoord:   maxCoord,
		segments: []segment{
			// Outer boundary (inner faces of the exterior walls).
			{minCoord, minCoord, maxCoord, minCoord}, // south
			{minCoord, maxCoord, maxCoord, maxCoord}, // north
			{minCoord, minCoord, minCoord, maxCoord}, // west
			{maxCoord, minCoord, maxCoord, maxCoord}, // east
			// Inner block (outer faces of the interior walls).
			{inner.XMin, inner.YMin, inner.XMax, inner.YMin}, // south
			{inner.XMin, inner.YMax, inner.XMax, inner.YMax}, // north
			{inner.XMin, inner.YMin, inner.XMin, inner.YMax}, // west
			{inner.XMax, inner.YMin, inner.XMax, inner.YMax}, // east
		},
	}
}

// RayFan caches the unit vectors of a fixed bearing fan in the ROBOT frame.
//
// A scan's bearings never change between ticks; only the yaw they are cast at
// does. Evaluating cos/sin per ray per scan therefore recomputes 2*len(angles)
// transcendentals for a fan that is constant, which profiling put at ~22% of a
// corpus sweep. Rotating a cached robot-frame unit vector into the world frame
// costs four multiplies and two adds against two cos/sin calls per ray, so a
// scan needs exactly two transcendentals regardless of how many rays it has.
type RayFan struct {
	angles   []float64
	cos, sin []float64
}

// NewRayFan precomputes the unit vectors for anglesRobot. The slice is copied,
// so a caller may reuse or mutate its own buffer afterwards.
func NewRayFan(anglesRobot []float64) *RayFan {
	f := &RayFan{
		angles: make([]float64, len(anglesRobot)),
		cos:    make([]float64, len(anglesRobot)),
		sin:    make([]float64, len(anglesRobot)),
	}
	copy(f.angles, anglesRobot)
	for i, a := range anglesRobot {
		f.cos[i], f.sin[i] = math.Cos(a), math.Sin(a)
	}
	return f
}

// Len is the number of rays in the fan.
func (f *RayFan) Len() int {
	return len(f.angles)
}

// Direction rotates ray i from the robot frame into the world frame, given the
// cos/sin of the body yaw. The caller computes those once per scan and passes
// them in, which is the whole point of the fan: two transcendentals per scan
// rather than two per ray.
func (f *RayFan) Direction(i int, cosYaw, sinYaw float64) (dx, dy float64) {
	return cosYaw*f.cos[i] - sinYaw*f.sin[i], sinYaw*f.cos[i] + cosYaw*f.sin[i]
}

// Matches reports whether this fan was built from exactly these bearings, so a
// caller handed a scan per tick can reuse a cached fan instead of rebuilding
// it, and stays correct if the bearings ever do change.
func (f *RayFan) Matches(anglesRobot []float64) bool {
	if len(f.angles) != len(anglesRobot) {
		return false
	}
	for i, a := range anglesRobot {
		if f.angles[i] != a {
			return false
		}
	}
	return true
}

// Raycast casts a fan of rays from (x, y) at heading yaw+anglesRobot[i] and
// returns the nearest wall range per ray, clamped to
// [lidarMinRangeM, lidarMaxRangeM]. A ray that hits nothing returns
// lidarMaxRangeM.
//
// Allocates both the fan and the result; a hot caller holding a fan across
// ticks should use RaycastFan instead.
func (w *TrackWalls) Raycast(
	x, y, yaw float64, anglesRobot []float64, lidarMinRangeM, lidarMaxRangeM float64,
) []float64 {
	return w.RaycastFan(x, y, yaw, NewRayFan(anglesRobot), lidarMinRangeM, lidarMaxRangeM, nil)
}

// RaycastFan is Raycast against a precomputed fan, writing into out (which may
// be nil, or any slice with at least fan.Len() capacity) and returning it. The
// result aliases out, so a caller that retains the ranges past the next call
// must pass a fresh slice or copy them.
func (w *TrackWalls) RaycastFan(
	x, y, yaw float64, fan *RayFan, lidarMinRangeM, lidarMaxRangeM float64, out []float64,
) []float64 {
	ranges := out[:0]
	if cap(ranges) < fan.Len() {
		ranges = make([]float64, fan.Len())
	}
	ranges = ranges[:fan.Len()]

	cosYaw, sinYaw := math.Cos(yaw), math.Sin(yaw)
	for i := range fan.cos {
		// Angle-sum identity for cos/sin(yaw + angles[i]).
		dx := cosYaw*fan.cos[i] - sinYaw*fan.sin[i]
		dy := sinYaw*fan.cos[i] + cosYaw*fan.sin[i]

		best := math.Inf(1)
		for _, s := range w.segments {
			ex, ey := s.x2-s.x1, s.y2-s.y1
			denom := dx*ey - dy*ex
			if denom == 0.0 {
				continue // ray parallel to this segment
			}
			rx, ry := s.x1-x, s.y1-y
			t := (rx*ey - ry*ex) / denom // distance along the ray
			u := (rx*dy - ry*dx) / denom // parameter along the segment
			if t >= 0.0 && u >= 0.0 && u <= 1.0 && t < best {
				best = t
			}
		}

		ranges[i] = navutil.Clamp(best, lidarMinRangeM, lidarMaxRangeM)
	}
	return ranges
}

// MinCoord is the track's outer boundary minimum (m), on both axes.
func (w *TrackWalls) MinCoord() float64 {
	return w.minCoord
}

// MaxCoord is the track's outer boundary maximum (m), on both axes.
func (w *TrackWalls) MaxCoord() float64 {
	return w.maxCoord
}

// PointInFreeSpace reports whether (x, y) is in the navigable ring with
// clearance margin: inside the track's outer boundary and outside the
// inner block.
func (w *TrackWalls) PointInFreeSpace(x, y, clearance float64) bool {
	if x < w.minCoord+clearance || x > w.maxCoord-clearance ||
		y < w.minCoord+clearance || y > w.maxCoord-clearance {
		return false
	}
	iv := w.InnerBlock
	inBlock := iv.XMin+clearance < x && x < iv.XMax-clearance && iv.YMin+clearance < y &&
		y < iv.YMax-clearance
	return !inBlock
}
