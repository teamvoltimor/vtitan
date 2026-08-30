package trackmodel

import "math"

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

// Raycast casts a fan of rays from (x, y) at heading yaw+anglesRobot[i] and
// returns the nearest wall range per ray, clamped to
// [lidarMinRangeM, lidarMaxRangeM]. A ray that hits nothing returns
// lidarMaxRangeM.
func (w *TrackWalls) Raycast(
	x, y, yaw float64, anglesRobot []float64, lidarMinRangeM, lidarMaxRangeM float64,
) []float64 {
	ranges := make([]float64, len(anglesRobot))
	for i, angleRobot := range anglesRobot {
		worldAngle := yaw + angleRobot
		dx, dy := math.Cos(worldAngle), math.Sin(worldAngle)

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

		ranges[i] = clamp(best, lidarMinRangeM, lidarMaxRangeM)
	}
	return ranges
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

func clamp(value, lo, hi float64) float64 {
	return math.Max(lo, math.Min(hi, value))
}
