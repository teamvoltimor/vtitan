package trackmodel

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"
)

// PathProjection is where a point sits relative to the planned path, in the
// path's own frame -- the frame rotates with the path, which is the whole
// point: a global-axis lateral component is only a cross-track error while
// the path runs along that axis.
type PathProjection struct {
	// X, Y is the closest point on the polyline.
	X, Y float64
	// DistanceM is the distance to the polyline itself, always
	// non-negative and clamped at the ends, so a point off the end of an
	// open path is correctly far away rather than merely off to one side
	// of the last segment's heading.
	DistanceM float64
	// SignedOffsetM is the lateral offset from the nearest segment's
	// LINE (not the clamped segment), positive to the left.
	SignedOffsetM float64
	// TangentRad is the heading of the path at the closest point.
	TangentRad float64
	// SegmentIndex is the polyline segment index the point projected
	// onto -- the handle for PathTurnAhead.
	SegmentIndex int
}

// minWaypointsForTurn is the fewest waypoints with a heading change to
// measure -- two waypoints are a single segment.
const minWaypointsForTurn = 3

// ProjectOntoPath projects (x, y) onto the waypoint polyline and returns
// the path frame -- the nearest point over every consecutive waypoint
// pair, not the nearest waypoint (which would overstate the offset by up
// to half the waypoint spacing).
func ProjectOntoPath(waypoints []Waypoint, x, y float64) PathProjection {
	var best *PathProjection
	bestDist := math.Inf(1)

	for i := range len(waypoints) - 1 {
		a, b := waypoints[i], waypoints[i+1]
		abx, aby := b.X-a.X, b.Y-a.Y
		segLenSq := abx*abx + aby*aby
		if segLenSq == 0.0 {
			continue
		}
		t := ((x-a.X)*abx + (y-a.Y)*aby) / segLenSq
		t = min(1.0, max(0.0, t))
		px, py := a.X+t*abx, a.Y+t*aby
		dist := Waypoint{X: x, Y: y}.DistanceTo(Waypoint{X: px, Y: py})
		if dist >= bestDist {
			continue
		}
		bestDist = dist
		best = &PathProjection{
			X:         px,
			Y:         py,
			DistanceM: dist,
			// Left-normal component about the segment's infinite line:
			// rotate the tangent +90deg and dot. Taken from the segment
			// start, not the clamped projection, which would read zero
			// for any point that projected past an end.
			SignedOffsetM: (-aby*(x-a.X) + abx*(y-a.Y)) / math.Sqrt(segLenSq),
			TangentRad:    math.Atan2(aby, abx),
			SegmentIndex:  i,
		}
	}

	if best != nil {
		return *best
	}

	// Fewer than two distinct waypoints: there is no tangent to define a
	// frame, so fall back to the nearest waypoint and leave the offset
	// unsigned. Keeps CrossTrackError meaningful on a degenerate path
	// rather than reporting a confident zero.
	if len(waypoints) == 0 {
		return PathProjection{X: x, Y: y, DistanceM: math.Inf(1), SignedOffsetM: math.Inf(1)}
	}
	nearest := waypoints[0]
	nearestDist := nearest.DistanceTo(Waypoint{X: x, Y: y})
	for _, w := range waypoints[1:] {
		if d := w.DistanceTo(Waypoint{X: x, Y: y}); d < nearestDist {
			nearest, nearestDist = w, d
		}
	}
	return PathProjection{
		X:             nearest.X,
		Y:             nearest.Y,
		DistanceM:     nearestDist,
		SignedOffsetM: nearestDist,
	}
}

// CrossTrackError is the perpendicular distance (meters) from (x, y) to the
// waypoint polyline -- unsigned; use ProjectOntoPath when the offsets are
// going to be differenced against each other.
func CrossTrackError(waypoints []Waypoint, x, y float64) float64 {
	return ProjectOntoPath(waypoints, x, y).DistanceM
}

// PathTurnAhead is the unsigned heading change the closed-loop path makes
// within previewDistanceM, measured forward from waypointIndex. Near zero
// along a straight and roughly previewDistance/arcRadius approaching a
// corner, so it signals a corner is coming before the robot has begun to
// fall behind one -- a distinction cross-track error cannot draw, since
// that only rises once the turn has already been missed.
//
// Walks the waypoint ring, so it reads correctly across the start/finish
// seam rather than reporting a straight for the last few waypoints of a
// lap. Returns 0 if the path is too short to measure one.
func PathTurnAhead(waypoints []Waypoint, waypointIndex int, previewDistanceM float64) float64 {
	count := len(waypoints)
	if count < minWaypointsForTurn || previewDistanceM <= 0.0 {
		return 0.0
	}

	start := ((waypointIndex % count) + count) % count
	var firstHeading, lastHeading *float64
	travelled := 0.0

	for offset := range count {
		a := waypoints[(start+offset)%count]
		b := waypoints[(start+offset+1)%count]
		segLen := a.DistanceTo(b)
		if segLen == 0.0 {
			continue
		}
		heading := a.BearingTo(b)
		if firstHeading == nil {
			firstHeading = &heading
		}
		lastHeading = &heading
		travelled += segLen
		if travelled >= previewDistanceM {
			break
		}
	}

	if firstHeading == nil || lastHeading == nil {
		return 0.0
	}
	return math.Abs(navutil.WrapAngle(*lastHeading - *firstHeading))
}
