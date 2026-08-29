package waypoints

import (
	"fmt"
	"math"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// CornerRadii holds the arc radius per corner, matching segments.py's
// build_all_segments' corner_radii dict (keyed "se"/"sw"/"nw"/"ne" there).
// Sized per corner by CornerArcRadius, so the four can differ and each
// straight is trimmed by the radius of the corner at its own end rather
// than by one shared value.
type CornerRadii struct {
	SE, SW, NW, NE float64
}

// BuildAllSegments constructs per-corridor waypoint lists (straights +
// corner arcs), matching segments.py's build_all_segments. northCY/
// southCY/eastCX/westCX are the corridor centerlines, already
// bias-adjusted; direction CCW reverses each segment.
func BuildAllSegments(
	northCY, southCY, eastCX, westCX float64,
	radii CornerRadii,
	direction trackmodel.Direction,
	cfg Config,
) map[trackmodel.Section][]trackmodel.Waypoint {
	numIntermediate := cfg.NumIntermediateArcPoints
	straightCount := cfg.StraightWaypointCount

	// Corner arc ICR positions and arc angle ranges (CW direction).
	seICR := trackmodel.Waypoint{X: eastCX - radii.SE, Y: southCY + radii.SE}
	swICR := trackmodel.Waypoint{X: westCX + radii.SW, Y: southCY + radii.SW}
	nwICR := trackmodel.Waypoint{X: westCX + radii.NW, Y: northCY - radii.NW}
	neICR := trackmodel.Waypoint{X: eastCX - radii.NE, Y: northCY - radii.NE}

	seCW := ArcWithEndpoints(seICR, radii.SE, 0.0, -math.Pi/2, numIntermediate)
	swCW := ArcWithEndpoints(swICR, radii.SW, -math.Pi/2, -math.Pi, numIntermediate)
	nwCW := ArcWithEndpoints(nwICR, radii.NW, math.Pi, math.Pi/2, numIntermediate)
	neCW := ArcWithEndpoints(neICR, radii.NE, math.Pi/2, 0.0, numIntermediate)

	// CW straight segments. Each end is trimmed by the radius of the
	// corner it runs into, which is why the two bounds don't share a
	// value.
	eastStraight := StraightWaypoints(eastCX, true, northCY-radii.NE, southCY+radii.SE, straightCount)
	southStraight := StraightWaypoints(southCY, false, eastCX-radii.SE, westCX+radii.SW, straightCount)
	westStraight := StraightWaypoints(westCX, true, southCY+radii.SW, northCY-radii.NW, straightCount)
	northStraight := StraightWaypoints(northCY, false, westCX+radii.NW, eastCX-radii.NE, straightCount)

	if direction == trackmodel.Clockwise {
		return map[trackmodel.Section][]trackmodel.Waypoint{
			trackmodel.East:  append(eastStraight, seCW...),
			trackmodel.South: append(southStraight, swCW...),
			trackmodel.West:  append(westStraight, nwCW...),
			trackmodel.North: append(northStraight, neCW...),
		}
	}
	// Counter-clockwise: reverse each segment.
	return map[trackmodel.Section][]trackmodel.Waypoint{
		trackmodel.East:  append(reversed(eastStraight), reversed(neCW)...),
		trackmodel.South: append(reversed(southStraight), reversed(seCW)...),
		trackmodel.West:  append(reversed(westStraight), reversed(swCW)...),
		trackmodel.North: append(reversed(northStraight), reversed(nwCW)...),
	}
}

// reversed returns a new slice with waypoints in reverse order.
func reversed(waypoints []trackmodel.Waypoint) []trackmodel.Waypoint {
	out := make([]trackmodel.Waypoint, len(waypoints))
	for i, w := range waypoints {
		out[len(waypoints)-1-i] = w
	}
	return out
}

// AssembleLoop concatenates per-section waypoint lists in the given lap
// order into one loop, matching segments.py's assemble_loop.
func AssembleLoop(
	order []trackmodel.Section, segments map[trackmodel.Section][]trackmodel.Waypoint,
) []trackmodel.Waypoint {
	var loop []trackmodel.Waypoint
	for _, section := range order {
		loop = append(loop, segments[section]...)
	}
	return loop
}

// BuildWaypointSequence builds multi-lap waypoints starting from the
// closest point in the first segment, matching segments.py's
// build_waypoint_sequence.
func BuildWaypointSequence(
	fullLoop []trackmodel.Waypoint,
	segments map[trackmodel.Section][]trackmodel.Waypoint,
	order []trackmodel.Section,
	startX, startY float64,
	numLaps int,
	cfg Config,
) []trackmodel.Waypoint {
	firstSeg := segments[order[0]]
	startIndex := NearestWaypointIndex(firstSeg, startX, startY)

	waypoints := append([]trackmodel.Waypoint{}, firstSeg[startIndex:]...)
	waypoints = append(waypoints, AssembleLoop(order[1:], segments)...)

	for range numLaps - 1 {
		waypoints = append(waypoints, fullLoop...)
	}

	if startIndex > 0 {
		waypoints = append(waypoints, firstSeg[:startIndex]...)
	}

	return DeduplicateConsecutive(waypoints, cfg)
}

// NearestWaypointIndex returns the index of the waypoint closest to
// (x, y), matching segments.py's nearest_waypoint_index.
func NearestWaypointIndex(waypoints []trackmodel.Waypoint, x, y float64) int {
	best := 0
	bestDistSq := math.Inf(1)
	for i, w := range waypoints {
		dx, dy := w.X-x, w.Y-y
		if distSq := dx*dx + dy*dy; distSq < bestDistSq {
			best, bestDistSq = i, distSq
		}
	}
	return best
}

// ValidateBounds reports an error if generation produced an out-of-bounds
// waypoint, matching segments.py's validate_bounds -- every waypoint must
// stay on the track and clear of the restricted inner square.
func ValidateBounds(waypoints []trackmodel.Waypoint, minCoordM, maxCoordM, cornerMinM, cornerMaxM float64) error {
	for _, w := range waypoints {
		if w.X < minCoordM || w.X > maxCoordM || w.Y < minCoordM || w.Y > maxCoordM {
			return fmt.Errorf("waypoints: generated waypoint (%.3f, %.3f) falls outside the track bounds", w.X, w.Y)
		}
		if cornerMinM < w.X && w.X < cornerMaxM && cornerMinM < w.Y && w.Y < cornerMaxM {
			return fmt.Errorf(
				"waypoints: generated waypoint (%.3f, %.3f) falls inside the restricted inner square", w.X, w.Y,
			)
		}
	}
	return nil
}

// DeduplicateConsecutive removes consecutive duplicate waypoints (within
// cfg.DedupeDistanceM), matching segments.py's deduplicate_consecutive.
func DeduplicateConsecutive(waypoints []trackmodel.Waypoint, cfg Config) []trackmodel.Waypoint {
	if len(waypoints) == 0 {
		return nil
	}
	deduped := []trackmodel.Waypoint{waypoints[0]}
	for _, point := range waypoints[1:] {
		prev := deduped[len(deduped)-1]
		if math.Abs(point.X-prev.X) > cfg.DedupeDistanceM || math.Abs(point.Y-prev.Y) > cfg.DedupeDistanceM {
			deduped = append(deduped, point)
		}
	}
	return deduped
}
