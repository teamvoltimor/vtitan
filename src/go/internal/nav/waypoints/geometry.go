package waypoints

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// roundMM rounds v to millimeter precision (3 decimal places), matching
// every Waypoint constructor in the Python original's round(x, 3).
func roundMM(v float64) float64 {
	const places = 1000.0
	return math.Round(v*places) / places
}

// CornerArcRadius is the largest arc radius at one corner that costs no
// clearance to the inner block, matching geometry.py's corner_arc_radius.
//
// A corner arc is tangent to both corridor centerlines, so its center sits
// at (radius, radius) in from their intersection. The path's distance to
// the inner block corner is therefore set by the radius, and stays at the
// value the straights already have (width/2 - centerBiasM) right up until
// the radius passes that value, after which the arc bulges past the
// centerline and starts eating margin -- so the largest such radius is
// strictly best. Takes the max of the two corridors, not the min: the
// radius has to reach the wider corridor's centerline to be tangent to
// it, and forcing it down to the narrower side pulls the arc off that
// tangent and INTO the corner.
func CornerArcRadius(widthEntryM, widthExitM, centerBiasM, maxRadius float64) float64 {
	return min(maxRadius, max(widthEntryM, widthExitM)/2-centerBiasM)
}

// ArcIntermediatePoints samples count evenly-spaced interior arc points
// (excluding endpoints), matching geometry.py's arc_intermediate_points.
func ArcIntermediatePoints(
	cx, cy, radius, thetaStart, thetaEnd float64,
	count int,
) []trackmodel.Waypoint {
	points := make([]trackmodel.Waypoint, 0, count)
	for step := 1; step <= count; step++ {
		fraction := float64(step) / float64(count+1)
		theta := thetaStart + fraction*(thetaEnd-thetaStart)
		points = append(points, trackmodel.Waypoint{
			X: roundMM(cx + radius*math.Cos(theta)),
			Y: roundMM(cy + radius*math.Sin(theta)),
		})
	}
	return points
}

// ArcWithEndpoints generates arc points including entry and exit, with
// numIntermediate interior samples, matching geometry.py's
// arc_with_endpoints.
func ArcWithEndpoints(
	center trackmodel.Waypoint, radius, thetaStart, thetaEnd float64, numIntermediate int,
) []trackmodel.Waypoint {
	entry := trackmodel.Waypoint{
		X: roundMM(center.X + radius*math.Cos(thetaStart)),
		Y: roundMM(center.Y + radius*math.Sin(thetaStart)),
	}
	exitPt := trackmodel.Waypoint{
		X: roundMM(center.X + radius*math.Cos(thetaEnd)),
		Y: roundMM(center.Y + radius*math.Sin(thetaEnd)),
	}
	intermediates := ArcIntermediatePoints(
		center.X,
		center.Y,
		radius,
		thetaStart,
		thetaEnd,
		numIntermediate,
	)

	points := make([]trackmodel.Waypoint, 0, len(intermediates)+2)
	points = append(points, entry)
	points = append(points, intermediates...)
	points = append(points, exitPt)
	return points
}

// StraightWaypoints generates count evenly-spaced waypoints along one axis
// of a corridor centerline, matching geometry.py's straight_waypoints.
// isX selects which axis fixedCoord fixes: true for X (East/West
// corridors), false for Y (North/South corridors).
func StraightWaypoints(
	fixedCoord float64,
	isX bool,
	start, end float64,
	count int,
) []trackmodel.Waypoint {
	points := make([]trackmodel.Waypoint, 0, count)
	for step := range count {
		fraction := 0.5
		if count > 1 {
			fraction = float64(step) / float64(count-1)
		}
		varying := start + fraction*(end-start)
		if isX {
			points = append(
				points,
				trackmodel.Waypoint{X: roundMM(fixedCoord), Y: roundMM(varying)},
			)
		} else {
			points = append(points, trackmodel.Waypoint{X: roundMM(varying), Y: roundMM(fixedCoord)})
		}
	}
	return points
}
