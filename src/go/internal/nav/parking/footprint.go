package parking

import "math"

// chassisCornerCount is the number of chassis footprint corners.
const chassisCornerCount = 4

// ChassisCorners returns the four corners of the chassis footprint at this
// pose (world frame).
func ChassisCorners(rx, ry, robotYaw, chassisLength, chassisWidth float64) [][2]float64 {
	halfL, halfW := chassisLength/2, chassisWidth/2
	cosYaw, sinYaw := math.Cos(robotYaw), math.Sin(robotYaw)
	corners := [][2]float64{
		{halfL, halfW},
		{halfL, -halfW},
		{-halfL, -halfW},
		{-halfL, halfW},
	}
	out := make([][2]float64, 0, chassisCornerCount)
	for _, c := range corners {
		dx, dy := c[0], c[1]
		out = append(out, [2]float64{
			rx + dx*cosYaw - dy*sinYaw,
			ry + dx*sinYaw + dy*cosYaw,
		})
	}
	return out
}

// IsBeyondLotcenter reports whether coord lies on the wall side of the lot's
// midline.
func IsBeyondLotcenter(coord float64, zone ParkZone) bool {
	center := zone.GapCX
	if zone.WallIsX {
		center = zone.GapCY
	}
	if zone.WallCoord > center {
		return coord > center
	}
	return coord < center
}

// FootprintInside reports whether the robot's whole projection on the mat lies
// inside the parking lot. This is the rule as written ("the projection of the
// robot on the mat is fully inside the rectangle between the two markers"), not
// the center-of-chassis approximation it replaces. The difference is not
// cosmetic: a center-in-box test reports a successful park for a robot sitting
// mostly in the corridor, or with its nose through the outer wall, because
// neither the footprint nor the heading enters into it.
func FootprintInside(
	rx, ry, robotYaw float64,
	zone ParkZone,
	chassisLength, chassisWidth float64,
) bool {
	corners := ChassisCorners(rx, ry, robotYaw, chassisLength, chassisWidth)
	for _, c := range corners {
		if c[0] < zone.XMin || c[0] > zone.XMax || c[1] < zone.YMin || c[1] > zone.YMax {
			return false
		}
	}
	return true
}

// FootprintBreachesWall reports whether any chassis corner has come within the
// wall standoff of the field wall. ENTER pure-pursues the lot center, which
// controls position but not heading, so a robot that arrives across the lot
// rather than along it drives its nose at the wall and keeps going. With the
// stop condition corrected to the actual rule, nothing stops it any more -- so
// the maneuver gives up here instead of pushing into the wall. Not colliding
// takes priority over completing the park.
func FootprintBreachesWall(rx, ry, robotYaw float64, zone ParkZone, cfg Config) bool {
	corners := ChassisCorners(rx, ry, robotYaw, cfg.ChassisLengthM, cfg.ChassisWidthM)
	for _, c := range corners {
		coord := c[1]
		if zone.WallIsX {
			coord = c[0]
		}
		if math.Abs(coord-zone.WallCoord) < cfg.WallStandoffM && IsBeyondLotcenter(coord, zone) {
			return true
		}
	}
	return false
}

// FootprintBreachesMarkers reports whether any chassis corner has come within
// the marker standoff of a fin. The wall guard alone used to be sufficient by
// accident: with the steering limit modeled at 30 deg the chassis could not
// turn tightly enough to swing a corner into a fin before the wall stopped it.
// At the real ~70 deg lock (R_min 0.034 m rather than 0.165 m) ENTER's pure
// pursuit of the lot center turns hard enough to reach them, so the fins need
// the same explicit give-up the wall has. Same priority as there: not
// colliding beats parking. A fin flanks the lot along the wall and spans its
// full depth, so a corner is in fin territory when it lies within the lot's
// depth band and at or past a fin's inner face.
func FootprintBreachesMarkers(rx, ry, robotYaw float64, zone ParkZone, cfg Config) bool {
	corners := ChassisCorners(rx, ry, robotYaw, cfg.ChassisLengthM, cfg.ChassisWidthM)
	depthMin, depthMax := zone.BoundsDepth()
	alongMin, alongMax := zone.BoundsAlong()
	for _, c := range corners {
		along, depth := zone.Project(c[0], c[1])
		if depth < depthMin-cfg.MarkerStandoffM || depth > depthMax+cfg.MarkerStandoffM {
			continue // out in the corridor, past the fins' ends -- nothing to hit
		}
		if along <= alongMin+cfg.MarkerStandoffM || along >= alongMax-cfg.MarkerStandoffM {
			return true
		}
	}
	return false
}
