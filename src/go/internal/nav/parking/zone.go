package parking

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// ParkZone is the parking lot rectangle (WRO: "the rectangle between the two
// markers"). This is the bay itself, not a tolerance box around it: bounded
// along the wall by the two fins' inner faces, and in depth by the outer wall
// and the fins' inner ends. The robot's whole projection has to fit inside it,
// so it is deliberately the true lot outline with no slack added -- containment
// margin belongs in the stop check, not here.
type ParkZone struct {
	XMin float64
	YMin float64
	XMax float64
	YMax float64
	// TargetYaw is the expected robot yaw when parked (radians), parallel to
	// the outer wall.
	TargetYaw float64
	// GapCX/GapCY is the center of the bay (world x/y).
	GapCX float64
	GapCY float64
	// WallIsX reports whether the field wall backing this lot runs along x
	// (the E/W sections) rather than along y.
	WallIsX bool
	// WallCoord is the wall's coordinate on the axis normal to it.
	WallCoord float64
}

// Project splits a world point into (along-wall, depth) coordinates for this
// lot. Which world axis plays which role flips between the N/S and E/W
// corridors, and getting it backwards silently swaps the bay's 0.43 m mouth
// for its 0.20 m depth. Both guards below need the same split, so it is
// derived once here rather than re-spelled at each use.
func (z ParkZone) Project(x, y float64) (along, depth float64) {
	if z.WallIsX {
		return y, x
	}
	return x, y
}

// BoundsAlong returns the lot's extent along the wall -- i.e. between the two
// fins' inner faces.
func (z ParkZone) BoundsAlong() (lo, hi float64) {
	if z.WallIsX {
		return z.YMin, z.YMax
	}
	return z.XMin, z.XMax
}

// BoundsDepth returns the lot's extent out from the wall -- i.e. the depth the
// fins span.
func (z ParkZone) BoundsDepth() (lo, hi float64) {
	if z.WallIsX {
		return z.XMin, z.XMax
	}
	return z.YMin, z.YMax
}

// BuildZone computes the parking lot rectangle and the wall-parallel target
// yaw. The markers are fins perpendicular to the outer wall: ParkingLotSpecs.
// WIDTH (20 mm) thick along the wall, ParkingLotSpecs.LENGTH (200 mm) deep out
// from it. So the lot spans, along the wall, between the fins' inner faces, and
// in depth from the wall out to the fins' inner ends. TargetYaw is parallel to
// the outer wall -- the lot is only as deep as the chassis is wide, so a
// nose-in pose cannot fit and is not what the rule asks for. Of the two
// parallel headings, the one matching direction of travel is chosen, so the
// robot never has to turn around inside a bay with no room to do it.
func BuildZone(
	b1, b2 BlockPosition,
	section trackmodel.Section,
	direction trackmodel.Direction,
	specs ParkingLotSpecs,
	dims TrackDimensions,
) ParkZone {
	halfFinThickness := specs.Width / 2
	cw := direction == trackmodel.Clockwise

	var xMin, xMax, yMin, yMax, targetYaw float64
	switch section {
	case trackmodel.South:
		x1, x2 := min(b1.X, b2.X), max(b1.X, b2.X)
		xMin = x1 + halfFinThickness
		xMax = x2 - halfFinThickness
		yMin, yMax = dims.MinCoord, dims.MinCoord+specs.Length
		if cw {
			targetYaw = math.Pi
		} else {
			targetYaw = 0.0
		}
	case trackmodel.North:
		x1, x2 := min(b1.X, b2.X), max(b1.X, b2.X)
		xMin = x1 + halfFinThickness
		xMax = x2 - halfFinThickness
		yMin, yMax = dims.MaxCoord-specs.Length, dims.MaxCoord
		if cw {
			targetYaw = 0.0
		} else {
			targetYaw = math.Pi
		}
	case trackmodel.East:
		y1, y2 := min(b1.Y, b2.Y), max(b1.Y, b2.Y)
		yMin = y1 + halfFinThickness
		yMax = y2 - halfFinThickness
		xMin, xMax = dims.MaxCoord-specs.Length, dims.MaxCoord
		if cw {
			targetYaw = math.Pi / 2
		} else {
			targetYaw = -math.Pi / 2
		}
	case trackmodel.West:
		y1, y2 := min(b1.Y, b2.Y), max(b1.Y, b2.Y)
		yMin = y1 + halfFinThickness
		yMax = y2 - halfFinThickness
		xMin, xMax = dims.MinCoord, dims.MinCoord+specs.Length
		if cw {
			targetYaw = -math.Pi / 2
		} else {
			targetYaw = math.Pi / 2
		}
	}

	wallIsX := section == trackmodel.East || section == trackmodel.West
	var wallCoord float64
	switch section {
	case trackmodel.East:
		wallCoord = xMax
	case trackmodel.West:
		wallCoord = xMin
	case trackmodel.North:
		wallCoord = yMax
	default: // South
		wallCoord = yMin
	}

	return ParkZone{
		XMin:      xMin,
		XMax:      xMax,
		YMin:      yMin,
		YMax:      yMax,
		TargetYaw: targetYaw,
		GapCX:     (xMin + xMax) / 2,
		GapCY:     (yMin + yMax) / 2,
		WallIsX:   wallIsX,
		WallCoord: wallCoord,
	}
}

// StagingPos returns the position directly in front of the gap opening, on the
// track side, offset by clearance from the bay mouth.
func StagingPos(zone ParkZone, section trackmodel.Section, clearance float64) trackmodel.Waypoint {
	switch section {
	case trackmodel.South:
		return trackmodel.Waypoint{X: zone.GapCX, Y: zone.YMax + clearance}
	case trackmodel.North:
		return trackmodel.Waypoint{X: zone.GapCX, Y: zone.YMin - clearance}
	case trackmodel.East:
		return trackmodel.Waypoint{X: zone.XMin - clearance, Y: zone.GapCY}
	default: // West
		return trackmodel.Waypoint{X: zone.XMax + clearance, Y: zone.GapCY}
	}
}
