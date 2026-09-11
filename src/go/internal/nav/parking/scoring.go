package parking

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"
)

// FullParkPoints is WRO 1.8.2 -- completely in the parking area AND parallel
// to the wall.
const FullParkPoints = 15

// PartialParkPoints is WRO 1.8.3 -- partly in the parking area, OR in it but
// not parallel.
const PartialParkPoints = 7

// scoringStandoffM is the contact standoff used for SCORING, not the
// controller's give-up margin. FootprintBreachesWall/Markers default to a
// safety standoff so the maneuver abandons before it touches; scoring
// through that margin would call a legal park a breach and hide exactly the
// near-wall poses partial credit depends on.
const scoringStandoffM = 0.0

// ParkScore is what a judge would award for a final pose, and the reasons
// behind it.
type ParkScore struct {
	Points int
	// Contained reports whether the whole footprint is inside the lot
	// rectangle (the 1.8.2 half of full credit).
	Contained bool
	// Parallel reports whether the wheel-to-wall difference is within
	// tolerance (the other 1.8.2 half).
	Parallel bool
	// Overlaps reports whether any part of the footprint is inside the lot
	// -- the 1.8.3 criterion.
	Overlaps bool
	// Touched reports contact with a fin or the wall. Vetoes both tiers
	// regardless of pose.
	Touched bool
}

// FootprintOverlapsLot reports whether ANY part of the chassis projection
// lies inside the parking lot. The partial-credit counterpart to
// FootprintInside. WRO scores parking in two tiers -- 15 points for
// "completely in the parking area and parallel" (1.8.2) and 7 for "parking
// partly or not parallel" (1.8.3).
//
// A true rectangle-rectangle overlap, not a corner-in-box test: at the
// headings that matter here the chassis can straddle the lot mouth with no
// corner of either rectangle inside the other, which a corner test reports
// as "outside". Separating-axis over both rectangles' edge normals, in the
// zone's (along, depth) frame where the lot is axis-aligned.
func FootprintOverlapsLot(
	rx, ry, robotYaw float64,
	zone ParkZone,
	chassisLength, chassisWidth float64,
) bool {
	alongMin, alongMax := zone.BoundsAlong()
	depthMin, depthMax := zone.BoundsDepth()
	corners := ChassisCorners(rx, ry, robotYaw, chassisLength, chassisWidth)
	chassis := make([][2]float64, len(corners))
	for i, c := range corners {
		along, depth := zone.Project(c[0], c[1])
		chassis[i] = [2]float64{along, depth}
	}
	lot := [][2]float64{
		{alongMin, depthMin},
		{alongMax, depthMin},
		{alongMax, depthMax},
		{alongMin, depthMax},
	}
	// The lot's own normals are the frame axes; the chassis contributes two
	// more. A gap on ANY axis separates the rectangles, so overlap needs all
	// four to overlap. Two edges suffice per rectangle -- opposite edges
	// share a normal.
	axes := [][2]float64{{1.0, 0.0}, {0.0, 1.0}}
	for _, i := range [2]int{1, 2} {
		prev := chassis[i-1]
		cur := chassis[i]
		axes = append(axes, [2]float64{cur[1] - prev[1], prev[0] - cur[0]})
	}
	for _, ax := range axes {
		axx, axy := ax[0], ax[1]
		cLo, cHi := projectExtent(chassis, axx, axy)
		lLo, lHi := projectExtent(lot, axx, axy)
		if cHi <= lLo || lHi <= cLo {
			return false
		}
	}
	return true
}

func projectExtent(points [][2]float64, axx, axy float64) (lo, hi float64) {
	lo, hi = math.Inf(1), math.Inf(-1)
	for _, p := range points {
		v := p[0]*axx + p[1]*axy
		if v < lo {
			lo = v
		}
		if v > hi {
			hi = v
		}
	}
	return lo, hi
}

// IsWallParallel reports whether the heading satisfies the rule's parallel
// test. The rule is about geometry, not travel: "the distances between the
// two wheels on one side and the wall do not differ by more than 2 cm",
// which is a wheelbase-scaled heading tolerance and is satisfied by BOTH
// headings along the wall. zone.TargetYaw names only one of them -- the one
// matching the direction of travel -- so comparing against it alone would
// score a robot parked perfectly but facing the other way as not parallel.
// The controller is right to aim at one; the judge does not care which.
func IsWallParallel(robotYaw float64, zone ParkZone, yawTolerance float64) bool {
	yawErr := math.Abs(navutil.WrapAngle(robotYaw - zone.TargetYaw))
	return math.Min(yawErr, math.Pi-yawErr) <= yawTolerance
}

// ScorePark awards 15, 7, or 0 for a final pose, per the 2026 scoring table.
// Contact is checked FIRST and short-circuits: it is a veto, not a
// deduction. Ruled 2026-09-03: touching the parking lot limitations stops
// the robot and voids ALL parking points, so a run that grinds its way to a
// perfect pose scores ZERO.
func ScorePark(rx, ry, robotYaw float64, zone ParkZone, cfg Config) ParkScore {
	scoringCfg := cfg
	scoringCfg.WallStandoffM = scoringStandoffM
	scoringCfg.MarkerStandoffM = scoringStandoffM

	touched := FootprintBreachesMarkers(rx, ry, robotYaw, zone, scoringCfg) ||
		FootprintBreachesWall(rx, ry, robotYaw, zone, scoringCfg)
	contained := FootprintInside(rx, ry, robotYaw, zone, cfg.ChassisLengthM, cfg.ChassisWidthM)
	parallel := IsWallParallel(robotYaw, zone, cfg.YawTolerance)
	overlaps := contained || FootprintOverlapsLot(rx, ry, robotYaw, zone, cfg.ChassisLengthM, cfg.ChassisWidthM)

	var points int
	switch {
	case touched:
		points = 0
	case contained && parallel:
		points = FullParkPoints
	case overlaps:
		points = PartialParkPoints
	default:
		points = 0
	}
	return ParkScore{
		Points:    points,
		Contained: contained,
		Parallel:  parallel,
		Overlaps:  overlaps,
		Touched:   touched,
	}
}
