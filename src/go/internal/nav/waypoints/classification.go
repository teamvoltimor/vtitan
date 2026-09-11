package waypoints

import "github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"

// CorridorForPosition classifies which corridor section the robot is
// currently in, matching classification.py's corridor_for_position.
//
// Uses the fixed inner-square boundaries (cornerMinM/cornerMaxM --
// profile.TrackConfig.Track.CornerMin/CornerMax) to assign a cardinal
// section. In corner zones (both x and y outside the inner square range
// simultaneously), the nearest boundary face determines the section, with
// ties resolving South, North, East, West in that order (matching the
// Python original's dict-insertion-order tie-break).
func CorridorForPosition(x, y, cornerMinM, cornerMaxM float64) trackmodel.Section {
	inX := cornerMinM <= x && x <= cornerMaxM
	inY := cornerMinM <= y && y <= cornerMaxM

	switch {
	case y < cornerMinM && inX:
		return trackmodel.South
	case y > cornerMaxM && inX:
		return trackmodel.North
	case x > cornerMaxM && inY:
		return trackmodel.East
	case x < cornerMinM && inY:
		return trackmodel.West
	}

	// Corner: classify by nearest inner-boundary face, S/N/E/W tie-break
	// order matching the Python dict's insertion order.
	best := trackmodel.South
	bestDist := abs(y - cornerMinM)
	if d := abs(y - cornerMaxM); d < bestDist {
		best, bestDist = trackmodel.North, d
	}
	if d := abs(x - cornerMaxM); d < bestDist {
		best, bestDist = trackmodel.East, d
	}
	if d := abs(x - cornerMinM); d < bestDist {
		best = trackmodel.West
	}
	return best
}

func abs(v float64) float64 {
	if v < 0 {
		return -v
	}
	return v
}
