package racetracker

import "github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"

// TravelNormal is the unit vector pointing the way the robot travels along a
// given corridor, matching shared.domain.models.TravelNormal.
type TravelNormal struct {
	NX float64
	NY float64
}

// TravelNormalFor returns the travel direction unit vector for a
// (section, direction) pair, matching race_tracker.py's TRAVEL_DIRS.
//
// LapDetector uses it as the finish-line normal, and
// corridor_estimator.section_from_heading relies on a property worth stating:
// for a FIXED travel direction all four vectors are distinct, so a heading
// alone identifies the corridor outright.
//
// A switch rather than a map, matching this codebase's preference: the pairs
// are a closed, compile-time-known set, and ok=false makes the unknown case
// explicit where Python's dict lookup would raise KeyError.
func TravelNormalFor(
	section trackmodel.Section,
	direction trackmodel.Direction,
) (TravelNormal, bool) {
	switch direction {
	case trackmodel.Clockwise:
		switch section {
		case trackmodel.South:
			return TravelNormal{NX: -1.0, NY: 0.0}, true
		case trackmodel.North:
			return TravelNormal{NX: 1.0, NY: 0.0}, true
		case trackmodel.East:
			return TravelNormal{NX: 0.0, NY: -1.0}, true
		case trackmodel.West:
			return TravelNormal{NX: 0.0, NY: 1.0}, true
		}
	case trackmodel.Counterclockwise:
		switch section {
		case trackmodel.South:
			return TravelNormal{NX: 1.0, NY: 0.0}, true
		case trackmodel.North:
			return TravelNormal{NX: -1.0, NY: 0.0}, true
		case trackmodel.East:
			return TravelNormal{NX: 0.0, NY: 1.0}, true
		case trackmodel.West:
			return TravelNormal{NX: 0.0, NY: -1.0}, true
		}
	}
	return TravelNormal{}, false
}
