package directionestimator

// Direction is the robot's travel direction around the WRO track loop,
// mirroring shared.domain.enums.Direction.
type Direction int

const (
	// Clockwise matches Direction.CLOCKWISE.
	Clockwise Direction = iota
	// Counterclockwise matches Direction.COUNTERCLOCKWISE.
	Counterclockwise
)

// String returns a short label for logging.
func (d Direction) String() string {
	switch d {
	case Clockwise:
		return "clockwise"
	case Counterclockwise:
		return "counterclockwise"
	default:
		return "unknown"
	}
}
