package trackmodel

// Direction is the robot's travel direction around the WRO track loop,
// matching shared.domain.enums.Direction. Lives here alongside Section
// (rather than in internal/nav/directionestimator, which only infers a
// Direction, and internal/nav/waypoints, which only consumes one) since
// both packages need it and neither owns it -- mirroring how Python's
// shared.domain.enums groups Section/Direction/CorridorSide together as
// track-domain vocabulary, not estimator- or planner-specific types.
type Direction int

// CorridorSide is which of a corridor's two boundaries something is
// measured toward, matching shared.domain.enums.CorridorSide. Every
// corridor is bounded by the mat's outer wall on one side and a face of
// the inner block on the other, whichever cardinal section it is; naming
// the side rather than a compass direction keeps the meaning the same for
// all four.
type CorridorSide int

const (
	// Clockwise matches Direction.CLOCKWISE.
	Clockwise Direction = iota
	// Counterclockwise matches Direction.COUNTERCLOCKWISE.
	Counterclockwise
)

const (
	// Inner matches CorridorSide.INNER.
	Inner CorridorSide = iota
	// Outer matches CorridorSide.OUTER.
	Outer
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
