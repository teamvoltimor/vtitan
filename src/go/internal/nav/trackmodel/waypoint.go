package trackmodel

import "math"

// Waypoint is a pure XY point to navigate towards, with no orientation --
// matches shared.domain.models.Waypoint.
type Waypoint struct {
	X, Y float64
}

// DistanceTo returns the Euclidean distance to other.
func (w Waypoint) DistanceTo(other Waypoint) float64 {
	return math.Hypot(w.X-other.X, w.Y-other.Y)
}

// BearingTo returns the bearing (radians, 0 = forward/+pi/2 = left) from w
// to other.
func (w Waypoint) BearingTo(other Waypoint) float64 {
	return math.Atan2(other.Y-w.Y, other.X-w.X)
}
