package trackmodel

import "math"

// Pose is the robot's position and orientation in world space, matching
// shared.domain.models.Pose. Use it for anything that carries or could
// carry a heading (robot state, localizer priors, escape/collision points
// needing orientation); for a pure XY point with no orientation, use
// Waypoint instead.
type Pose struct {
	X, Y, Yaw float64
}

// DistanceTo returns the Euclidean distance from p to other (position only),
// matching Pose.distance_to.
func (p Pose) DistanceTo(other Waypoint) float64 {
	return math.Hypot(p.X-other.X, p.Y-other.Y)
}

// ToLocalFrame rotates target into p's local frame (x forward, y left),
// matching Pose.to_local_frame.
func (p Pose) ToLocalFrame(target Waypoint) (xLocal, yLocal float64) {
	dx := target.X - p.X
	dy := target.Y - p.Y
	cosYaw, sinYaw := math.Cos(p.Yaw), math.Sin(p.Yaw)
	return dx*cosYaw + dy*sinYaw, -dx*sinYaw + dy*cosYaw
}

// ToWaypoint drops the heading, returning a pure XY Waypoint, matching
// Pose.to_waypoint.
func (p Pose) ToWaypoint() Waypoint {
	return Waypoint{X: p.X, Y: p.Y}
}
