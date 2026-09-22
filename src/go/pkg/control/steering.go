package control

import "math"

// SteeringNormFromAngleRad encodes a physical steering angle (radians,
// + = left) into a normalised command in [-1, 1], clamped rather than left
// to overshoot past the physical limit. It mirrors
// shared.domain.steering.angle_rad_to_steering_norm.
func SteeringNormFromAngleRad(angleRad, maxSteeringAngleRad float64) float64 {
	if maxSteeringAngleRad <= 0.0 {
		return 0.0
	}
	return math.Max(-SteeringNormLimit, math.Min(SteeringNormLimit, angleRad/maxSteeringAngleRad))
}
