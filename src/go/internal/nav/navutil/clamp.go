package navutil

import "math"

// Clamp restricts value to the closed interval [lo, hi], matching
// utils.clamp.
func Clamp(value, lo, hi float64) float64 {
	return max(lo, min(hi, value))
}

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
