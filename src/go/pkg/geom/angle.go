package geom

import "math"

// WrapAngle wraps angle to [-pi, pi], matching utils.wrap_angle.
func WrapAngle(angle float64) float64 {
	return math.Atan2(math.Sin(angle), math.Cos(angle))
}
