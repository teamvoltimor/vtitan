package navutil

import "math"

// quarterTurnRad is a quarter turn (90deg) in radians -- the WRO track is a
// Manhattan world, so "aligned with a corridor" means "within tolerance of
// a multiple of 90deg" regardless of which corridor.
const quarterTurnRad = math.Pi / 2

// WrapAngle wraps angle to [-pi, pi], matching utils.wrap_angle.
func WrapAngle(angle float64) float64 {
	return math.Atan2(math.Sin(angle), math.Cos(angle))
}

// AxisOffsetRad is the signed deviation of yaw from the nearest track axis:
// positive is left of it, negative is right, matching utils.axis_offset_rad.
func AxisOffsetRad(yaw float64) float64 {
	return WrapAngle(yaw - math.Round(yaw/quarterTurnRad)*quarterTurnRad)
}

// AxisErrorRad is how far yaw sits from the nearest track axis, always
// non-negative, matching utils.axis_error_rad.
func AxisErrorRad(yaw float64) float64 {
	return math.Abs(AxisOffsetRad(yaw))
}
