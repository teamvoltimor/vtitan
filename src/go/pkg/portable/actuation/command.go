package actuation

import "math"

// FiniteCommand reports whether a command's speed and steering angle are
// both finite. A command that is not must be rejected whole and treated as
// missing: it does not refresh the Watchdog, so a stream of them stops the
// drive and centers the steering exactly as silence would. Acting on it is
// not an option - +Inf clamps to full forward or full lock, and NaN
// survives Go's min/max into the PWM layer. Half-applying it (the finite
// field only) would act on a command its sender got wrong.
func FiniteCommand(speed, steeringAngle float64) bool {
	return finite(speed) && finite(steeringAngle)
}

func finite(v float64) bool {
	return !math.IsNaN(v) && !math.IsInf(v, 0)
}
