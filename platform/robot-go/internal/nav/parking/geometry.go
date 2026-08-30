package parking

import "math"

// NormaliseAngle wraps an angle into the (-pi, pi] range.
func NormaliseAngle(angle float64) float64 {
	for angle > math.Pi {
		angle -= 2 * math.Pi
	}
	for angle < -math.Pi {
		angle += 2 * math.Pi
	}
	return angle
}
