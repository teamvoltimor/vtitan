package navutil

import "math"

// AngleFan returns n evenly-spaced angles spanning [-pi, pi) (open upper
// bound), matching the Python sector helpers' lidar_angles=None fallback.
// The first angle is -pi; the last is just below +pi. Use this for LIDAR
// sweeps where angle i indexes range i.
func AngleFan(n int) []float64 {
	angles := make([]float64, n)
	if n == 0 {
		return angles
	}
	step := 2 * math.Pi / float64(n)
	for i := range angles {
		angles[i] = -math.Pi + float64(i)*step
	}
	return angles
}

// AngleFanClosed returns n evenly-spaced angles spanning [-pi, pi] (closed
// upper bound), i.e. with step 2*pi/(n-1). Use this where the sweep must
// include both end-fire rays (e.g. numpy linspace(-pi, pi, n)).
func AngleFanClosed(n int) []float64 {
	angles := make([]float64, n)
	if n <= 1 {
		return angles
	}
	step := 2 * math.Pi / float64(n-1)
	for i := range angles {
		angles[i] = -math.Pi + float64(i)*step
	}
	return angles
}
