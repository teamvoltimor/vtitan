package navutil

// Clamp restricts value to the closed interval [lo, hi], matching
// utils.clamp.
func Clamp(value, lo, hi float64) float64 {
	return max(lo, min(hi, value))
}
