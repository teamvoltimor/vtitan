package navutil_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"
)

func TestNearestRay(t *testing.T) {
	t.Parallel()

	ranges := []float64{1.0, 2.0, 3.0, 4.0}
	angles := []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2}

	tests := []struct {
		name   string
		target float64
		want   float64
	}{
		{name: "forward", target: 0, want: 1.0},
		{name: "left", target: math.Pi / 2, want: 2.0},
		{name: "back", target: math.Pi, want: 3.0},
		{name: "right", target: -math.Pi / 2, want: 4.0},
		{name: "near-forward picks forward", target: 0.1, want: 1.0},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			if got := navutil.NearestRay(ranges, angles, tt.target); got != tt.want {
				t.Errorf("NearestRay(target=%v) = %v, want %v", tt.target, got, tt.want)
			}
		})
	}
}

func TestForwardClearance(t *testing.T) {
	t.Parallel()

	const arcRad = 8 * math.Pi / 180
	const minValid = 0.05

	t.Run("minimum within arc", func(t *testing.T) {
		t.Parallel()

		ranges := []float64{2.0, 1.0, 5.0}
		angles := []float64{0, 0.05, math.Pi} // last ray is outside the arc
		got := navutil.ForwardClearance(ranges, angles, arcRad, minValid)
		if got != 1.0 {
			t.Errorf("ForwardClearance() = %v, want 1.0", got)
		}
	})

	t.Run("no valid ray in arc returns +Inf", func(t *testing.T) {
		t.Parallel()

		ranges := []float64{5.0}
		angles := []float64{math.Pi} // outside the forward arc
		got := navutil.ForwardClearance(ranges, angles, arcRad, minValid)
		if !math.IsInf(got, 1) {
			t.Errorf("ForwardClearance() = %v, want +Inf", got)
		}
	})

	t.Run("below min valid range excluded", func(t *testing.T) {
		t.Parallel()

		ranges := []float64{0.01, 2.0}
		angles := []float64{0, 0}
		got := navutil.ForwardClearance(ranges, angles, arcRad, minValid)
		if got != 2.0 {
			t.Errorf("ForwardClearance() = %v, want 2.0 (0.01 below min valid excluded)", got)
		}
	})
}
