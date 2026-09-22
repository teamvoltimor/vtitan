package geom_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/pkg/geom"
)

const tolerance = 1e-9

func TestWrapAngle(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name string
		in   float64
		want float64
	}{
		{name: "already in range", in: 0.5, want: 0.5},
		{name: "just over pi wraps negative", in: math.Pi + 0.1, want: -math.Pi + 0.1},
		{name: "just under -pi wraps positive", in: -math.Pi - 0.1, want: math.Pi - 0.1},
		{name: "two pi wraps to zero", in: 2 * math.Pi, want: 0},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			if got := geom.WrapAngle(tt.in); math.Abs(got-tt.want) > tolerance {
				t.Errorf("WrapAngle(%v) = %v, want %v", tt.in, got, tt.want)
			}
		})
	}
}
