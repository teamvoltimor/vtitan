package actuation_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/actuation"
)

func TestFiniteCommand(t *testing.T) {
	t.Parallel()

	for _, tc := range []struct {
		name         string
		speed, steer float64
		want         bool
	}{
		{"both finite", 0.5, -0.2, true},
		{"NaN speed", math.NaN(), 0, false},
		{"+Inf speed", math.Inf(1), 0, false},
		{"NaN steer", 0, math.NaN(), false},
		{"-Inf steer", 0, math.Inf(-1), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()

			if got := actuation.FiniteCommand(tc.speed, tc.steer); got != tc.want {
				t.Errorf("FiniteCommand(%v, %v) = %v, want %v", tc.speed, tc.steer, got, tc.want)
			}
		})
	}
}
