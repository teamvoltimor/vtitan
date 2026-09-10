package navutil_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
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

			if got := navutil.WrapAngle(tt.in); math.Abs(got-tt.want) > tolerance {
				t.Errorf("WrapAngle(%v) = %v, want %v", tt.in, got, tt.want)
			}
		})
	}
}

func TestAxisOffsetRad(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name string
		yaw  float64
		want float64
	}{
		{name: "on axis (0)", yaw: 0, want: 0},
		{name: "on axis (pi/2)", yaw: math.Pi / 2, want: 0},
		{name: "5 degrees left of axis", yaw: 5 * math.Pi / 180, want: 5 * math.Pi / 180},
		{name: "5 degrees right of axis", yaw: -5 * math.Pi / 180, want: -5 * math.Pi / 180},
		{
			name: "5 degrees past pi/2 offsets from pi/2, not 0",
			yaw:  math.Pi/2 + 5*math.Pi/180,
			want: 5 * math.Pi / 180,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			if got := navutil.AxisOffsetRad(tt.yaw); math.Abs(got-tt.want) > tolerance {
				t.Errorf("AxisOffsetRad(%v) = %v, want %v", tt.yaw, got, tt.want)
			}
		})
	}
}

func TestAxisErrorRad_AlwaysNonNegative(t *testing.T) {
	t.Parallel()

	got := navutil.AxisErrorRad(-5 * math.Pi / 180)
	want := 5 * math.Pi / 180
	if math.Abs(got-want) > tolerance {
		t.Errorf("AxisErrorRad(-5deg) = %v, want %v (sign dropped)", got, want)
	}
}
