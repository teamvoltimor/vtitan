package motor

// White-box (package motor, not motor_test): splitDuty/clampDuty are
// unexported pure functions — this is the actual boundary the Fast-Brake
// invariant lives at, deliberately tested directly rather than only through
// Controller.

import (
	"math"
	"testing"
)

// dutyTolerance is the float64 comparison tolerance for duty-cycle values
// in this file.
const dutyTolerance = 1e-9

func TestClampDuty(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name     string
		in, want float64
	}{
		{name: "within range", in: 0.5, want: 0.5},
		{name: "negative within", in: -0.5, want: -0.5},
		{name: "above max", in: 1.5, want: 1.0},
		{name: "below min", in: -1.5, want: -1.0},
		{name: "exactly max", in: 1.0, want: 1.0},
		{name: "exactly min", in: -1.0, want: -1.0},
		{name: "zero", in: 0.0, want: 0.0},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			if got := clampDuty(tt.in); got != tt.want {
				t.Errorf("clampDuty(%v) = %v, want %v", tt.in, got, tt.want)
			}
		})
	}
}

// TestSplitDuty_NeverBothNonzero is the direct test of the Fast-Brake
// invariant (see splitDuty's doc comment): RPWM=LPWM=HIGH is a documented
// fault state on the BTS7960 and must be structurally impossible to produce
// from splitDuty's output, across the full input range, not just a few
// hand-picked cases.
func TestSplitDuty_NeverBothNonzero(t *testing.T) {
	t.Parallel()

	const step = 0.01
	for signed := -1.5; signed <= 1.5; signed += step {
		rpwm, lpwm := splitDuty(signed)

		if rpwm != 0 && lpwm != 0 {
			t.Fatalf("splitDuty(%v) = (rpwm=%v, lpwm=%v), both nonzero -- Fast Brake fault state", signed, rpwm, lpwm)
		}
		if rpwm < 0 || rpwm > 1 {
			t.Fatalf("splitDuty(%v) rpwm = %v, want in [0, 1]", signed, rpwm)
		}
		if lpwm < 0 || lpwm > 1 {
			t.Fatalf("splitDuty(%v) lpwm = %v, want in [0, 1]", signed, lpwm)
		}
	}
}

func TestSplitDuty_MagnitudeMatchesClampedInput(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name       string
		in         float64
		wantRPWM   float64
		wantLPWM   float64
		wantWasNeg bool
	}{
		{name: "forward half", in: 0.5, wantRPWM: 0.5, wantLPWM: 0},
		{name: "reverse half", in: -0.5, wantRPWM: 0, wantLPWM: 0.5, wantWasNeg: true},
		{name: "forward full", in: 1.0, wantRPWM: 1.0, wantLPWM: 0},
		{name: "reverse full", in: -1.0, wantRPWM: 0, wantLPWM: 1.0, wantWasNeg: true},
		{name: "zero", in: 0.0, wantRPWM: 0, wantLPWM: 0},
		{name: "clamped beyond", in: 2.0, wantRPWM: 1.0, wantLPWM: 0},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			rpwm, lpwm := splitDuty(tt.in)
			if math.Abs(rpwm-tt.wantRPWM) > dutyTolerance || math.Abs(lpwm-tt.wantLPWM) > dutyTolerance {
				t.Errorf("splitDuty(%v) = (%v, %v), want (%v, %v)", tt.in, rpwm, lpwm, tt.wantRPWM, tt.wantLPWM)
			}
		})
	}
}
