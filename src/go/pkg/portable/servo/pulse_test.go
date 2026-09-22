package servo_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/servo"
)

// pulseTolerance is the float64 comparison tolerance for pulse widths in
// this file.
const pulseTolerance = 1e-9

// servoTOML mirrors src/config/hardware/motors/servo.toml's base pulse
// values, with the 270 degree servo profile's range.
var servoTOML = servo.Pulse{MinPulseUS: 500, MaxPulseUS: 2500, CenterPulseUS: 1500, RangeDeg: 270}

func TestPulseUS(t *testing.T) {
	t.Parallel()

	reversed := servoTOML
	reversed.Reversed = true

	tests := []struct {
		name  string
		p     servo.Pulse
		angle float64
		want  float64
	}{
		{name: "center", p: servoTOML, angle: 0, want: 1500},
		{name: "half travel right", p: servoTOML, angle: 67.5, want: 2000},
		{name: "half travel left", p: servoTOML, angle: -67.5, want: 1000},
		{name: "clamped high", p: servoTOML, angle: 1000, want: 2500},
		{name: "clamped low", p: servoTOML, angle: -1000, want: 500},
		{name: "reversed flips sign", p: reversed, angle: 67.5, want: 1000},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			if got := servo.PulseUS(tt.p, tt.angle); math.Abs(got-tt.want) > pulseTolerance {
				t.Errorf("PulseUS(%v) = %v, want %v", tt.angle, got, tt.want)
			}
		})
	}
}

func TestNeedsWrite(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name    string
		last    float64
		hasLast bool
		next    float64
		want    bool
	}{
		{name: "first write after connect", last: 1500, hasLast: false, next: 1500, want: true},
		{name: "unchanged", last: 1500, hasLast: true, next: 1500, want: false},
		{name: "under a microsecond", last: 1500, hasLast: true, next: 1500.99, want: false},
		{name: "exactly a microsecond", last: 1500, hasLast: true, next: 1501, want: true},
		{name: "a microsecond down", last: 1500, hasLast: true, next: 1499, want: true},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			if got := servo.NeedsWrite(tt.last, tt.hasLast, tt.next); got != tt.want {
				t.Errorf("NeedsWrite(%v, %v, %v) = %v, want %v", tt.last, tt.hasLast, tt.next, got, tt.want)
			}
		})
	}
}
