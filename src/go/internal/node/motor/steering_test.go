package motor_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/node/motor"
)

const servoDegTolerance = 1e-4

// The pipeline of ackermann_motor_node.py:545-559: rad -> wheel deg ->
// / linkage -> + offset -> clamp to the servo's travel.
func TestSteeringToServoDeg(t *testing.T) {
	t.Parallel()

	hiwonder := motor.SteeringConfig{LinkageRatio: 85.0 / 135.0, ServoMaxAngleDeg: 135}
	trimmed := hiwonder
	trimmed.OffsetDeg = 2

	tests := []struct {
		name        string
		cfg         motor.SteeringConfig
		rad         float32
		want        float64
		wantClamped bool
	}{
		{name: "straight", cfg: hiwonder, rad: 0, want: 0},
		// 0.5 rad = 28.6479 wheel deg; / (85/135) = 45.4996 servo deg.
		{name: "left", cfg: hiwonder, rad: 0.5, want: 45.4996},
		{name: "right keeps its sign", cfg: hiwonder, rad: -0.5, want: -45.4996},
		{name: "offset after the linkage", cfg: trimmed, rad: 0.5, want: 47.4996},
		{name: "offset alone", cfg: trimmed, rad: 0, want: 2},
		// 85 wheel deg is full servo travel exactly: not clamped.
		{name: "full lock", cfg: hiwonder, rad: float32(85 * math.Pi / 180), want: 135},
		{name: "clamped left", cfg: hiwonder, rad: 2, want: 135, wantClamped: true},
		{name: "clamped right", cfg: hiwonder, rad: -2, want: -135, wantClamped: true},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()
			got, clamped := motor.SteeringToServoDeg(tt.rad, tt.cfg)
			if math.Abs(got-tt.want) > servoDegTolerance {
				t.Errorf("SteeringToServoDeg(%v) = %v, want %v", tt.rad, got, tt.want)
			}
			if clamped != tt.wantClamped {
				t.Errorf("SteeringToServoDeg(%v) clamped = %v, want %v", tt.rad, clamped, tt.wantClamped)
			}
		})
	}
}

func TestSteeringConfig_Validate(t *testing.T) {
	t.Parallel()

	good := motor.SteeringConfig{LinkageRatio: 85.0 / 135.0, ServoMaxAngleDeg: 135}
	if err := good.Validate(); err != nil {
		t.Fatalf("Validate(%+v) = %v", good, err)
	}
	for name, cfg := range map[string]motor.SteeringConfig{
		"zero linkage (no profile)": {ServoMaxAngleDeg: 135},
		"NaN linkage (0/0)":         {LinkageRatio: math.NaN(), ServoMaxAngleDeg: 135},
		"zero travel":               {LinkageRatio: 1},
		"infinite offset":           {LinkageRatio: 1, ServoMaxAngleDeg: 90, OffsetDeg: math.Inf(1)},
	} {
		if err := cfg.Validate(); err == nil {
			t.Errorf("%s: Validate accepted %+v", name, cfg)
		}
	}
}
