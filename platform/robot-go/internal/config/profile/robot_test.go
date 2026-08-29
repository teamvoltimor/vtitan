package profile_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/config/profile"
)

func TestRobotConfig_LidarYawOffsetRad(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name       string
		inverted   bool
		offsetDeg  float64
		wantRadian float64
	}{
		{name: "upright, no residual", inverted: false, offsetDeg: 0, wantRadian: 0},
		{name: "inverted, no residual", inverted: true, offsetDeg: 0, wantRadian: math.Pi},
		{name: "upright with residual", inverted: false, offsetDeg: 5, wantRadian: 5 * math.Pi / 180},
		{name: "inverted with residual", inverted: true, offsetDeg: 5, wantRadian: 185 * math.Pi / 180},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			cfg := &profile.RobotConfig{}
			cfg.Lidar.Inverted = tt.inverted
			cfg.Lidar.MountYawOffsetDeg = tt.offsetDeg

			if got := cfg.LidarYawOffsetRad(); math.Abs(got-tt.wantRadian) > 1e-9 {
				t.Errorf("LidarYawOffsetRad() = %v, want %v", got, tt.wantRadian)
			}
		})
	}
}
