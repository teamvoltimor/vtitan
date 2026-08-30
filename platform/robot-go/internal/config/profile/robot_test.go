package profile_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/config/profile"
)

func TestRobotConfig_DerivedValues(t *testing.T) {
	t.Parallel()

	cfg := &profile.RobotConfig{}
	cfg.Steering.ServoMaxAngleDeg = 135.0
	cfg.Steering.MaxWheelAngleDeg = 85.0
	cfg.Chassis.Length = 0.30
	cfg.Lidar.MountXOffset = 0.1222

	wantLinkageRatio := 85.0 / 135.0
	if got := cfg.LinkageRatio(); math.Abs(got-wantLinkageRatio) > 1e-9 {
		t.Errorf("LinkageRatio() = %v, want %v", got, wantLinkageRatio)
	}

	wantMaxSteeringAngle := 85.0 * math.Pi / 180.0
	if got := cfg.MaxSteeringAngle(); math.Abs(got-wantMaxSteeringAngle) > 1e-9 {
		t.Errorf(
			"MaxSteeringAngle() (unset SteeringLimitDeg) = %v, want %v",
			got,
			wantMaxSteeringAngle,
		)
	}

	cfg.Steering.SteeringLimitDeg = 60.0
	wantLimitedSteeringAngle := 60.0 * math.Pi / 180.0
	if got := cfg.MaxSteeringAngle(); math.Abs(got-wantLimitedSteeringAngle) > 1e-9 {
		t.Errorf(
			"MaxSteeringAngle() (set SteeringLimitDeg) = %v, want %v",
			got,
			wantLimitedSteeringAngle,
		)
	}

	wantFront := 0.30/2 - 0.1222
	if got := cfg.LidarToFrontBumper(); math.Abs(got-wantFront) > 1e-9 {
		t.Errorf("LidarToFrontBumper() = %v, want %v", got, wantFront)
	}

	wantRear := 0.30/2 + 0.1222
	if got := cfg.LidarToRearBumper(); math.Abs(got-wantRear) > 1e-9 {
		t.Errorf("LidarToRearBumper() = %v, want %v", got, wantRear)
	}
}

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
		{
			name:       "upright with residual",
			inverted:   false,
			offsetDeg:  5,
			wantRadian: 5 * math.Pi / 180,
		},
		{
			name:       "inverted with residual",
			inverted:   true,
			offsetDeg:  5,
			wantRadian: 185 * math.Pi / 180,
		},
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
