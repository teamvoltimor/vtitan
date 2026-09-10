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

func TestRobotConfig_LidarMountFields(t *testing.T) {
	t.Parallel()

	// LidarYawOffsetRad() used to live here and combine these two into one
	// additive offset. It was removed because an upside-down mount reverses
	// the sensor's apparent spin direction, which an offset cannot express
	// (lidar.correctAngleDeg). What this type still owes its callers is the
	// two raw facts, unmodified, so the driver can build the correction.
	cfg := &profile.RobotConfig{}
	cfg.Lidar.Inverted = true
	cfg.Lidar.MountYawOffsetDeg = 5

	if !cfg.Lidar.Inverted {
		t.Error("Lidar.Inverted = false, want true")
	}
	if got := cfg.Lidar.MountYawOffsetDeg; got != 5 {
		t.Errorf("Lidar.MountYawOffsetDeg = %v, want 5", got)
	}
}
