//go:build linux

package motor_test

import (
	"math"
	"path/filepath"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/internal/node/motor"
)

// shippedRoot is the repo root, from src/go/internal/node/motor.
var shippedRoot = filepath.Join("..", "..", "..", "..", "..")

// Against the checked-in tree and the profile the robot runs: the linkage
// is max_wheel_angle_deg / servo_max_angle_deg = 85 / 135.
func TestSteeringFor_ShippedTreeWithProfile(t *testing.T) {
	t.Setenv(profile.EnvVar, "270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm")

	cfg, err := motor.SteeringFor(shippedRoot)
	if err != nil {
		t.Fatalf("SteeringFor: %v", err)
	}
	if math.Abs(cfg.LinkageRatio-85.0/135.0) > 1e-12 {
		t.Errorf("LinkageRatio = %v, want 85/135", cfg.LinkageRatio)
	}
	if cfg.ServoMaxAngleDeg != 135 {
		t.Errorf("ServoMaxAngleDeg = %v, want 135", cfg.ServoMaxAngleDeg)
	}
	if cfg.OffsetDeg != 0 {
		t.Errorf("OffsetDeg = %v, want motors.toml's 0.0", cfg.OffsetDeg)
	}
}

// Without a servo profile the geometry is unknown, and SteeringFor says so
// instead of guessing.
func TestSteeringFor_WithoutProfileIsAnError(t *testing.T) {
	t.Setenv(profile.EnvVar, "")

	if _, err := motor.SteeringFor(shippedRoot); err == nil {
		t.Fatal("SteeringFor resolved a linkage with no servo profile active")
	}
	if _, err := motor.SteeringFor(""); err == nil {
		t.Fatal("SteeringFor(\"\") resolved a linkage")
	}
}
