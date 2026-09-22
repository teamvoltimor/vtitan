package hwconfig_test

import (
	"path/filepath"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/internal/hwconfig"
)

// shippedRoot is the repo root, from src/go/internal/hwconfig, so the
// loader is exercised against the checked-in servo.toml and its overlay.
var shippedRoot = filepath.Join("..", "..", "..", "..")

// The 270 degree servo profile overlays range_deg; everything else is the
// base servo.toml.
func TestServo_ShippedTOMLWithServoProfile(t *testing.T) {
	t.Setenv(profile.EnvVar, "270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm")

	cfg, err := hwconfig.Servo(shippedRoot)
	if err != nil {
		t.Fatalf("Servo: %v", err)
	}
	if cfg.RangeDeg != 270 {
		t.Errorf("RangeDeg = %v, want 270 from the servo profile", cfg.RangeDeg)
	}
	if cfg.GPIOPin != 12 || cfg.PWMChip != 0 || cfg.PWMChannel != 0 || cfg.FrequencyHz != 50 {
		t.Errorf("wiring = GPIO %d pwmchip%d/pwm%d %d Hz, want GPIO 12 pwmchip0/pwm0 50 Hz",
			cfg.GPIOPin, cfg.PWMChip, cfg.PWMChannel, cfg.FrequencyHz)
	}
	if cfg.MinPulseUS != 500 || cfg.CenterPulseUS != 1500 || cfg.MaxPulseUS != 2500 || cfg.Reversed {
		t.Errorf("pulses = %v/%v/%v reversed=%v, want 500/1500/2500 false",
			cfg.MinPulseUS, cfg.CenterPulseUS, cfg.MaxPulseUS, cfg.Reversed)
	}
	if cfg.Root != "" {
		t.Errorf("Root = %q, want empty (the driver's default)", cfg.Root)
	}
}

func TestServo_WithoutProfileIsTheBaseFile(t *testing.T) {
	t.Setenv(profile.EnvVar, "")

	cfg, err := hwconfig.Servo(shippedRoot)
	if err != nil {
		t.Fatalf("Servo: %v", err)
	}
	if cfg.RangeDeg != 180 {
		t.Errorf("RangeDeg = %v, want the base 180", cfg.RangeDeg)
	}
}

func TestServo_EmptyRootIsAnError(t *testing.T) {
	t.Parallel()

	if _, err := hwconfig.Servo(""); err == nil {
		t.Fatal("Servo(\"\") returned a config; there is no safe default")
	}
}
