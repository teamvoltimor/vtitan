package profile_test

import (
	"path/filepath"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/config/profile"
)

func TestLoad_BaseOnly(t *testing.T) {
	t.Parallel()

	cfg, err := profile.Load[profile.RobotConfig](filepath.Join("testdata", "robot.toml"), nil)
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	if cfg.Lidar.Inverted {
		t.Errorf("Lidar.Inverted = true, want false")
	}
	if cfg.Lidar.MountYawOffsetDeg != 0 {
		t.Errorf("Lidar.MountYawOffsetDeg = %v, want 0", cfg.Lidar.MountYawOffsetDeg)
	}
}

func TestLoad_ProfileOverlayMerges(t *testing.T) {
	t.Parallel()

	cfg, err := profile.Load[profile.RobotConfig](filepath.Join("testdata", "robot.toml"), []string{"inverted-mount"})
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	if !cfg.Lidar.Inverted {
		t.Errorf("Lidar.Inverted = false, want true")
	}
	if cfg.Lidar.MountYawOffsetDeg != 5.0 {
		t.Errorf("Lidar.MountYawOffsetDeg = %v, want 5.0", cfg.Lidar.MountYawOffsetDeg)
	}
}

func TestLoad_UnknownProfileErrors(t *testing.T) {
	t.Parallel()

	_, err := profile.Load[profile.RobotConfig](filepath.Join("testdata", "robot.toml"), []string{"does-not-exist"})
	if err == nil {
		t.Fatal("Load: want error for unknown profile, got nil")
	}
}

func TestLoad_ProfileDirWithoutMatchingFileIsSkipped(t *testing.T) {
	t.Parallel()

	cfg, err := profile.Load[profile.RobotConfig](filepath.Join("testdata", "robot.toml"), []string{"empty-overlay-dir"})
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	if cfg.Lidar.Inverted {
		t.Errorf("Lidar.Inverted = true, want false (base unchanged)")
	}
}

func TestLoad_MotorsConfig(t *testing.T) {
	t.Parallel()

	cfg, err := profile.Load[profile.MotorsConfig](filepath.Join("testdata", "motors.toml"), nil)
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	if cfg.Drive.SpeedScale != 30.0 {
		t.Errorf("Drive.SpeedScale = %v, want 30.0", cfg.Drive.SpeedScale)
	}
}

func TestLoad_MissingBaseErrors(t *testing.T) {
	t.Parallel()

	_, err := profile.Load[profile.RobotConfig](filepath.Join("testdata", "does-not-exist.toml"), nil)
	if err == nil {
		t.Fatal("Load: want error for missing base file, got nil")
	}
}
