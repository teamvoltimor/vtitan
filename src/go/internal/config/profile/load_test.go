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

	cfg, err := profile.Load[profile.RobotConfig](
		filepath.Join("testdata", "robot.toml"),
		[]string{"inverted-mount"},
	)
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

	_, err := profile.Load[profile.RobotConfig](
		filepath.Join("testdata", "robot.toml"),
		[]string{"does-not-exist"},
	)
	if err == nil {
		t.Fatal("Load: want error for unknown profile, got nil")
	}
}

func TestLoad_ProfileDirWithoutMatchingFileIsSkipped(t *testing.T) {
	t.Parallel()

	cfg, err := profile.Load[profile.RobotConfig](
		filepath.Join("testdata", "robot.toml"), []string{"empty-overlay-dir"},
	)
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

func TestLoadRobotConfig_MissingRequiredFieldsErrors(t *testing.T) {
	t.Parallel()

	_, err := profile.LoadRobotConfig(filepath.Join("testdata", "robot.toml"), nil)
	if err == nil {
		t.Fatal(
			"LoadRobotConfig: want error when no profile supplies drivetrain/steering facts, got nil",
		)
	}
}

func TestLoadRobotConfig_ProfileSuppliesRequiredFields(t *testing.T) {
	t.Parallel()

	cfg, err := profile.LoadRobotConfig(
		filepath.Join("testdata", "robot.toml"),
		[]string{"full-specs"},
	)
	if err != nil {
		t.Fatalf("LoadRobotConfig: %v", err)
	}
	if cfg.Drivetrain.MaxSpeedMPS != 1.0 {
		t.Errorf("Drivetrain.MaxSpeedMPS = %v, want 1.0", cfg.Drivetrain.MaxSpeedMPS)
	}
	if cfg.Steering.ServoMaxAngleDeg != 135.0 {
		t.Errorf("Steering.ServoMaxAngleDeg = %v, want 135.0", cfg.Steering.ServoMaxAngleDeg)
	}
	if cfg.Steering.MaxWheelAngleDeg != 85.0 {
		t.Errorf("Steering.MaxWheelAngleDeg = %v, want 85.0", cfg.Steering.MaxWheelAngleDeg)
	}
}

func TestLoad_TrackConfig(t *testing.T) {
	t.Parallel()

	cfg, err := profile.Load[profile.TrackConfig](filepath.Join("testdata", "track.toml"), nil)
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	if cfg.Track.MatSize != 3.2 {
		t.Errorf("Track.MatSize = %v, want 3.2", cfg.Track.MatSize)
	}
	if cfg.Corridor.DivisionLines != [2]float64{0.40, 0.60} {
		t.Errorf("Corridor.DivisionLines = %v, want [0.40, 0.60]", cfg.Corridor.DivisionLines)
	}
	if cfg.StartingZone.SpawnAlignment != [3]string{"inner", "outer", "outer"} {
		t.Errorf(
			"StartingZone.SpawnAlignment = %v, want [inner outer outer]",
			cfg.StartingZone.SpawnAlignment,
		)
	}
}

func TestLoad_MissingBaseErrors(t *testing.T) {
	t.Parallel()

	_, err := profile.Load[profile.RobotConfig](
		filepath.Join("testdata", "does-not-exist.toml"),
		nil,
	)
	if err == nil {
		t.Fatal("Load: want error for missing base file, got nil")
	}
}
