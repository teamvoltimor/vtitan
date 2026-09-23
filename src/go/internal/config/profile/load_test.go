package profile_test

import (
	"os"
	"path/filepath"
	"slices"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/hardware"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/hardware/imu"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/hardware/motors"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
)

func TestLoad_BaseOnly(t *testing.T) {
	t.Parallel()

	cfg, err := profile.LoadRobotValues(filepath.Join("testdata", "robot.toml"), nil)
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

	cfg, err := profile.LoadRobotValues(
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

	_, err := profile.LoadRobotValues(
		filepath.Join("testdata", "robot.toml"),
		[]string{"does-not-exist"},
	)
	if err == nil {
		t.Fatal("Load: want error for unknown profile, got nil")
	}
}

// TestLoad_ProfileKnownAboveIsSkipped pins the component-tree rule: a
// component whose own profiles/ tree lacks a name still loads when a
// profiles/ tree above it knows the name (src/config/profiles above
// src/config/hardware/...), and still errors when nothing does.
func TestLoad_ProfileKnownAboveIsSkipped(t *testing.T) {
	t.Parallel()

	root := t.TempDir()
	if err := os.MkdirAll(filepath.Join(root, "profiles", "servo-a"), 0o750); err != nil {
		t.Fatal(err)
	}
	component := filepath.Join(root, "hardware", "lidar.toml")
	if err := os.MkdirAll(filepath.Dir(component), 0o750); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(component, []byte("serial_port = \"/dev/ttyUSB0\"\n"), 0o600); err != nil {
		t.Fatal(err)
	}

	cfg, err := profile.Load[hardware.HardwareLidar](component, []string{"servo-a"})
	if err != nil {
		t.Fatalf("Load with a profile known above: %v", err)
	}
	if cfg.SerialPort != "/dev/ttyUSB0" {
		t.Errorf("SerialPort = %q, want the base value", cfg.SerialPort)
	}

	if _, err = profile.Load[hardware.HardwareLidar](component, []string{"servo-typo"}); err == nil {
		t.Fatal("Load: want error for a profile no tree knows, got nil")
	}
}

// TestLoad_ShippedComponentsTakeTheRobotsProfiles loads the shipped LIDAR and
// IMU files with the profile set the robot runs. Neither tree has a profiles/
// directory for those names, and this used to fail, so both components ran on
// their literal fallbacks whenever VTITAN_HARDWARE_PROFILE was set.
func TestLoad_ShippedComponentsTakeTheRobotsProfiles(t *testing.T) {
	t.Parallel()

	names := []string{"270deg-hiwonder-35kg", "rev-hd-hex-motor-6000rpm"}
	repo := filepath.Join("..", "..", "..", "..", "..")
	if _, err := profile.Load[hardware.HardwareLidar](
		filepath.Join(repo, profile.DefaultLidarLaunchTOMLPath), names,
	); err != nil {
		t.Errorf("LIDAR: %v", err)
	}
	if _, err := profile.Load[imu.HardwareImuBno08XUartRvc](
		filepath.Join(repo, profile.DefaultIMUUARTRVCTOMLPath), names,
	); err != nil {
		t.Errorf("IMU: %v", err)
	}
}

func TestLoad_ProfileDirWithoutMatchingFileIsSkipped(t *testing.T) {
	t.Parallel()

	cfg, err := profile.LoadRobotValues(
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

	cfg, err := profile.Load[motors.HardwareMotorsMotors](filepath.Join("testdata", "motors.toml"), nil)
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

	cfg, err := profile.Load[generated.TrackConfig](filepath.Join("testdata", "track.toml"), nil)
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	if cfg.Track.MatSize != 3.2 {
		t.Errorf("Track.MatSize = %v, want 3.2", cfg.Track.MatSize)
	}
	if !slices.Equal(cfg.Corridor.DivisionLines, []float64{0.40, 0.60}) {
		t.Errorf("Corridor.DivisionLines = %v, want [0.40, 0.60]", cfg.Corridor.DivisionLines)
	}
	wantAlignment := []generated.SpawnAlignment{
		generated.SpawnAlignmentInner,
		generated.SpawnAlignmentOuter,
		generated.SpawnAlignmentOuter,
	}
	if !slices.Equal(cfg.StartingZone.SpawnAlignment, wantAlignment) {
		t.Errorf(
			"StartingZone.SpawnAlignment = %v, want [inner outer outer]",
			cfg.StartingZone.SpawnAlignment,
		)
	}
}

func TestLoad_MissingBaseErrors(t *testing.T) {
	t.Parallel()

	_, err := profile.LoadRobotValues(
		filepath.Join("testdata", "does-not-exist.toml"),
		nil,
	)
	if err == nil {
		t.Fatal("Load: want error for missing base file, got nil")
	}
}
