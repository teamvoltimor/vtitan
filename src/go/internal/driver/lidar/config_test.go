package lidar_test

import (
	"log/slog"
	"os"
	"path/filepath"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/internal/driver/lidar"
)

const serialTOML = "serial_port = \"/dev/ttyUSB1\"\nserial_baudrate = 460800\n"

// writeConfigRoot lays out the two files ConfigFor reads -- the serial
// settings in lidar.toml and the physical mount in robot.toml -- under a
// temporary root, at the same relative paths the repo uses.
func writeConfigRoot(t *testing.T, lidarTOML, robotTOML string) string {
	t.Helper()

	root := t.TempDir()
	for path, body := range map[string]string{
		profile.DefaultLidarLaunchTOMLPath: lidarTOML,
		profile.DefaultRobotTOMLPath:       robotTOML,
	} {
		full := filepath.Join(root, path)
		if err := os.MkdirAll(filepath.Dir(full), 0o755); err != nil {
			t.Fatalf("MkdirAll(%s): %v", filepath.Dir(full), err)
		}
		if err := os.WriteFile(full, []byte(body), 0o600); err != nil {
			t.Fatalf("WriteFile(%s): %v", full, err)
		}
	}
	return root
}

// TestConfigForResolvesMountCorrection is the regression guard for the
// defect this wiring closes: ConfigFor used to set only Port and BaudRate,
// so the driver ran with Inverted=false and published raw sensor bearings
// no matter what robot.toml said about the mount.
func TestConfigForResolvesMountCorrection(t *testing.T) {
	t.Parallel()

	root := writeConfigRoot(t, serialTOML,
		"[lidar]\ninverted = true\nmount_yaw_offset_deg = 5.0\n")

	cfg := lidar.ConfigFor(slog.Default(), root)

	if cfg.Port != "/dev/ttyUSB1" {
		t.Errorf("Port = %q, want /dev/ttyUSB1", cfg.Port)
	}
	if cfg.BaudRate != 460800 {
		t.Errorf("BaudRate = %d, want 460800", cfg.BaudRate)
	}
	if !cfg.Inverted {
		t.Error("Inverted = false, want true from robot.toml's [lidar]")
	}
	if cfg.YawOffsetDeg != 5.0 {
		t.Errorf("YawOffsetDeg = %v, want 5", cfg.YawOffsetDeg)
	}
}

// TestConfigForMountCorrectionNeedsNoHardwareProfile pins that the mount
// correction survives an unset VTITAN_HARDWARE_PROFILE. robot.toml omits
// the steering and drivetrain keys on purpose, so a loader that enforced
// them would reject this file and fall back to an uncorrected frame --
// letting a missing servo spec decide which way the LIDAR points.
func TestConfigForMountCorrectionNeedsNoHardwareProfile(t *testing.T) {
	t.Setenv("VTITAN_HARDWARE_PROFILE", "")

	root := writeConfigRoot(t, serialTOML,
		"[lidar]\ninverted = true\nmount_yaw_offset_deg = 0.0\n")

	if cfg := lidar.ConfigFor(slog.Default(), root); !cfg.Inverted {
		t.Error("Inverted = false with no active hardware profile, want true")
	}
}

// TestConfigForUprightMountLeavesAnglesAlone covers the other branch: an
// upright mount must not acquire a correction from the mere presence of the
// section.
func TestConfigForUprightMountLeavesAnglesAlone(t *testing.T) {
	t.Parallel()

	root := writeConfigRoot(t, serialTOML,
		"[lidar]\ninverted = false\nmount_yaw_offset_deg = 0.0\n")

	cfg := lidar.ConfigFor(slog.Default(), root)

	if cfg.Inverted {
		t.Error("Inverted = true, want false")
	}
	if cfg.YawOffsetDeg != 0 {
		t.Errorf("YawOffsetDeg = %v, want 0", cfg.YawOffsetDeg)
	}
}

// TestConfigForMissingRobotTOMLKeepsStreaming pins the degradation choice:
// an unreadable robot.toml costs the correction, not the sensor.
func TestConfigForMissingRobotTOMLKeepsStreaming(t *testing.T) {
	t.Parallel()

	root := t.TempDir()
	full := filepath.Join(root, profile.DefaultLidarLaunchTOMLPath)
	if err := os.MkdirAll(filepath.Dir(full), 0o755); err != nil {
		t.Fatalf("MkdirAll: %v", err)
	}
	if err := os.WriteFile(full, []byte(serialTOML), 0o600); err != nil {
		t.Fatalf("WriteFile: %v", err)
	}

	cfg := lidar.ConfigFor(slog.Default(), root)

	if cfg.Port != "/dev/ttyUSB1" {
		t.Errorf("Port = %q, want the serial config to still load", cfg.Port)
	}
	if cfg.Inverted || cfg.YawOffsetDeg != 0 {
		t.Errorf("mount correction = (%v, %v), want the uncorrected (false, 0)",
			cfg.Inverted, cfg.YawOffsetDeg)
	}
}
