// Package robotconfig loads the robot physical-constants TOML source of truth
// (platform/shared/config/robot.toml) and renders it into the generated files each
// consumer (Go simconfig, xacro, Python) actually reads. Regenerate via
// `simgen generate-robot-constants` (wired to `task gen:robot-constants`).
package robotconfig

import (
	"fmt"
	"maps"
	"math"
	"os"
	"path/filepath"
	"strings"

	"github.com/pelletier/go-toml/v2"
)

type (
	// Config is the parsed contents of robot.toml. All lengths are meters, masses are
	// kilograms, angles are radians.
	Config struct {
		Chassis    Chassis    `toml:"chassis"`
		Ackermann  Ackermann  `toml:"ackermann"`
		Steering   Steering   `toml:"steering"`
		Wheel      Wheel      `toml:"wheel"`
		Drivetrain Drivetrain `toml:"drivetrain"`
		Lidar      Lidar      `toml:"lidar"`
		Imu        Imu        `toml:"imu"`
		Camera     Camera     `toml:"camera"`
	}

	// Chassis holds the robot body's box dimensions and mass.
	Chassis struct {
		Length float64 `toml:"length"`
		Width  float64 `toml:"width"`
		Height float64 `toml:"height"`
		Mass   float64 `toml:"mass"`
	}

	// Ackermann holds the steering geometry shared by the drivetrain and the
	// Gazebo Ackermann-steering plugin. The full-lock road-wheel angle is not
	// a field: it is whatever the steering hardware produces, so it is derived
	// from Steering rather than declared alongside it.
	Ackermann struct {
		Wheelbase  float64 `toml:"wheelbase"`
		TrackWidth float64 `toml:"track_width"`
	}

	// Steering holds the servo's travel and the bench-measured road-wheel
	// angle it produces at full lock through the linkage.
	Steering struct {
		ServoMaxAngleDeg float64 `toml:"servo_max_angle_deg"`
		MaxWheelAngleDeg float64 `toml:"max_wheel_angle_deg"`
	}

	// Drivetrain holds the drive motor's measured limits and the steering
	// linkage ratio. These are physical ceilings, not tuning: the kinematics
	// clamp to them, so a speed profile above the top speed is inert.
	Drivetrain struct {
		MaxSpeedMPS    float64 `toml:"max_speed_mps"`
		MaxAccelMPS2   float64 `toml:"max_accel_mps2"`
		RearSteerRatio float64 `toml:"rear_steer_ratio"`
	}

	// Wheel holds one wheel's dimensions and mass.
	Wheel struct {
		Radius float64 `toml:"radius"`
		Width  float64 `toml:"width"`
		Mass   float64 `toml:"mass"`
	}

	// Lidar holds the Slamtec C1 mount offset and orientation.
	Lidar struct {
		MountXOffset      float64 `toml:"mount_x_offset"`
		MountZOffset      float64 `toml:"mount_z_offset"`
		Inverted          bool    `toml:"inverted"`
		MountYawOffsetDeg float64 `toml:"mount_yaw_offset_deg"`
	}

	// Imu holds the BNO085 mount offset.
	Imu struct {
		MountZOffset float64 `toml:"mount_z_offset"`
	}

	// Camera holds the RPi Camera Module 3 Wide mount offset and tilt.
	Camera struct {
		MountXOffset float64 `toml:"mount_x_offset"`
		MountZOffset float64 `toml:"mount_z_offset"`
		MountPitch   float64 `toml:"mount_pitch"`
	}
)

// MaxSteeringAngle is the road-wheel angle at full lock, in radians. Just the
// bench-measured MaxWheelAngleDeg converted to radians -- see robot.toml's
// [steering] comment for why that is the field that gets measured and
// declared, rather than a ratio.
func (s Steering) MaxSteeringAngle() float64 {
	return s.MaxWheelAngleDeg * math.Pi / 180
}

// LinkageRatio is road-wheel degrees produced per servo degree, derived from
// the bench-measured MaxWheelAngleDeg rather than declared directly -- kept
// only for consumers that still want the ratio form (e.g. converting an
// arbitrary wheel angle to a servo angle), not as the source of truth.
func (s Steering) LinkageRatio() float64 {
	return s.MaxWheelAngleDeg / s.ServoMaxAngleDeg
}

// TotalYawOffsetDeg combines the mandatory 180deg from an inverted (upside-down)
// mount with any independent residual miscalibration. Consumers that only need
// the resulting frame orientation (the URDF/xacro model) use this; consumers
// that also need to set a driver-level `inverted` launch parameter (the real
// ROS2 nodes) read Inverted and MountYawOffsetDeg separately instead.
func (l Lidar) TotalYawOffsetDeg() float64 {
	if l.Inverted {
		return 180.0 + l.MountYawOffsetDeg
	}
	return l.MountYawOffsetDeg
}

// Load reads and parses the robot.toml config at basePath, then deep-merges
// each named hardware profile's overlay on top, in order (a later profile
// wins any key both it and an earlier one set). profileNames is typically
// parsed from a comma-separated CLI flag via ParseProfileNames.
//
// A profile only needs to declare the keys it changes -- e.g. a [steering]
// block for a different servo -- so the merge happens on the raw parsed
// maps before decoding into Config, the same way
// shared.config.robot_constants.RobotConstants.load_default does on the
// Python side. See docs/internal/plans/2026-08-11-servo-hardware-profiles.md.
func Load(basePath string, profileNames ...string) (*Config, error) {
	merged, err := readTOMLMap(basePath)
	if err != nil {
		return nil, fmt.Errorf("read base robot config: %w", err)
	}

	profilesRoot := filepath.Join(filepath.Dir(basePath), "profiles")
	for _, name := range profileNames {
		overlayDir := filepath.Join(profilesRoot, name)
		if info, statErr := os.Stat(overlayDir); statErr != nil || !info.IsDir() {
			return nil, fmt.Errorf("unknown hardware profile %q: expected a directory at %s", name, overlayDir)
		}

		overlayPath := filepath.Join(overlayDir, "robot.toml")
		if _, statErr := os.Stat(overlayPath); statErr != nil {
			continue // profile dir exists but has no robot.toml overlay -- nothing to merge
		}

		overlay, readErr := readTOMLMap(overlayPath)
		if readErr != nil {
			return nil, fmt.Errorf("read profile %q robot config: %w", name, readErr)
		}
		merged = deepMergeMaps(merged, overlay)
	}

	remarshaled, err := toml.Marshal(merged)
	if err != nil {
		return nil, fmt.Errorf("remarshal merged robot config: %w", err)
	}

	var cfg Config
	if err := toml.Unmarshal(remarshaled, &cfg); err != nil {
		return nil, fmt.Errorf("parse merged robot config: %w", err)
	}

	return &cfg, nil
}

// ParseProfileNames splits a comma-separated profile-list flag value (e.g.
// "270deg-hiwonder-35kg" or "270deg-hiwonder-35kg,other") into ordered, trimmed, non-empty names.
// Mirrors shared.config.hardware_profile.active_profiles on the Python side.
func ParseProfileNames(raw string) []string {
	var names []string
	for name := range strings.SplitSeq(raw, ",") {
		name = strings.TrimSpace(name)
		if name != "" {
			names = append(names, name)
		}
	}
	return names
}

func readTOMLMap(path string) (map[string]any, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}

	var m map[string]any
	if err := toml.Unmarshal(data, &m); err != nil {
		return nil, err
	}
	return m, nil
}

// deepMergeMaps merges override onto base, recursing into nested tables so
// override only needs to declare the keys it changes -- a plain map merge
// would drop sibling keys inside any table override also touches.
func deepMergeMaps(base, override map[string]any) map[string]any {
	result := make(map[string]any, len(base))
	maps.Copy(result, base)
	for k, v := range override {
		if overrideTable, ok := v.(map[string]any); ok {
			if baseTable, ok := result[k].(map[string]any); ok {
				result[k] = deepMergeMaps(baseTable, overrideTable)
				continue
			}
		}
		result[k] = v
	}
	return result
}
