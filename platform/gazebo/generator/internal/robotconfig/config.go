// Package robotconfig loads the robot physical-constants TOML source of truth
// (platform/shared/config/robot.toml) and renders it into the generated files each
// consumer (Go simconfig, xacro, Python) actually reads. Regenerate via
// `simgen generate-robot-constants` (wired to `task shared:gen-robot-constants`).
package robotconfig

import (
	"fmt"
	"os"

	"github.com/pelletier/go-toml/v2"
)

type (
	// Config is the parsed contents of robot.toml. All lengths are meters, masses are
	// kilograms, angles are radians.
	Config struct {
		Chassis   Chassis   `toml:"chassis"`
		Ackermann Ackermann `toml:"ackermann"`
		Wheel     Wheel     `toml:"wheel"`
		Lidar     Lidar     `toml:"lidar"`
		Camera    Camera    `toml:"camera"`
	}

	// Chassis holds the robot body's box dimensions and mass.
	Chassis struct {
		Length float64 `toml:"length"`
		Width  float64 `toml:"width"`
		Height float64 `toml:"height"`
		Mass   float64 `toml:"mass"`
	}

	// Ackermann holds the steering geometry shared by the drivetrain and the
	// Gazebo Ackermann-steering plugin.
	Ackermann struct {
		Wheelbase        float64 `toml:"wheelbase"`
		TrackWidth       float64 `toml:"track_width"`
		MaxSteeringAngle float64 `toml:"max_steering_angle"`
	}

	// Wheel holds one wheel's dimensions and mass.
	Wheel struct {
		Radius float64 `toml:"radius"`
		Width  float64 `toml:"width"`
		Mass   float64 `toml:"mass"`
	}

	// Lidar holds the Slamtec C1 mount offset.
	Lidar struct {
		MountXOffset float64 `toml:"mount_x_offset"`
	}

	// Camera holds the RPi Camera Module 3 Wide mount offset and tilt.
	Camera struct {
		MountXOffset float64 `toml:"mount_x_offset"`
		MountZOffset float64 `toml:"mount_z_offset"`
		MountPitch   float64 `toml:"mount_pitch"`
	}
)

// Load reads and parses the robot.toml config at path.
func Load(path string) (*Config, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("read robot config: %w", err)
	}

	var cfg Config
	if err := toml.Unmarshal(data, &cfg); err != nil {
		return nil, fmt.Errorf("parse robot config: %w", err)
	}

	return &cfg, nil
}
