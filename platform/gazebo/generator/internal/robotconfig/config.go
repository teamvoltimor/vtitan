// Package robotconfig loads the robot physical-constants TOML source of truth
// (platform/shared/config/robot.toml) and renders it into the generated files each
// consumer (Go simconfig, xacro, Python) actually reads. Regenerate via
// `simgen generate-robot-constants` (wired to `task gen:robot-constants`).
package robotconfig

import (
	"fmt"
	"math"
	"os"

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

	// Steering holds the servo's travel and the linkage that converts it into
	// road-wheel angle.
	Steering struct {
		ServoMaxAngleDeg float64 `toml:"servo_max_angle_deg"`
		LinkageRatio     float64 `toml:"linkage_ratio"`
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

// MaxSteeringAngle is the road-wheel angle at full lock, in radians.
//
// Derived rather than declared: it is not a free parameter, it is whatever the
// servo's travel produces through the linkage. Declaring it invites the two
// from drifting apart, which is exactly what happened -- the checked-in value
// was a hand-computed product whose comment pointed at a file that pointed at
// another file.
func (s Steering) MaxSteeringAngle() float64 {
	return s.ServoMaxAngleDeg * s.LinkageRatio * math.Pi / 180
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
