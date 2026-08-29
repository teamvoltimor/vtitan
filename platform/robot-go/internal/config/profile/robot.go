package profile

import "math"

// DefaultRobotTOMLPath is platform/shared/config/robot.toml, relative to the
// repo root.
const DefaultRobotTOMLPath = "platform/shared/config/robot.toml"

// RobotConfig mirrors the subset of platform/shared/config/robot.toml (as
// loaded by shared.config.robot_constants.RobotConstants) that robot-go
// currently consumes.
type RobotConfig struct {
	Lidar struct {
		// Inverted matches robot.toml's lidar.inverted -- true when the
		// LIDAR is mounted upside-down, driving a mandatory 180deg
		// correction (see LidarYawOffsetRad).
		Inverted bool `mapstructure:"inverted"`
		// MountYawOffsetDeg matches robot.toml's
		// lidar.mount_yaw_offset_deg -- residual mount miscalibration
		// added on top of Inverted's 180deg, not a replacement for it.
		MountYawOffsetDeg float64 `mapstructure:"mount_yaw_offset_deg"`
	} `mapstructure:"lidar"`
}

// LidarYawOffsetRad mirrors
// shared.config.constants.robot.RobotSpecs.lidar_yaw_offset_rad(): rotates
// raw scan bearings into the robot frame (0 rad = forward) by combining the
// mandatory 180deg for an upside-down mount with any residual
// miscalibration, in that order, not interchangeably.
func (c *RobotConfig) LidarYawOffsetRad() float64 {
	invertedDeg := 0.0
	if c.Lidar.Inverted {
		invertedDeg = 180.0
	}
	return (invertedDeg + c.Lidar.MountYawOffsetDeg) * math.Pi / 180.0
}
