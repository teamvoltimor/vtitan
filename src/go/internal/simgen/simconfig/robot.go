package simconfig

import (
	"fmt"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
)

// Robot is the chassis and sensor geometry the sim pipeline works from,
// loaded once from robot.toml at runtime (with the active hardware-profile
// overlays, so servo and motor fields resolve).
//
// It replaces the former hand-maintained `robot_constants.gen.go`: that file
// duplicated robot.toml and had already drifted (its LIDAR mount z was +0.02
// against robot.toml's -0.02, and its steering values were the retired 180
// degree servo). Values now come from the same robot.toml the rest of Go and
// Python read.
type Robot struct {
	RobotLength      float64
	RobotWidth       float64
	RobotHeight      float64
	RobotWheelbase   float64
	RobotTrackWidth  float64
	RobotWheelRadius float64
	RobotWheelWidth  float64

	RobotMaxSteering      float64
	RobotServoMaxAngleDeg float64
	RobotMaxWheelAngleDeg float64
	RobotLinkageRatio     float64

	RobotChassisMass float64
	RobotWheelMass   float64

	RobotMaxSpeedMPS    float64
	RobotMaxAccelMPS2   float64
	RobotRearSteerRatio float64

	RobotLidarMountXOffset      float64
	RobotLidarMountZOffset      float64
	RobotLidarInverted          bool
	RobotLidarMountYawOffsetDeg float64

	RobotImuMountZOffset float64

	RobotCameraMountXOffset float64
	RobotCameraMountZOffset float64
	RobotCameraPitchRad     float64
}

// DefaultHardwareProfiles are the shipped hardware-profile overlays the sim
// tooling applies when VTITAN_HARDWARE_PROFILE is unset: the servo profile and
// the drive-motor profile the robot actually ships with. They are the pair that
// supplies the required steering and drivetrain facts robot.toml omits.
func DefaultHardwareProfiles() []string {
	return []string{"270deg-hiwonder-35kg", "rev-hd-hex-motor-6000rpm"}
}

// ActiveHardwareProfiles returns the hardware-profile names in
// VTITAN_HARDWARE_PROFILE, or DefaultHardwareProfiles when it is unset.
func ActiveHardwareProfiles() []string {
	if names := profile.ActiveNames(); len(names) > 0 {
		return names
	}
	return DefaultHardwareProfiles()
}

// LoadRobot loads robot.toml under configRoot overlaid with the active
// hardware profiles and maps it into Robot.
//
// Args:
//
//	configRoot: directory holding robot.toml (e.g. "src/config") or the
//	    robot.toml path itself.
//	profileNames: active hardware profile directory names (e.g.
//	    "270deg-hiwonder-35kg", "rev-hd-hex-motor-6000rpm"). The servo and
//	    motor facts are required from these, so it must not be empty.
//
// Returns:
//
//	The loaded Robot, or an error if robot.toml cannot be read or a required
//	component fact is missing.
func LoadRobot(robotPath string, profileNames []string) (*Robot, error) {
	if filepath.Ext(robotPath) == "" {
		robotPath = filepath.Join(robotPath, "robot.toml")
	}
	cfg, err := profile.LoadRobotConfig(robotPath, profileNames)
	if err != nil {
		return nil, fmt.Errorf("simconfig: load robot.toml: %w", err)
	}
	return &Robot{
		RobotLength:      cfg.Chassis.Length,
		RobotWidth:       cfg.Chassis.Width,
		RobotHeight:      cfg.Chassis.Height,
		RobotWheelbase:   cfg.Ackermann.Wheelbase,
		RobotTrackWidth:  cfg.Ackermann.TrackWidth,
		RobotWheelRadius: cfg.Wheel.Radius,
		RobotWheelWidth:  cfg.Wheel.Width,

		RobotMaxSteering:      cfg.MaxSteeringAngle(),
		RobotServoMaxAngleDeg: cfg.Steering.ServoMaxAngleDeg,
		RobotMaxWheelAngleDeg: cfg.Steering.MaxWheelAngleDeg,
		RobotLinkageRatio:     cfg.LinkageRatio(),

		RobotChassisMass: cfg.Chassis.Mass,
		RobotWheelMass:   cfg.Wheel.Mass,

		RobotMaxSpeedMPS:    cfg.Drivetrain.MaxSpeedMPS,
		RobotMaxAccelMPS2:   cfg.Drivetrain.MaxAccelMPS2,
		RobotRearSteerRatio: cfg.Drivetrain.RearSteerRatio,

		RobotLidarMountXOffset:      cfg.Lidar.MountXOffset,
		RobotLidarMountZOffset:      cfg.Lidar.MountZOffset,
		RobotLidarInverted:          cfg.Lidar.Inverted,
		RobotLidarMountYawOffsetDeg: cfg.Lidar.MountYawOffsetDeg,

		RobotImuMountZOffset: cfg.Imu.MountZOffset,

		RobotCameraMountXOffset: cfg.Camera.MountXOffset,
		RobotCameraMountZOffset: cfg.Camera.MountZOffset,
		RobotCameraPitchRad:     cfg.Camera.MountPitch,
	}, nil
}
