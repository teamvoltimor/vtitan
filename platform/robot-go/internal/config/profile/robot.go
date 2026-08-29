package profile

import (
	"fmt"
	"math"
)

// RobotChassis mirrors robot.toml's [chassis] section.
type RobotChassis struct {
	Length float64 `mapstructure:"length"`
	Width  float64 `mapstructure:"width"`
	Height float64 `mapstructure:"height"`
	Mass   float64 `mapstructure:"mass"`
}

// RobotAckermann mirrors robot.toml's [ackermann] section.
type RobotAckermann struct {
	Wheelbase  float64 `mapstructure:"wheelbase"`
	TrackWidth float64 `mapstructure:"track_width"`
}

// RobotSteering mirrors robot.toml's [steering] section. ServoMaxAngleDeg
// and MaxWheelAngleDeg are deliberately absent from the base robot.toml --
// they describe a specific servo, so they're required from an active
// hardware profile (see LoadRobotConfig).
type RobotSteering struct {
	ServoMaxAngleDeg float64 `mapstructure:"servo_max_angle_deg"`
	MaxWheelAngleDeg float64 `mapstructure:"max_wheel_angle_deg"`
	// SteeringLimitDeg optionally caps the road-wheel angle below
	// MaxWheelAngleDeg -- 0 means unset (use MaxWheelAngleDeg), matching
	// RobotConstants.Steering.steering_limit_deg's `None` default.
	SteeringLimitDeg float64 `mapstructure:"steering_limit_deg"`
}

// RobotWheel mirrors robot.toml's [wheel] section.
type RobotWheel struct {
	Radius float64 `mapstructure:"radius"`
	Width  float64 `mapstructure:"width"`
	Mass   float64 `mapstructure:"mass"`
}

// RobotDrivetrain mirrors robot.toml's [drivetrain] section. MaxSpeedMPS is
// deliberately absent from the base robot.toml -- it describes a specific
// motor, so it's required from an active hardware profile (see
// LoadRobotConfig).
type RobotDrivetrain struct {
	MaxSpeedMPS    float64 `mapstructure:"max_speed_mps"`
	MaxAccelMPS2   float64 `mapstructure:"max_accel_mps2"`
	RearSteerRatio float64 `mapstructure:"rear_steer_ratio"`
}

// RobotLidar mirrors robot.toml's [lidar] section.
type RobotLidar struct {
	MountXOffset float64 `mapstructure:"mount_x_offset"`
	MountZOffset float64 `mapstructure:"mount_z_offset"`
	// Inverted -- true when the LIDAR is mounted upside-down, driving a
	// mandatory 180deg correction (see RobotConfig.LidarYawOffsetRad).
	Inverted bool `mapstructure:"inverted"`
	// MountYawOffsetDeg -- residual mount miscalibration added on top of
	// Inverted's 180deg, not a replacement for it.
	MountYawOffsetDeg float64 `mapstructure:"mount_yaw_offset_deg"`
	MinRange          float64 `mapstructure:"min_range"`
	MaxRange          float64 `mapstructure:"max_range"`
	Samples           int     `mapstructure:"samples"`
	UpdateRate        float64 `mapstructure:"update_rate"`
	NoiseStddev       float64 `mapstructure:"noise_stddev"`
	Diameter          float64 `mapstructure:"diameter"`
	Height            float64 `mapstructure:"height"`
}

// RobotImu mirrors robot.toml's [imu] section.
type RobotImu struct {
	MountZOffset float64    `mapstructure:"mount_z_offset"`
	UpdateRate   float64    `mapstructure:"update_rate"`
	GyroNoise    float64    `mapstructure:"gyro_noise"`
	AccelNoise   float64    `mapstructure:"accel_noise"`
	Mass         float64    `mapstructure:"mass"`
	Size         [3]float64 `mapstructure:"size"`
}

// RobotCamera mirrors robot.toml's [camera] section.
type RobotCamera struct {
	MountXOffset float64 `mapstructure:"mount_x_offset"`
	MountZOffset float64 `mapstructure:"mount_z_offset"`
	MountPitch   float64 `mapstructure:"mount_pitch"`
	Hfov         float64 `mapstructure:"hfov"`
	Width        int     `mapstructure:"width"`
	Height       int     `mapstructure:"height"`
	UpdateRate   float64 `mapstructure:"update_rate"`
	NearClip     float64 `mapstructure:"near_clip"`
	FarClip      float64 `mapstructure:"far_clip"`
}

// RobotConfig mirrors
// shared.config.robot_constants.RobotConstants's full schema, as loaded
// from platform/shared/config/robot.toml.
type RobotConfig struct {
	Chassis    RobotChassis    `mapstructure:"chassis"`
	Ackermann  RobotAckermann  `mapstructure:"ackermann"`
	Steering   RobotSteering   `mapstructure:"steering"`
	Wheel      RobotWheel      `mapstructure:"wheel"`
	Drivetrain RobotDrivetrain `mapstructure:"drivetrain"`
	Lidar      RobotLidar      `mapstructure:"lidar"`
	Imu        RobotImu        `mapstructure:"imu"`
	Camera     RobotCamera     `mapstructure:"camera"`
}

// DefaultRobotTOMLPath is platform/shared/config/robot.toml, relative to the
// repo root.
const DefaultRobotTOMLPath = "platform/shared/config/robot.toml"

// degToRadTurn is a half-turn in degrees -- used to convert both
// MaxSteeringAngle's and LidarYawOffsetRad's degree inputs to radians, and
// as the mandatory correction LidarYawOffsetRad adds for an inverted LIDAR
// mount.
const degToRadTurn = 180.0

// requiredRobotKeys are the fields robot.toml deliberately omits and which
// must come from an active hardware profile, matching
// RobotConstants._require_component_facts().
var requiredRobotKeys = []string{
	"drivetrain.max_speed_mps",
	"steering.servo_max_angle_deg",
	"steering.max_wheel_angle_deg",
}

// LoadRobotConfig loads RobotConfig from basePath overlaid with
// profileNames (see Load), then enforces requiredRobotKeys the same way
// RobotConstants._require_component_facts() does: an error naming the
// missing key and the active profiles, not a silently-zero field.
func LoadRobotConfig(basePath string, profileNames []string) (*RobotConfig, error) {
	v, err := merge(basePath, profileNames, nil)
	if err != nil {
		return nil, err
	}

	for _, key := range requiredRobotKeys {
		if !v.IsSet(key) {
			return nil, fmt.Errorf(
				"profile: robot.toml requires %s from an active hardware profile (%s); active profiles: %v",
				key, EnvVar, profileNames,
			)
		}
	}

	var cfg RobotConfig
	if unmarshalErr := v.Unmarshal(&cfg); unmarshalErr != nil {
		return nil, fmt.Errorf("profile: unmarshaling merged config: %w", unmarshalErr)
	}
	return &cfg, nil
}

// LinkageRatio mirrors RobotConstants.Steering.linkage_ratio: the servo's
// travel is amplified/reduced by this factor to reach the road wheel.
func (c *RobotConfig) LinkageRatio() float64 {
	return c.Steering.MaxWheelAngleDeg / c.Steering.ServoMaxAngleDeg
}

// MaxSteeringAngle mirrors RobotConstants.Steering.max_steering_angle:
// SteeringLimitDeg if set (nonzero), else MaxWheelAngleDeg, in radians.
func (c *RobotConfig) MaxSteeringAngle() float64 {
	limitDeg := c.Steering.SteeringLimitDeg
	if limitDeg == 0 {
		limitDeg = c.Steering.MaxWheelAngleDeg
	}
	return limitDeg * math.Pi / degToRadTurn
}

// LidarYawOffsetRad mirrors
// shared.config.constants.robot.RobotSpecs.lidar_yaw_offset_rad(): rotates
// raw scan bearings into the robot frame (0 rad = forward) by combining the
// mandatory 180deg for an upside-down mount with any residual
// miscalibration, in that order, not interchangeably.
func (c *RobotConfig) LidarYawOffsetRad() float64 {
	invertedDeg := 0.0
	if c.Lidar.Inverted {
		invertedDeg = degToRadTurn
	}
	return (invertedDeg + c.Lidar.MountYawOffsetDeg) * math.Pi / degToRadTurn
}

// LidarToFrontBumper mirrors RobotSpecs.LIDAR_TO_FRONT_BUMPER: meters from
// the sensor to the front bumper face.
func (c *RobotConfig) LidarToFrontBumper() float64 {
	return c.Chassis.Length/2 - c.Lidar.MountXOffset
}

// LidarToRearBumper mirrors RobotSpecs.LIDAR_TO_REAR_BUMPER: meters from
// the sensor to the rear bumper face.
func (c *RobotConfig) LidarToRearBumper() float64 {
	return c.Chassis.Length/2 + c.Lidar.MountXOffset
}
