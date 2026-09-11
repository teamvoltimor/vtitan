package profile

import (
	"fmt"
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"
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

// RobotDrivetrain mirrors robot.toml's [drivetrain] section. MaxSpeedMPS,
// MaxAccelMPS2 and SpeedResponseTauS are deliberately absent from the base
// robot.toml -- they describe a specific motor, so they're required from an
// active hardware profile (see LoadRobotConfig).
type RobotDrivetrain struct {
	MaxSpeedMPS       float64 `mapstructure:"max_speed_mps"`
	MaxAccelMPS2      float64 `mapstructure:"max_accel_mps2"`
	SpeedResponseTauS float64 `mapstructure:"speed_response_tau_s"`
	RearSteerRatio    float64 `mapstructure:"rear_steer_ratio"`
	YawGain           float64 `mapstructure:"yaw_gain"`
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
	// Device is the V4L2 device node for direct hardware capture (e.g.
	// "/dev/video0" for the CSI camera module 3 on a Pi 5). It is NOT hardcoded:
	// it comes from robot.toml so a different board or a simulated source can
	// override it. Empty when Source is "topic" (frames arrive over NATS).
	Device string `mapstructure:"device"`
	// Source selects the capture backend: "v4l2" (direct CSI capture, the
	// default on hardware), "topic" (subscribe to vtitan.sensor.v1.camera, the
	// Python camera_source="topic" path), or "synthetic" (test pattern, bench).
	Source string `mapstructure:"source"`
}

// RobotConfig mirrors
// shared.config.robot_constants.RobotConstants's full schema, as loaded
// from src/config/robot.toml.
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

// DefaultRobotTOMLPath is src/config/robot.toml, relative to the
// repo root.
const DefaultRobotTOMLPath = "src/config/robot.toml"

// requiredRobotKeys are the fields robot.toml deliberately omits and which
// must come from an active hardware profile, matching
// RobotConstants._require_component_facts().
var requiredRobotKeys = []string{
	"drivetrain.max_speed_mps",
	"drivetrain.max_accel_mps2",
	"drivetrain.speed_response_tau_s",
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
				key,
				EnvVar,
				profileNames,
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
	return limitDeg * math.Pi / navutil.DegreesPerHalfTurn
}

// LidarYawOffsetRad was removed deliberately: an upside-down mount reverses
// the sensor's apparent spin direction, which no single additive offset can
// express -- see lidar.correctAngleDeg, whose comment records the 2026-08-31
// eight-bearing hardware test that found a constant +180 leaves left/right
// correct while swapping front and back. Returning one scalar invited every
// caller to add it and believe the frame was corrected.
//
// Consumers do not need a replacement. The correction is applied once in the
// driver, from Lidar.Inverted/Lidar.MountYawOffsetDeg below, so scans are
// already in the robot frame by the time anything else sees them.

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
