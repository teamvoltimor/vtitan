package profile

import (
	"fmt"
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"
)

// RobotConfig is the value-based view of robot.toml the rest of Go reads.
//
// It carries no `mapstructure` tags and no field-to-key knowledge: the keys are
// defined once, by generated.RobotConfig (from src/model/robot.schema.json),
// which LoadRobotConfig decodes. This view exists only so callers get plain
// values and the derived accessors below instead of walking the generated
// pointer fields.
type RobotConfig struct {
	Chassis    RobotChassis
	Ackermann  RobotAckermann
	Steering   RobotSteering
	Wheel      RobotWheel
	Drivetrain RobotDrivetrain
	Lidar      RobotLidar
	Imu        RobotImu
	Camera     RobotCamera
}

// RobotChassis mirrors robot.toml's [chassis] section.
type RobotChassis struct {
	Length float64
	Width  float64
	Height float64
	Mass   float64
}

// RobotAckermann mirrors robot.toml's [ackermann] section.
type RobotAckermann struct {
	Wheelbase  float64
	TrackWidth float64
}

// RobotSteering mirrors robot.toml's [steering] section. ServoMaxAngleDeg and
// MaxWheelAngleDeg are deliberately absent from the base robot.toml -- they
// describe a specific servo, so they're required from an active hardware
// profile (see LoadRobotConfig).
type RobotSteering struct {
	ServoMaxAngleDeg float64
	MaxWheelAngleDeg float64
	// SteeringLimitDeg optionally caps the road-wheel angle below
	// MaxWheelAngleDeg -- 0 means unset (use MaxWheelAngleDeg), matching
	// RobotConstants.Steering.steering_limit_deg's `None` default.
	SteeringLimitDeg float64
}

// RobotWheel mirrors robot.toml's [wheel] section.
type RobotWheel struct {
	Radius float64
	Width  float64
	Mass   float64
}

// RobotDrivetrain mirrors robot.toml's [drivetrain] section. MaxSpeedMPS,
// MaxAccelMPS2 and SpeedResponseTauS are deliberately absent from the base
// robot.toml -- they describe a specific motor, so they're required from an
// active hardware profile (see LoadRobotConfig).
type RobotDrivetrain struct {
	MaxSpeedMPS       float64
	MaxAccelMPS2      float64
	SpeedResponseTauS float64
	RearSteerRatio    float64
	YawGain           float64
	// MinTurnRadiusM matches RobotSpecs.MIN_TURN_RADIUS_M: the curvature
	// floor a bicycle model alone does not have, calibrated against a
	// measured chassis saturation.
	MinTurnRadiusM float64
}

// RobotLidar mirrors robot.toml's [lidar] section.
type RobotLidar struct {
	MountXOffset float64
	MountZOffset float64
	// Inverted -- true when the LIDAR is mounted upside-down, driving a
	// mandatory 180deg correction (see lidar.correctAngleDeg).
	Inverted bool
	// MountYawOffsetDeg -- residual mount miscalibration added on top of
	// Inverted's 180deg, not a replacement for it.
	MountYawOffsetDeg float64
	MinRange          float64
	MaxRange          float64
	Samples           int
	UpdateRate        float64
	NoiseStddev       float64
	Diameter          float64
	Height            float64
}

// RobotImu mirrors robot.toml's [imu] section.
type RobotImu struct {
	MountZOffset float64
	UpdateRate   float64
	GyroNoise    float64
	AccelNoise   float64
	Mass         float64
	Size         [3]float64
}

// RobotCamera mirrors robot.toml's [camera] section.
type RobotCamera struct {
	MountXOffset float64
	MountZOffset float64
	MountPitch   float64
	Hfov         float64
	Width        int
	Height       int
	UpdateRate   float64
	NearClip     float64
	FarClip      float64
}

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

// LoadRobotConfig loads robot.toml from basePath overlaid with profileNames,
// decodes it into the generated RobotConfig DTO, and returns the value view.
//
// It enforces requiredRobotKeys the same way
// RobotConstants._require_component_facts() does: an error naming the missing
// key and the active profiles, not a silently-zero field.
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

	var dto generated.RobotConfig
	if unmarshalErr := v.Unmarshal(&dto); unmarshalErr != nil {
		return nil, fmt.Errorf("profile: unmarshaling merged config: %w", unmarshalErr)
	}
	return robotConfigFromDTO(&dto), nil
}

// LoadRobotValues is LoadRobotConfig without the requiredRobotKeys check, for
// callers that read only base-file facts (e.g. the LIDAR mount correction)
// and must not have a servo/motor profile decide whether they resolve.
func LoadRobotValues(basePath string, profileNames []string) (*RobotConfig, error) {
	v, err := merge(basePath, profileNames, nil)
	if err != nil {
		return nil, err
	}
	var dto generated.RobotConfig
	if unmarshalErr := v.Unmarshal(&dto); unmarshalErr != nil {
		return nil, fmt.Errorf("profile: unmarshaling merged config: %w", unmarshalErr)
	}
	return robotConfigFromDTO(&dto), nil
}

// deref reads a generated optional field, yielding its zero value when unset.
func deref[T any](p *T) T {
	if p == nil {
		var zero T
		return zero
	}
	return *p
}

// robotConfigFromDTO flattens the generated pointer DTO into the value view.
func robotConfigFromDTO(d *generated.RobotConfig) *RobotConfig {
	chassis := deref(d.Chassis)
	ackermann := deref(d.Ackermann)
	steering := deref(d.Steering)
	wheel := deref(d.Wheel)
	drivetrain := deref(d.Drivetrain)
	lidar := deref(d.Lidar)
	imu := deref(d.Imu)
	camera := deref(d.Camera)

	imuSize := [3]float64{}
	for i := range imuSize {
		if i < len(imu.Size) {
			imuSize[i] = imu.Size[i]
		}
	}

	return &RobotConfig{
		Chassis: RobotChassis{
			Length: deref(chassis.Length),
			Width:  deref(chassis.Width),
			Height: deref(chassis.Height),
			Mass:   deref(chassis.Mass),
		},
		Ackermann: RobotAckermann{
			Wheelbase:  deref(ackermann.Wheelbase),
			TrackWidth: deref(ackermann.TrackWidth),
		},
		Steering: RobotSteering{
			ServoMaxAngleDeg: deref(steering.ServoMaxAngleDeg),
			MaxWheelAngleDeg: deref(steering.MaxWheelAngleDeg),
			SteeringLimitDeg: deref(steering.SteeringLimitDeg),
		},
		Wheel: RobotWheel{
			Radius: deref(wheel.Radius),
			Width:  deref(wheel.Width),
			Mass:   deref(wheel.Mass),
		},
		Drivetrain: RobotDrivetrain{
			MaxSpeedMPS:       deref(drivetrain.MaxSpeedMps),
			MaxAccelMPS2:      deref(drivetrain.MaxAccelMps2),
			SpeedResponseTauS: deref(drivetrain.SpeedResponseTauS),
			RearSteerRatio:    deref(drivetrain.RearSteerRatio),
			YawGain:           deref(drivetrain.YawGain),
			MinTurnRadiusM:    deref(drivetrain.MinTurnRadiusM),
		},
		Lidar: RobotLidar{
			MountXOffset:      deref(lidar.MountXOffset),
			MountZOffset:      deref(lidar.MountZOffset),
			Inverted:          deref(lidar.Inverted),
			MountYawOffsetDeg: deref(lidar.MountYawOffsetDeg),
			MinRange:          deref(lidar.MinRange),
			MaxRange:          deref(lidar.MaxRange),
			Samples:           deref(lidar.Samples),
			UpdateRate:        deref(lidar.UpdateRate),
			NoiseStddev:       deref(lidar.NoiseStddev),
			Diameter:          deref(lidar.Diameter),
			Height:            deref(lidar.Height),
		},
		Imu: RobotImu{
			MountZOffset: deref(imu.MountZOffset),
			UpdateRate:   deref(imu.UpdateRate),
			GyroNoise:    deref(imu.GyroNoise),
			AccelNoise:   deref(imu.AccelNoise),
			Mass:         deref(imu.Mass),
			Size:         imuSize,
		},
		Camera: RobotCamera{
			MountXOffset: deref(camera.MountXOffset),
			MountZOffset: deref(camera.MountZOffset),
			MountPitch:   deref(camera.MountPitch),
			Hfov:         deref(camera.Hfov),
			Width:        deref(camera.Width),
			Height:       deref(camera.Height),
			UpdateRate:   deref(camera.UpdateRate),
			NearClip:     deref(camera.NearClip),
			FarClip:      deref(camera.FarClip),
		},
	}
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
// correct while swapping front and back. Consumers do not need a replacement:
// the correction is applied once in the driver, from Lidar.Inverted and
// Lidar.MountYawOffsetDeg, so scans are already in the robot frame here.

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
