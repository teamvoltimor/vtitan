package bayexit

import (
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/corridorfollower"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/parking"
)

// Config is BayExit's tuning, bundling corridorfollower.Config (which owns
// every BAY_EXIT_* field, matching Python's tuning.corridor_follower) with
// the robot-geometry and parking-lot constants BayExit reads from
// neighboring sections, matching bay_exit.py's own imports of RobotSpecs,
// ParkingLotSpecs, tuning.pursuit.MAX_STEERING_RATE and
// tuning.control.CONTROL_HZ.
type Config struct {
	// Follower carries every BayExit* field plus MinForwardClearanceM
	// (is_clear's threshold), ForwardArcHalfFovRad/MinValidRangeM (the
	// forward-clearance cone), and CornerSpeedScale/ReverseSpeedScale.
	Follower corridorfollower.Config

	// WheelbaseM is RobotSpecs.WHEELBASE, the axle spacing (not the
	// effective turning wheelbase -- see EffectiveWheelbaseM).
	WheelbaseM float64
	// RearSteerRatio is RobotSpecs.REAR_STEER_RATIO: the rear axle steers
	// counter-phase, so the instantaneous turn centre sits between the
	// axles rather than at the rear axle.
	RearSteerRatio float64
	// YawGain is RobotSpecs.YAW_GAIN, the fraction of the geometric yaw
	// rate the real chassis achieves (calibrated against bag data).
	YawGain float64
	// ChassisLengthM/ChassisWidthM are RobotSpecs.LENGTH/WIDTH, for the
	// clearance guard's swept-rectangle model.
	ChassisLengthM float64
	ChassisWidthM  float64
	// MaxSteeringAngleRad is the road-wheel angle at full lock (in
	// radians), matching RobotSpecs.MAX_WHEEL_ANGLE_DEG converted --
	// reused from corridorfollower.Config.MaxSteeringAngleRad rather than
	// duplicated, since that field is already profile-sourced there.

	// ParkingLot is ParkingLotSpecs: the two fins' geometry for the
	// clearance guard's dead-reckoned model.
	ParkingLot parking.ParkingLotSpecs

	// MaxSteeringRateRadPerS is tuning.pursuit.MAX_STEERING_RATE: the
	// servo's modelled slew rate, used to model the standstill a leg
	// change costs and the clearance guard's predicted pose.
	MaxSteeringRateRadPerS float64
	// ControlHz is tuning.control.CONTROL_HZ: ticks per second, for
	// converting a slew rate into a per-tick angle step.
	ControlHz float64
}

// DefaultConfig returns the Config matching the shipped defaults: every
// BayExit* field from corridorfollower.DefaultConfig, RobotSpecs' shipped
// constants (robot.toml), and ParkingLotSpecs' shipped constants
// (track.toml's [parking]).
func DefaultConfig() Config {
	return Config{
		Follower: corridorfollower.DefaultConfig(),

		WheelbaseM:     DefaultWheelbaseM,
		RearSteerRatio: DefaultRearSteerRatio,
		YawGain:        DefaultYawGain,
		ChassisLengthM: DefaultChassisLengthM,
		ChassisWidthM:  DefaultChassisWidthM,

		ParkingLot: parking.DefaultParkingLotSpecs,

		MaxSteeringRateRadPerS: DefaultMaxSteeringRateRadPerS,
		ControlHz:              DefaultControlHz,
	}
}

// Shipped defaults, matching platform/shared/config/robot.toml and
// internal/nav/controllers.DefaultConfig's own mirrors of the same values.
const (
	// DefaultWheelbaseM matches robot.toml's [ackermann] wheelbase.
	DefaultWheelbaseM = 0.19
	// DefaultRearSteerRatio matches robot.toml's rear_steer_ratio.
	DefaultRearSteerRatio = 1.0
	// DefaultYawGain matches robot.toml's yaw_gain, calibrated against bag
	// data 2026-08-29.
	DefaultYawGain = 0.55
	// DefaultChassisLengthM/DefaultChassisWidthM match robot.toml's
	// [chassis] length/width.
	DefaultChassisLengthM = 0.30
	DefaultChassisWidthM  = 0.194
	// DefaultMaxSteeringRateRadPerS matches
	// controllers.DefaultMaxSteeringRate.
	DefaultMaxSteeringRateRadPerS = 1.2
	// DefaultControlHz matches controllers.DefaultControlHz.
	DefaultControlHz = 20.0
)

// EffectiveWheelbaseM is the wheelbase the chassis actually turns about, not
// the axle spacing, matching bay_exit.py's _EFFECTIVE_WHEELBASE_M.
//
// The rear axle steers counter-phase, so the instantaneous turn centre sits
// between the axles: wheelbase / (1 + rear_steer_ratio), which at the
// shipped ratio of 1.0 is HALF the wheelbase.
func (c Config) EffectiveWheelbaseM() float64 {
	return c.WheelbaseM / (1.0 + c.RearSteerRatio)
}
