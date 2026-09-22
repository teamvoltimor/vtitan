package picolink

import (
	"fmt"
	"math"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/pkg/driver/servo"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/actuation"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardloop"
)

// Profile is what the board's Config and the host's odometry conversion are
// built from, resolved from the hardware profile by LoadProfile. Each field
// is the value the Zero resolves for the same purpose, from the same file.
type Profile struct {
	// Steering is internal/node/motor.SteeringFor: robot.toml's linkage ratio
	// and steering.servo_max_angle_deg, motors.toml's steering.offset.
	Steering actuation.SteeringConfig
	// Servo is hwconfig.Servo: servo.toml's pulse range, range_deg and
	// reversed, with the servo profile overlaid.
	Servo servo.Config
	// SpeedScalePercentPerMPS is internal/node/motor.SpeedScaleFor:
	// motors.toml's drive.speed_scale.
	SpeedScalePercentPerMPS float64
	// Encoder is nil when no encoder profile resolves (hwconfig.Encoder
	// failed). Odometry is then not published, as on the Zero.
	Encoder *EncoderParams
}

// EncoderParams is the part of the encoder profile the host needs to turn
// the board's raw counts into JointStates.
type EncoderParams struct {
	// CountsPerRev is encoder.toml's counts_per_rev, from the motor profile.
	CountsPerRev float64
	// Invert flips the count's sign into the command frame, as
	// encoder.Config.Invert does on the Zero.
	Invert bool
}

// Board timing the host chooses and sends in every Config. The firmware
// keeps no compiled-in tuning (boardlink's doc), so these live here.
const (
	// HardwareWatchdogMS is the RP2350 hardware watchdog period. Half of the
	// Zero's 500 ms command timeout: a hung firmware is reset, and its pins
	// released to their safe pull-downs, before a healthy one would have
	// stopped on command silence. Well above any control-loop iteration, and
	// far below the RP2350's ~16.7 s ceiling.
	HardwareWatchdogMS = 250
	// StatusIntervalMS is how often the board reports Status: 20 Hz, the
	// navigator's DefaultRateHz, which is the cadence the Zero publishes
	// MotorStatus at (once per command).
	StatusIntervalMS = 50
	// OdometryIntervalMS is how often the board reports Odometry: 50 Hz,
	// internal/node/motor.DefaultFeedbackInterval, so JointStates arrives at
	// the rate internal/nav/bayexit already expects from the Zero.
	OdometryIntervalMS = 20
)

// BoardConfig builds the Config the board is sent when a session starts and
// on every Hello.
// invertDrive and commandTimeout are the Pi 5's equivalents of the Zero's
// --motor-invert and --motor-command-timeout flags; everything else comes
// from p.
//
// It errors rather than truncating a command timeout the wire's uint16
// milliseconds cannot carry, or one that rounds to zero, and on anything
// boardloop.ValidateConfig (the rule the board itself applies) would refuse:
// a refused Config leaves the board unconfigured, so it is better caught
// here, with the reason, than seen as a Hello that keeps coming back.
func BoardConfig(p Profile, invertDrive bool, commandTimeout time.Duration) (boardlink.Config, error) {
	timeoutMS := commandTimeout.Milliseconds()
	if timeoutMS < 1 || timeoutMS > math.MaxUint16 {
		return boardlink.Config{}, fmt.Errorf(
			"picolink: command timeout %v must be between 1 ms and %d ms", commandTimeout, math.MaxUint16,
		)
	}
	c := boardlink.Config{
		CommandTimeoutMS:    uint16(timeoutMS),
		SpeedScalePctPerMPS: float32(p.SpeedScalePercentPerMPS),
		InvertDrive:         invertDrive,
		LinkageRatio:        float32(p.Steering.LinkageRatio),
		ServoMaxAngleDeg:    float32(p.Steering.ServoMaxAngleDeg),
		SteeringOffsetDeg:   float32(p.Steering.OffsetDeg),
		ServoMinPulseUS:     float32(p.Servo.MinPulseUS),
		ServoMaxPulseUS:     float32(p.Servo.MaxPulseUS),
		ServoCenterPulseUS:  float32(p.Servo.CenterPulseUS),
		ServoRangeDeg:       float32(p.Servo.RangeDeg),
		ServoReversed:       p.Servo.Reversed,
		HardwareWatchdogMS:  HardwareWatchdogMS,
		StatusIntervalMS:    StatusIntervalMS,
		OdometryIntervalMS:  OdometryIntervalMS,
	}
	if err := boardloop.ValidateConfig(c); err != nil {
		return boardlink.Config{}, fmt.Errorf("picolink: %w", err)
	}
	return c, nil
}
