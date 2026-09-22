package boardloop

import (
	"errors"
	"fmt"
	"math"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/actuation"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/servo"
)

// session is a validated boardlink.Config in the types the loop works in.
type session struct {
	wire           boardlink.Config
	steering       actuation.SteeringConfig
	pulse          servo.Pulse
	speedScale     float64
	commandTimeout time.Duration
	statusEvery    time.Duration
	odometryEvery  time.Duration
}

// Servo pulse bounds a Config must fall within. A hobby servo is driven
// between roughly 500 and 2500 us; the bounds leave margin either side and
// still reject a value in the wrong unit (seconds, milliseconds) or a
// pulse that would fill most of the 20 ms frame.
const (
	MinServoPulseUS = 100.0
	MaxServoPulseUS = 3000.0
)

// ErrInvalidConfig is wrapped by every error ValidateConfig returns.
var ErrInvalidConfig = errors.New("boardloop: invalid config")

// ValidateConfig reports whether the board can apply c: the steering
// conversion is valid (actuation.SteeringConfig.Validate), the speed scale
// is positive and finite, the servo pulse calibration is ordered and within
// [MinServoPulseUS, MaxServoPulseUS] with a positive travel, and the
// command timeout, hardware watchdog and status interval are positive. An
// OdometryIntervalMS of 0 is valid and turns odometry off.
func ValidateConfig(c boardlink.Config) error {
	_, err := newSession(c)
	return err
}

// newSession validates c and converts it.
func newSession(c boardlink.Config) (session, error) {
	steering := actuation.SteeringConfig{
		LinkageRatio:     float64(c.LinkageRatio),
		ServoMaxAngleDeg: float64(c.ServoMaxAngleDeg),
		OffsetDeg:        float64(c.SteeringOffsetDeg),
	}
	if err := steering.Validate(); err != nil {
		return session{}, fmt.Errorf("%w: %w", ErrInvalidConfig, err)
	}
	pulse := servo.Pulse{
		MinPulseUS:    float64(c.ServoMinPulseUS),
		MaxPulseUS:    float64(c.ServoMaxPulseUS),
		CenterPulseUS: float64(c.ServoCenterPulseUS),
		RangeDeg:      float64(c.ServoRangeDeg),
		Reversed:      c.ServoReversed,
	}
	if err := validatePulse(pulse); err != nil {
		return session{}, err
	}
	scale := float64(c.SpeedScalePctPerMPS)
	if !(scale > 0) || math.IsInf(scale, 0) {
		return session{}, fmt.Errorf("%w: speed scale %v must be positive and finite", ErrInvalidConfig, scale)
	}
	switch {
	case c.CommandTimeoutMS == 0:
		return session{}, fmt.Errorf("%w: command timeout must be positive", ErrInvalidConfig)
	case c.HardwareWatchdogMS == 0:
		return session{}, fmt.Errorf("%w: hardware watchdog must be positive", ErrInvalidConfig)
	case c.StatusIntervalMS == 0:
		return session{}, fmt.Errorf("%w: status interval must be positive", ErrInvalidConfig)
	}
	return session{
		wire:           c,
		steering:       steering,
		pulse:          pulse,
		speedScale:     scale,
		commandTimeout: time.Duration(c.CommandTimeoutMS) * time.Millisecond,
		statusEvery:    time.Duration(c.StatusIntervalMS) * time.Millisecond,
		odometryEvery:  time.Duration(c.OdometryIntervalMS) * time.Millisecond,
	}, nil
}

// validatePulse checks what servo.PulseUS assumes, plus the sanity bounds.
// The comparisons are written so that NaN fails every one of them.
func validatePulse(p servo.Pulse) error {
	if !(p.MinPulseUS >= MinServoPulseUS && p.MaxPulseUS <= MaxServoPulseUS) {
		return fmt.Errorf("%w: servo pulse range [%v, %v] us outside [%v, %v]",
			ErrInvalidConfig, p.MinPulseUS, p.MaxPulseUS, MinServoPulseUS, MaxServoPulseUS)
	}
	if !(p.MinPulseUS < p.MaxPulseUS) {
		return fmt.Errorf("%w: servo min pulse %v us must be below max %v us",
			ErrInvalidConfig, p.MinPulseUS, p.MaxPulseUS)
	}
	if !(p.CenterPulseUS >= p.MinPulseUS && p.CenterPulseUS <= p.MaxPulseUS) {
		return fmt.Errorf("%w: servo center pulse %v us outside [%v, %v]",
			ErrInvalidConfig, p.CenterPulseUS, p.MinPulseUS, p.MaxPulseUS)
	}
	if !(p.RangeDeg > 0) || math.IsInf(p.RangeDeg, 0) {
		return fmt.Errorf("%w: servo range %v deg must be positive and finite", ErrInvalidConfig, p.RangeDeg)
	}
	return nil
}
