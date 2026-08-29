package kinematics

import (
	"math"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
)

// Params configures an AckermannKinematics integrator. Unlike the Python
// constructor, which pulls unset fields from a shared tuning/RobotSpecs
// context when the caller omits them, every field here is explicit: the
// caller assembles it from profile.RobotConfig (WheelbaseM, MaxSteerRad,
// MaxAccelMPS2, SpeedTauS, MaxSpeedMPS, RearSteerRatio, YawGain) and
// kinematics.Config (MaxSteerRateRadPerS), rather than this package
// re-deriving RobotSpecs constants itself.
type Params struct {
	WheelbaseM          float64
	MaxSteerRad         float64
	MaxSteerRateRadPerS float64
	MaxAccelMPS2        float64
	MaxSpeedMPS         float64
	RearSteerRatio      float64
	// SpeedTauS is the first-order lag between a commanded speed and the
	// achieved one (s), from RobotDrivetrain.SpeedResponseTauS. Zero
	// disables the lag, reproducing the pre-2026-08-29 model.
	SpeedTauS float64
	// YawGain is the fraction of the modelled yaw rate the chassis
	// actually delivers, from RobotDrivetrain.YawGain -- the tyre slip
	// the zero-slip geometry above has no term for. Measured 0.55 on
	// 2026-08-29; before it existed the sim cornered 1.83x harder than
	// the car it modelled.
	YawGain float64
	// Substeps is the number of sub-integration steps per Step call.
	// DefaultSubsteps is used if this is <= 0.
	Substeps int
}

// AckermannKinematics is a sub-stepped counter-phase four-wheel-steer
// integrator with hardware rate limits, matching
// src.simulation.kinematics.AckermannKinematics.
type AckermannKinematics struct {
	wheelbase        float64
	maxSteer         float64
	maxSteerRate     float64
	maxAccel         float64
	maxSpeed         float64
	rearSteerRatio   float64
	speedTauS        float64
	yawGain          float64
	substeps         int
	turnReferenceLen float64
}

// DefaultSubsteps matches AckermannKinematics.__init__'s substeps=5.
const DefaultSubsteps = 5

// oneWheelDrivenRatio is the "1" in "1 + rear_steer_ratio": a front-steer
// front axle always contributes its full angle to the turn, regardless of
// how much the rear axle counter-steers.
const oneWheelDrivenRatio = 1.0

// fullyClosedFraction caps the per-substep lag step at "close the whole
// remaining gap". Without it a substep longer than the time constant
// overshoots the setpoint and oscillates, which a first-order lag never
// does.
const fullyClosedFraction = 1.0

// NewAckermannKinematics builds an integrator from p.
func NewAckermannKinematics(p Params) *AckermannKinematics {
	substeps := p.Substeps
	if substeps <= 0 {
		substeps = DefaultSubsteps
	}
	return &AckermannKinematics{
		wheelbase:      p.WheelbaseM,
		maxSteer:       p.MaxSteerRad,
		maxSteerRate:   p.MaxSteerRateRadPerS,
		maxAccel:       p.MaxAccelMPS2,
		maxSpeed:       p.MaxSpeedMPS,
		rearSteerRatio: p.RearSteerRatio,
		speedTauS:      p.SpeedTauS,
		yawGain:        p.YawGain,
		substeps:       substeps,
		// Effective turn length: the yaw rate is v/L_eff * tan(steer).
		// Front-only steering pivots about the rear axle (L_eff = L);
		// counter-phase steering with equal angles pivots about the
		// chassis center (L_eff = L/2), i.e. twice the yaw rate for the
		// same steering angle.
		turnReferenceLen: p.WheelbaseM / (oneWheelDrivenRatio + math.Abs(p.RearSteerRatio)),
	}
}

// Step advances state by dt under a controller command, matching
// AckermannKinematics.step.
//
// targetSpeed is the commanded linear speed (m/s); may be negative
// (reverse). targetSteerNorm is the commanded steering in the
// controller's normalised [-1, 1] range (as published in
// Velocity.angular). dt is the control interval (seconds).
func (k *AckermannKinematics) Step(state AckermannState, targetSpeed, targetSteerNorm, dt float64) AckermannState {
	const steerNormLimit = 1.0
	targetSteer := clamp(targetSteerNorm, -steerNormLimit, steerNormLimit) * k.maxSteer

	x, y, yaw := state.X, state.Y, state.Yaw
	v, steer := state.V, state.Steer

	h := dt / float64(k.substeps)
	for i := 0; i < k.substeps; i++ {
		// Servo steering slew toward the target angle.
		steer = approach(steer, targetSteer, k.maxSteerRate*h)
		steer = clamp(steer, -k.maxSteer, k.maxSteer)
		// Drive response: a first-order lag toward the setpoint, then
		// the acceleration clamp on top. Ordered that way because they
		// model different things -- the lag is how this drivetrain
		// habitually answers a command, the clamp is a ceiling it may
		// not cross -- and a lag that produced an impossible
		// acceleration would still be impossible.
		setpoint := clamp(targetSpeed, -k.maxSpeed, k.maxSpeed)
		lagged := setpoint
		if k.speedTauS > 0 {
			lagged = v + (setpoint-v)*math.Min(h/k.speedTauS, fullyClosedFraction)
		}
		v = approach(v, lagged, k.maxAccel*h)

		x += v * math.Cos(yaw) * h
		y += v * math.Sin(yaw) * h
		yaw += k.yawGain * (v / k.turnReferenceLen) * math.Tan(steer) * h
	}

	return AckermannState{X: x, Y: y, Yaw: navutil.WrapAngle(yaw), V: v, Steer: steer}
}

// approach moves current toward target by at most maxDelta, matching the
// private _approach slew-rate helper.
func approach(current, target, maxDelta float64) float64 {
	delta := target - current
	if delta > maxDelta {
		return current + maxDelta
	}
	if delta < -maxDelta {
		return current - maxDelta
	}
	return target
}

// clamp restricts value to [lo, hi]. Private to this package: navutil is
// off-limits to edit under the concurrent file-ownership constraint this
// port was done under, so a shared clamp helper isn't this package's to
// add there.
func clamp(value, lo, hi float64) float64 {
	return math.Max(lo, math.Min(hi, value))
}
