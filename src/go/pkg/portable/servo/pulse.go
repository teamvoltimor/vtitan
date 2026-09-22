package servo

import "math"

// Pulse is the servo's pulse calibration: the part of the Linux driver's
// Config (pkg/driver/servo) that decides the pulse width, without the
// wiring. Validation stays with whoever loads it; PulseUS assumes
// MinPulseUS < MaxPulseUS, CenterPulseUS within them and RangeDeg > 0.
type Pulse struct {
	MinPulseUS    float64
	MaxPulseUS    float64
	CenterPulseUS float64
	// RangeDeg is the servo's full mechanical travel (180 or 270): the angle
	// that spans MinPulseUS..MaxPulseUS.
	RangeDeg float64
	// Reversed flips the sign of every commanded angle.
	Reversed bool
}

// EpsilonUS is the smallest pulse change worth writing
// (servo/driver.py:63 _PULSE_EPSILON_US). A controller holding a
// heading resends the same angle every tick; 1 us is well under a hobby
// servo's own 2-10 us deadband, so this cannot swallow a real move.
const EpsilonUS = 1.0

// PulseUS maps a servo angle (deg, 0 = center) to a pulse width in
// microseconds: center + (angle / range) * (max - min), sign flipped when
// Reversed, clamped to [min, max]. servo/driver.py:149-155
// _position_to_pulse_us. The caller keeps angleDeg finite.
func PulseUS(p Pulse, angleDeg float64) float64 {
	signed := angleDeg
	if p.Reversed {
		signed = -angleDeg
	}
	spanUS := p.MaxPulseUS - p.MinPulseUS
	pulseUS := p.CenterPulseUS + (signed/p.RangeDeg)*spanUS
	return max(p.MinPulseUS, min(p.MaxPulseUS, pulseUS))
}

// NeedsWrite reports whether nextUS must be written given the last pulse
// written, lastUS. hasLast is false before the first write after a
// (re)connect, so that write is never skipped as a no-op
// (servo/driver.py:89 _pulse_us = None, reset on disconnect at :147);
// otherwise a change under EpsilonUS is not written.
func NeedsWrite(lastUS float64, hasLast bool, nextUS float64) bool {
	return !hasLast || math.Abs(nextUS-lastUS) >= EpsilonUS
}
