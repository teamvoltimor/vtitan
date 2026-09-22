package hbridge

import "math"

const (
	dutyMin = -1.0
	dutyMax = 1.0
	dutyOff = 0.0
)

// clampDuty clamps a signed duty fraction to [-1, 1], and maps NaN to off.
//
// Go's min and max return NaN if either argument is NaN, so without the
// guard NaN would pass straight through to a PWM channel, where the
// float-to-int conversion of its pulse width is architecture-defined.
// Infinities already clamp to the rails, which is correct arithmetic but a
// full-throttle command: rejecting those is the caller's job.
func clampDuty(duty float64) float64 {
	if math.IsNaN(duty) {
		return dutyOff
	}
	return min(max(duty, dutyMin), dutyMax)
}

// splitDuty resolves a signed, clamped duty fraction into independent RPWM
// (forward) and LPWM (reverse) duty fractions, each in [0, 1].
//
// Exactly one of the two return values is ever nonzero. This is the
// Fast-Brake invariant: per the BTS7960's own truth table, RPWM=LPWM=HIGH is
// "Fast Brake" (motor terminals shorted) and RPWM=LPWM=LOW is "Coast" (see
// src/python/docs/bts7960-ibt2-wiring.md) — neither is drive, so the two
// channels must never carry a nonzero duty at the same time. splitDuty is
// the single place that invariant is enforced structurally: it is
// impossible to call it and get two nonzero results back.
func splitDuty(signedDuty float64) (rpwm, lpwm float64) {
	clamped := clampDuty(signedDuty)
	if clamped >= dutyOff {
		return clamped, dutyOff
	}
	return dutyOff, -clamped
}
