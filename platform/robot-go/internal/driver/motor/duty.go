package motor

const (
	dutyMin = -1.0
	dutyMax = 1.0
	dutyOff = 0.0
)

// clampDuty clamps a signed duty fraction to [-1, 1].
func clampDuty(duty float64) float64 {
	return min(max(duty, dutyMin), dutyMax)
}

// splitDuty resolves a signed, clamped duty fraction into independent RPWM
// (forward) and LPWM (reverse) duty fractions, each in [0, 1].
//
// Exactly one of the two return values is ever nonzero. This is the
// Fast-Brake invariant: per the BTS7960's own truth table, RPWM=LPWM=HIGH is
// "Fast Brake" (motor terminals shorted) and RPWM=LPWM=LOW is "Coast" (see
// platform/robot/docs/bts7960-ibt2-wiring.md) — neither is drive, so the two
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
