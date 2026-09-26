package sensormodel

import "math/rand/v2"

// TickPeriod samples control periods from the measured distribution.
type TickPeriod struct {
	quantiles []float64
	levels    []float64
	nominalS  float64
	rng       *rand.Rand
}

// tickJitterStreamSalt keeps the period draws off every other stream.
const tickJitterStreamSalt = 0x5f17_3c6e_f372_fe94

// SampleQuantile maps a uniform draw u in [0, 1) through the piecewise-linear
// inverse CDF given by quantiles at levels, as the Python oracle's
// _sample_confidence and _sample_period do. A pair of different lengths, or
// fewer than two levels, returns fallback; a draw above the last level
// returns the last quantile (a flat tail).
func SampleQuantile(u float64, quantiles, levels []float64, fallback float64) float64 {
	if len(quantiles) != len(levels) || len(levels) < 2 {
		return fallback
	}
	for i := 1; i < len(levels); i++ {
		if u > levels[i] {
			continue
		}
		span := levels[i] - levels[i-1]
		frac := 0.0
		if span > 0 {
			frac = (u - levels[i-1]) / span
		}
		return quantiles[i-1] + frac*(quantiles[i]-quantiles[i-1])
	}
	return quantiles[len(quantiles)-1]
}

// NewTickPeriod builds a sampler over p's measured periods; an unusable
// distribution yields nominalS every tick.
func NewTickPeriod(p Params, nominalS float64, seed uint64) *TickPeriod {
	return &TickPeriod{
		quantiles: p.TickPeriodQuantiles,
		levels:    p.TickPeriodLevels,
		nominalS:  nominalS,
		rng:       rand.New(rand.NewPCG(seed^tickJitterStreamSalt, seed)),
	}
}

// Next returns the next tick's period, seconds.
func (t *TickPeriod) Next() float64 {
	return SampleQuantile(t.rng.Float64(), t.quantiles, t.levels, t.nominalS)
}
