package corridorestimator

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// votes is one corridor's tally, matching Python's [narrow_votes, wide_votes].
type votes struct {
	narrow int
	wide   int
}

// WidthEstimator is a running per-section estimate of the track layout, from
// LIDAR only, matching CorridorWidthEstimator.
//
// Starts from an assumed width and re-classifies a corridor only after
// repeated agreeing observations.
type WidthEstimator struct {
	cfg   Config
	fixed bool

	// Parallel arrays indexed by Section rather than maps, matching this
	// codebase's preference and making iteration order fixed.
	widths   [len(sections)]float64
	observed [len(sections)]bool
	tally    [len(sections)]votes
}

// Option configures a WidthEstimator.
type Option func(*WidthEstimator)

// floatTolerance mirrors math.isclose's role in the width comparison, which
// only ever compares two of the same two constants.
const floatTolerance = 1e-9

// sections is every corridor, in a fixed order so iteration is deterministic.
var sections = [...]trackmodel.Section{
	trackmodel.North, trackmodel.South, trackmodel.East, trackmodel.West,
}

func (v votes) total() int { return v.narrow + v.wide }

// WithFixedWidth pins every corridor to the assumed width, for a challenge
// whose width is a KNOWN CONSTANT rather than a prior.
//
// The Obstacles Challenge is exactly that case: every corridor is 1.0 m by
// rule, not by discovery. There, a sign or pillar hugging one wall can feed
// the vote a run of falsely-narrow readings -- a LIDAR ray clipping the
// obstacle instead of the real wall -- with nothing to correct it back. And
// unlike the Open Challenge, where believing narrow is deliberately the safe
// direction to be wrong in, an Obstacles corridor wrongly believed narrow
// re-plans with LESS room than actually exists, right where an obstacle
// already eats into the true 1.0 m.
//
// Every other behavior is unchanged: this only stops ObserveMeasurement from
// moving a width away from the assumed one.
func WithFixedWidth() Option {
	return func(e *WidthEstimator) { e.fixed = true }
}

// New builds an estimator whose corridors all start at assumedWidthM.
//
// The safe prior for the Open Challenge is the NARROW width -- see the
// package doc for why, and for why that is the wrong prior in Obstacles.
func New(assumedWidthM float64, cfg Config, opts ...Option) *WidthEstimator {
	estimator := &WidthEstimator{cfg: cfg}
	for i := range estimator.widths {
		estimator.widths[i] = assumedWidthM
	}
	for _, opt := range opts {
		opt(estimator)
	}
	return estimator
}

// Widths returns the current best estimate for every section.
func (e *WidthEstimator) Widths() map[trackmodel.Section]float64 {
	out := make(map[trackmodel.Section]float64, len(sections))
	for i, section := range sections {
		out[section] = e.widths[i]
	}
	return out
}

// WidthFor returns the current best estimate for one section.
func (e *WidthEstimator) WidthFor(section trackmodel.Section) float64 {
	return e.widths[indexOf(section)]
}

// IsObserved reports whether a section has been confirmed at least once
// rather than assumed.
func (e *WidthEstimator) IsObserved(section trackmodel.Section) bool {
	return e.observed[indexOf(section)]
}

// IsComplete reports whether every corridor has been measured rather than
// assumed.
func (e *WidthEstimator) IsComplete() bool {
	for _, observed := range e.observed {
		if !observed {
			return false
		}
	}
	return true
}

// Observe folds one scan into the estimate for section, reporting whether the
// estimate changed so the caller knows to replan against the new layout.
func (e *WidthEstimator) Observe(
	section trackmodel.Section,
	rangesM []float64,
	anglesRad []float64,
	yaw float64,
) bool {
	measurement, ok := MeasureCorridorWidth(rangesM, anglesRad, yaw, e.cfg)
	if !ok {
		return false
	}
	return e.ObserveMeasurement(section, measurement.WidthM)
}

// ObserveMeasurement folds in a width already produced by
// MeasureCorridorWidth.
//
// Separate from Observe so a reading can be taken BEFORE it can be
// attributed. A blind round infers its travel direction after it has started
// driving, and attribution needs that direction, so scans from before it
// settles would otherwise be discarded -- throwing away the cleanest readings
// of the starting corridor and leaving the first surviving ones to be taken
// at a corner, where the side rays span the NEXT corridor. Buffer them, replay
// them here once the direction is known.
func (e *WidthEstimator) ObserveMeasurement(section trackmodel.Section, measuredM float64) bool {
	if e.fixed {
		return false
	}

	index := indexOf(section)
	if measuredM >= e.cfg.DecisionBoundaryM {
		e.tally[index].wide++
	} else {
		e.tally[index].narrow++
	}
	if e.tally[index].total() < e.cfg.MinSamples {
		return false
	}

	// Ties fall to NARROW, matching Python's `wide if wide > narrow else
	// narrow` -- the safe direction to be wrong in for the Open Challenge.
	verdict := e.cfg.NarrowWidthM
	if e.tally[index].wide > e.tally[index].narrow {
		verdict = e.cfg.WideWidthM
	}

	e.observed[index] = true
	if math.Abs(e.widths[index]-verdict) < floatTolerance {
		return false
	}
	e.widths[index] = verdict
	return true
}

func indexOf(section trackmodel.Section) int {
	for i, candidate := range sections {
		if candidate == section {
			return i
		}
	}
	// Section is a closed enum; an out-of-range value is a programming error
	// rather than a runtime condition, and North is the safe fallback since
	// every corridor starts at the same assumed width.
	return 0
}
