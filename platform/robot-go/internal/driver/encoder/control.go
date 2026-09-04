package encoder

import (
	"errors"
	"math"
)

// errCountsPerRevPositive mirrors control.py's _CPR_POSITIVE guard: every
// conversion divides by counts_per_rev, so a zero or negative value is a
// configuration error rather than something to clamp silently.
var errCountsPerRevPositive = errors.New("encoder: counts_per_rev must be positive")

// secondsPerMinute converts the estimator's revs/second into the RPM the
// rest of the drivetrain speaks (motors.toml, the feedforward calibration,
// and the Python oracle all work in rpm).
const secondsPerMinute = 60.0

// CountsToRevolutions returns output-shaft revolutions for a raw quadrature
// count. Errors when countsPerRev is not positive.
func CountsToRevolutions(counts int64, countsPerRev float64) (float64, error) {
	if countsPerRev <= 0 {
		return 0, errCountsPerRevPositive
	}
	return float64(counts) / countsPerRev, nil
}

// RevolutionsToDistance returns the linear wheel travel [m] for a number of
// output-shaft revolutions.
func RevolutionsToDistance(revolutions, wheelDiameterM float64) float64 {
	return revolutions * math.Pi * wheelDiameterM
}

// CountsToDistance returns the linear wheel travel [m] for a raw quadrature
// count. Errors when countsPerRev is not positive.
func CountsToDistance(counts int64, countsPerRev, wheelDiameterM float64) (float64, error) {
	revolutions, err := CountsToRevolutions(counts, countsPerRev)
	if err != nil {
		return 0, err
	}
	return RevolutionsToDistance(revolutions, wheelDiameterM), nil
}

// DefaultSmoothing is SpeedEstimator's exponential-smoothing factor,
// matching control.py's SpeedEstimator(smoothing=0.3) default.
const DefaultSmoothing = 0.3

// DefaultMinWindowS is the minimum interval SpeedEstimator accumulates
// counts over before computing a rate, matching control.py's
// min_window_s=0.1.
//
// A single ~20ms nav tick is not enough window on its own at low RPM and
// coarse counts_per_rev: bench data at 86 counts_per_rev / 13.6rpm target
// (2026-08-28) averaged ~0.39 counts per tick, so a per-tick rate is a raw
// 0-vs-1 count difference -- a >100% relative swing smoothing cannot
// remove, since it damps a noisy signal rather than fixing the signal's own
// resolution. At the same operating point a 0.1s window averages ~1.95
// counts, so a +-1 count difference is a ~50% swing instead. Reasoned, NOT
// live-verified -- re-check after any counts_per_rev/target-rpm change.
const DefaultMinWindowS = 0.1

// SpeedEstimator estimates signed output-shaft RPM from successive encoder
// counts. Ported from control.py's SpeedEstimator, including its
// accumulate-a-window-then-smooth shape; see DefaultMinWindowS for why the
// window exists at all.
//
// Not safe for concurrent use -- Quadrature owns one and serializes access
// under its own mutex.
type SpeedEstimator struct {
	countsPerRev float64
	smoothing    float64
	minWindowS   float64

	havePrev     bool
	prevCounts   int64
	windowCounts int64
	windowDTS    float64
	rpm          float64
}

// NewSpeedEstimator builds an estimator over countsPerRev, using
// DefaultSmoothing and DefaultMinWindowS. countsPerRev must be positive.
func NewSpeedEstimator(countsPerRev float64) (*SpeedEstimator, error) {
	return NewSpeedEstimatorWith(countsPerRev, DefaultSmoothing, DefaultMinWindowS)
}

// NewSpeedEstimatorWith is NewSpeedEstimator with explicit smoothing (in
// (0, 1]) and window (>= 0), for callers tuning against a different
// operating point.
func NewSpeedEstimatorWith(countsPerRev, smoothing, minWindowS float64) (*SpeedEstimator, error) {
	if countsPerRev <= 0 {
		return nil, errCountsPerRevPositive
	}
	if smoothing <= 0 || smoothing > 1 {
		return nil, errors.New("encoder: smoothing must be in (0, 1]")
	}
	if minWindowS < 0 {
		return nil, errors.New("encoder: min_window_s must be >= 0")
	}
	return &SpeedEstimator{
		countsPerRev: countsPerRev,
		smoothing:    smoothing,
		minWindowS:   minWindowS,
	}, nil
}

// Reset forgets history. Call when the encoder counter is zeroed or motion
// stops, so the next window is not differenced against a stale count.
func (e *SpeedEstimator) Reset() {
	e.havePrev = false
	e.prevCounts = 0
	e.windowCounts = 0
	e.windowDTS = 0
	e.rpm = 0
}

// Update folds in a new count reading taken dtS seconds after the previous
// one, returning the smoothed RPM once enough window has accumulated and
// the held value from the last completed window otherwise. The first call
// after construction or Reset only seeds the reference count.
func (e *SpeedEstimator) Update(counts int64, dtS float64) float64 {
	if dtS <= 0 {
		return e.rpm
	}
	if !e.havePrev {
		e.havePrev = true
		e.prevCounts = counts
		return e.rpm
	}

	e.windowCounts += counts - e.prevCounts
	e.prevCounts = counts
	e.windowDTS += dtS
	if e.windowDTS < e.minWindowS {
		return e.rpm
	}

	revolutions := float64(e.windowCounts) / e.countsPerRev
	rpmRaw := revolutions / e.windowDTS * secondsPerMinute
	e.rpm = e.smoothing*rpmRaw + (1.0-e.smoothing)*e.rpm
	e.windowCounts = 0
	e.windowDTS = 0
	return e.rpm
}

// RPM returns the most recent value Update computed, without resampling --
// Python's get_last_rpm.
func (e *SpeedEstimator) RPM() float64 {
	return e.rpm
}
