package quadrature

import (
	"errors"
	"math"
)

// SpeedEstimator estimates signed output-shaft RPM from successive encoder
// counts. Ported from control.py's SpeedEstimator, including its
// accumulate-a-window-then-smooth shape; see DefaultMinWindowS for why the
// window exists at all.
//
// Not safe for concurrent use -- encoder.Quadrature owns one and serializes
// access under its own mutex.
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

// SpeedEstimatorParams configures NewSpeedEstimatorWith, mirroring the
// kinematics.Params pattern: three interchangeable float64s named at the
// call site instead of relying on positional order.
type SpeedEstimatorParams struct {
	CountsPerRev float64
	// Smoothing is the exponential-smoothing factor, in (0, 1].
	Smoothing float64
	// MinWindowS is the minimum interval counts accumulate over, >= 0.
	MinWindowS float64
}

// DefaultSmoothing is SpeedEstimator's exponential-smoothing factor,
// matching control.py's SpeedEstimator(smoothing=0.3) default.
const DefaultSmoothing = 0.3

// DefaultMinWindowS is the minimum interval SpeedEstimator accumulates
// counts over before computing a rate, matching control.py's
// min_window_s=0.1.
//
// A single nav tick is not enough window on its own at low RPM and coarse
// counts_per_rev: a per-tick rate is a raw 0-vs-1 count difference, a large
// relative swing smoothing cannot remove, since it damps a noisy signal
// rather than fixing the signal's own resolution. A longer window averages
// several counts, so a +-1 count difference is a much smaller swing.
// Reasoned, NOT live-verified -- re-check after any counts_per_rev/target-rpm
// change. See adr:0076-drivetrain-and-steering-hardware.
const DefaultMinWindowS = 0.1

// ErrCountsPerRevPositive mirrors control.py's _CPR_POSITIVE guard: every
// conversion divides by counts_per_rev, so a zero or negative value is a
// configuration error rather than something to clamp silently.
var ErrCountsPerRevPositive = errors.New("quadrature: counts_per_rev must be positive")

// CountsToRevolutions returns output-shaft revolutions for a raw quadrature
// count. Errors when countsPerRev is not positive.
func CountsToRevolutions(counts int64, countsPerRev float64) (float64, error) {
	if countsPerRev <= 0 {
		return 0, ErrCountsPerRevPositive
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

// NewSpeedEstimator builds an estimator over countsPerRev, using
// DefaultSmoothing and DefaultMinWindowS. countsPerRev must be positive.
func NewSpeedEstimator(countsPerRev float64) (*SpeedEstimator, error) {
	return NewSpeedEstimatorWith(SpeedEstimatorParams{
		CountsPerRev: countsPerRev,
		Smoothing:    DefaultSmoothing,
		MinWindowS:   DefaultMinWindowS,
	})
}

// NewSpeedEstimatorWith is NewSpeedEstimator with explicit smoothing and
// window, for callers tuning against a different operating point.
func NewSpeedEstimatorWith(p SpeedEstimatorParams) (*SpeedEstimator, error) {
	if p.CountsPerRev <= 0 {
		return nil, ErrCountsPerRevPositive
	}
	if p.Smoothing <= 0 || p.Smoothing > 1 {
		return nil, errors.New("quadrature: smoothing must be in (0, 1]")
	}
	if p.MinWindowS < 0 {
		return nil, errors.New("quadrature: min_window_s must be >= 0")
	}
	return &SpeedEstimator{
		countsPerRev: p.CountsPerRev,
		smoothing:    p.Smoothing,
		minWindowS:   p.MinWindowS,
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
	rpmRaw := revolutions / e.windowDTS * SecondsPerMinute
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
