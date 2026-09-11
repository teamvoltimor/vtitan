//go:build linux

package encoder

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"time"

	"github.com/warthog618/go-gpiocdev"
)

// gpioLineHigh is the raw line-value integer go-gpiocdev's Values returns
// for a HIGH reading.
const gpioLineHigh = 1

// errSampleBeforeConnect is returned by the sampling methods when called
// before Connect has completed successfully.
var errSampleBeforeConnect = errors.New("encoder: sampled before Connect")

// nominalDTS is the step assumed on the first RPM sample, before a real
// interval can be measured -- control.py's _NOMINAL_DT_S.
const nominalDTS = 0.02

// minDTS/maxDTS bound a measured step, so two calls in the same instant or
// a long stall cannot turn into a divide-by-zero or a huge derivative kick
// -- control.py's _MIN_DT_S/_MAX_DT_S.
const (
	minDTS = 0.001
	maxDTS = 0.5
)

// Quadrature is the GPIO-backed A/B quadrature encoder on the drive shaft.
// Counting is interrupt-driven: both channels are requested with both-edge
// detection and the kernel's events feed the pure Decoder, so no count is
// lost between polls the way a sampled reader would lose them at speed.
//
// Python gets this from gpiozero's RotaryEncoder, whose own counting is
// likewise edge-driven; what is NOT inherited is gpiozero's counts-per-cycle
// convention -- see Decoder.CountsPerEdge before trusting DistanceM on
// hardware.
//
// Safe for concurrent use: the event handler runs on go-gpiocdev's own
// goroutine, so every access to the decoder and estimator is under mu.
type Quadrature struct {
	cfg   Config
	lines *gpiocdev.Lines

	mu        sync.Mutex
	decoder   Decoder
	estimator *SpeedEstimator
	// levels caches each channel's last known level, indexed by the same
	// order as Config.PinA/PinB. An edge event reports only the line that
	// changed, and quadrature decoding needs both.
	levels    [2]bool
	lastRPMAt time.Time
	haveRPMAt bool
}

// New validates cfg and returns a Quadrature. Call Connect before sampling.
func New(cfg Config) (*Quadrature, error) {
	if err := cfg.Validate(); err != nil {
		return nil, err
	}
	estimator, err := NewSpeedEstimator(cfg.CountsPerRev)
	if err != nil {
		return nil, err
	}
	return &Quadrature{cfg: cfg, estimator: estimator}, nil
}

// Connect requests the A and B channels as both-edge inputs and seeds the
// decoder with their current levels.
//
// The initial read matters: without it the first edge would be decoded
// against a zeroed reference state and could fabricate a count in the wrong
// direction. Decoder.Sample tolerates that by design (its first call only
// establishes state), and seeding here means that throwaway sample is spent
// at startup rather than on the first real motion.
func (q *Quadrature) Connect(_ context.Context) error {
	lines, err := gpiocdev.RequestLines(
		q.cfg.GPIOChip,
		[]int{q.cfg.PinA, q.cfg.PinB},
		gpiocdev.AsInput,
		gpiocdev.WithBothEdges,
		gpiocdev.WithEventHandler(q.handleEdge),
	)
	if err != nil {
		return fmt.Errorf(
			"encoder: requesting GPIO lines %s:%d,%d: %w",
			q.cfg.GPIOChip, q.cfg.PinA, q.cfg.PinB, err,
		)
	}

	values := make([]int, 2)
	if valuesErr := lines.Values(values); valuesErr != nil {
		if closeErr := lines.Close(); closeErr != nil {
			return errors.Join(
				fmt.Errorf("encoder: reading initial line values: %w", valuesErr),
				fmt.Errorf("encoder: closing GPIO lines: %w", closeErr),
			)
		}
		return fmt.Errorf("encoder: reading initial line values: %w", valuesErr)
	}

	q.mu.Lock()
	q.lines = lines
	q.levels[0] = values[0] == gpioLineHigh
	q.levels[1] = values[1] == gpioLineHigh
	q.decoder.Sample(q.levels[0], q.levels[1])
	q.mu.Unlock()
	return nil
}

// handleEdge folds one kernel edge event into the decoder. It runs on
// go-gpiocdev's event goroutine, not the caller's.
func (q *Quadrature) handleEdge(event gpiocdev.LineEvent) {
	index := 0
	if event.Offset == q.cfg.PinB {
		index = 1
	} else if event.Offset != q.cfg.PinA {
		return
	}

	q.mu.Lock()
	q.levels[index] = event.Type == gpiocdev.LineEventRisingEdge
	q.decoder.Sample(q.levels[0], q.levels[1])
	q.mu.Unlock()
}

// Counts returns the accumulated quadrature count in the COMMAND frame
// (Config.Invert applied), or 0 before Connect. Sign-corrected here rather
// than at each call site so every derived quantity inherits it consistently
// and cannot disagree with the others.
func (q *Quadrature) Counts() int64 {
	q.mu.Lock()
	defer q.mu.Unlock()
	return q.countsLocked()
}

func (q *Quadrature) countsLocked() int64 {
	if q.lines == nil {
		return 0
	}
	if q.cfg.Invert {
		return -q.decoder.Counts()
	}
	return q.decoder.Counts()
}

// RPM samples the counter and returns the smoothed output-shaft RPM,
// measuring its own interval since the previous RPM call (bounded by
// minDTS/maxDTS). Returns the held value until a full estimator window has
// accumulated -- see DefaultMinWindowS.
func (q *Quadrature) RPM() (float64, error) {
	q.mu.Lock()
	defer q.mu.Unlock()
	if q.lines == nil {
		return 0, errSampleBeforeConnect
	}
	return q.estimator.Update(q.countsLocked(), q.elapsedLocked()), nil
}

// elapsedLocked returns seconds since the previous RPM sample, seeding the
// first call with nominalDTS.
func (q *Quadrature) elapsedLocked() float64 {
	now := time.Now()
	if !q.haveRPMAt {
		q.haveRPMAt = true
		q.lastRPMAt = now
		return nominalDTS
	}
	dtS := now.Sub(q.lastRPMAt).Seconds()
	q.lastRPMAt = now
	return min(max(dtS, minDTS), maxDTS)
}

// Odometry returns a full sample (counts, revolutions, RPM, distance) from
// the CACHED rpm, matching Python's get_odometry: the caller that wants a
// fresh speed calls RPM first, exactly as the Python node's feedback timer
// does. Recomputing it here would give the estimator a second, shorter
// window on every call and quietly halve its noise rejection.
func (q *Quadrature) Odometry() (Odometry, error) {
	q.mu.Lock()
	defer q.mu.Unlock()
	if q.lines == nil {
		return Odometry{}, errSampleBeforeConnect
	}

	counts := q.countsLocked()
	revolutions, err := CountsToRevolutions(counts, q.cfg.CountsPerRev)
	if err != nil {
		return Odometry{}, err
	}
	return Odometry{
		Counts:      counts,
		Revolutions: revolutions,
		RPM:         q.estimator.RPM(),
		DistanceM:   RevolutionsToDistance(revolutions, q.cfg.WheelDiameterM),
	}, nil
}

// Reset zeroes the counter and the speed estimator, e.g. at the start of a
// new race.
func (q *Quadrature) Reset() {
	q.mu.Lock()
	defer q.mu.Unlock()
	q.decoder.Reset()
	q.estimator.Reset()
	q.haveRPMAt = false
	// Re-seed the decoder from the levels the edge handler last saw, so the
	// next real edge decodes against a known state instead of spending a
	// throwaway sample mid-motion.
	q.decoder.Sample(q.levels[0], q.levels[1])
}

// Close releases the GPIO lines and makes the Quadrature unusable.
func (q *Quadrature) Close() error {
	q.mu.Lock()
	defer q.mu.Unlock()
	if q.lines == nil {
		return nil
	}
	err := q.lines.Close()
	q.lines = nil
	if err != nil {
		return fmt.Errorf("encoder: closing GPIO lines: %w", err)
	}
	return nil
}
