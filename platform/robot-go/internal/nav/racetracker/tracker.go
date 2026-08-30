package racetracker

import (
	"log/slog"
	"math"
	"time"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// RaceMetrics is the performance record of a run, matching race_tracker.py's
// RaceMetrics pydantic model.
//
// Python's extra_data free-form dict is omitted: nothing in the Go stack
// reads it, and carrying an untyped map for a consumer that does not exist
// would be worse than adding a typed field when one does.
type RaceMetrics struct {
	Elapsed           time.Duration
	TotalDistanceM    float64
	CurrentLap        int
	CompletedLaps     int
	WaypointIndex     int
	MaxSpeedMPS       float64
	AvgSpeedMPS       float64
	EscapeManeuvers   int
	StuckDetections   int
	CollisionWarnings int
	// LapSplits is elapsed time at each lap completion, newest last.
	LapSplits []time.Duration
}

// LoopProgress is a snapshot of lap, waypoint and distance progress,
// matching shared.domain.models.LoopProgress.
type LoopProgress struct {
	LapNumber     int
	WaypointIndex int
	// DistanceM is distance traveled since the current lap began.
	DistanceM float64
	// TotalDistanceM is distance traveled since the race began.
	TotalDistanceM float64
}

// RaceSummary is the end-of-race rollup, matching get_race_summary.
//
// A struct rather than Python's dict: every key it produces is known at
// compile time, and the two derived values below are the only things the
// dict added over RaceMetrics itself.
type RaceSummary struct {
	Metrics       RaceMetrics
	LapsRemaining int
	// EstFinishTime extrapolates from laps completed so far, and is zero
	// before the first lap lands (there is nothing to extrapolate from).
	EstFinishTime time.Duration
}

// Option configures a RaceTracker.
type Option func(*RaceTracker)

// RaceTracker tracks race metrics and state throughout a run, matching
// race_tracker.py's RaceTracker.
type RaceTracker struct {
	numLaps int
	metrics RaceMetrics
	now     func() time.Time

	startTime         time.Time
	lastPos           *trackmodel.Waypoint
	speedSampleSum    float64
	speedSampleCount  int
	lapStartDistanceM float64
}

// WithClock replaces the time source, so tests do not have to sleep to
// advance elapsed time the way the Python suite does.
func WithClock(now func() time.Time) Option {
	return func(t *RaceTracker) { t.now = now }
}

// NewRaceTracker builds a tracker for a race of numLaps laps.
//
// CurrentLap starts at 1, not 0: it is the lap being DRIVEN, while
// CompletedLaps is the count finished.
func NewRaceTracker(numLaps int, opts ...Option) *RaceTracker {
	tracker := &RaceTracker{
		numLaps: numLaps,
		metrics: RaceMetrics{CurrentLap: 1},
		now:     time.Now,
	}
	for _, opt := range opts {
		opt(tracker)
	}
	tracker.startTime = tracker.now()
	return tracker
}

// UpdatePosition folds one tick of robot state into the metrics.
//
// Average speed is kept as a running sum rather than a retained sample slice.
// Python appends every sample and re-sums the whole list on each call, which
// is the same mean but grows without bound across a multi-minute race; only
// the mean is ever read back.
func (t *RaceTracker) UpdatePosition(currentPos trackmodel.Waypoint, currentSpeedMPS float64, waypointIndex int) {
	t.metrics.Elapsed = t.now().Sub(t.startTime)
	t.metrics.WaypointIndex = waypointIndex

	if t.lastPos != nil {
		t.metrics.TotalDistanceM += math.Hypot(currentPos.X-t.lastPos.X, currentPos.Y-t.lastPos.Y)
	}
	t.lastPos = &currentPos

	t.metrics.MaxSpeedMPS = math.Max(t.metrics.MaxSpeedMPS, currentSpeedMPS)
	t.speedSampleSum += currentSpeedMPS
	t.speedSampleCount++
	t.metrics.AvgSpeedMPS = t.speedSampleSum / float64(t.speedSampleCount)
}

// IncrementLap records completion of the current lap.
func (t *RaceTracker) IncrementLap() {
	t.metrics.CompletedLaps++
	t.metrics.CurrentLap = t.metrics.CompletedLaps + 1
	t.lapStartDistanceM = t.metrics.TotalDistanceM
	t.metrics.LapSplits = append(t.metrics.LapSplits, t.now().Sub(t.startTime))
}

// RecordEscapeManeuver records an escape maneuver (K-turn and similar).
func (t *RaceTracker) RecordEscapeManeuver() { t.metrics.EscapeManeuvers++ }

// RecordStuckDetection records a stuck-robot detection.
func (t *RaceTracker) RecordStuckDetection() { t.metrics.StuckDetections++ }

// RecordCollisionWarning records a high-risk collision scenario.
func (t *RaceTracker) RecordCollisionWarning() { t.metrics.CollisionWarnings++ }

// IsRaceComplete reports whether every lap has been completed.
func (t *RaceTracker) IsRaceComplete() bool { return t.metrics.CompletedLaps >= t.numLaps }

// Metrics returns the current metrics. LapSplits is cloned so a caller
// cannot mutate the tracker's own slice through the returned value.
func (t *RaceTracker) Metrics() RaceMetrics {
	metrics := t.metrics
	metrics.LapSplits = append([]time.Duration(nil), t.metrics.LapSplits...)
	return metrics
}

// Progress returns a lap/waypoint/distance snapshot.
//
// DistanceM reads zero until the first UpdatePosition, matching Python's
// conditional: before any position is known there is no lap-relative
// distance to report, and reporting the total instead would overstate
// progress within the current lap.
func (t *RaceTracker) Progress() LoopProgress {
	distanceM := 0.0
	if t.lastPos != nil {
		distanceM = t.metrics.TotalDistanceM - t.lapStartDistanceM
	}
	return LoopProgress{
		LapNumber:      t.metrics.CurrentLap,
		WaypointIndex:  t.metrics.WaypointIndex,
		DistanceM:      distanceM,
		TotalDistanceM: t.metrics.TotalDistanceM,
	}
}

// Summary returns the end-of-race rollup.
func (t *RaceTracker) Summary() RaceSummary {
	summary := RaceSummary{
		Metrics:       t.Metrics(),
		LapsRemaining: max(0, t.numLaps-t.metrics.CompletedLaps),
	}
	if t.metrics.CompletedLaps > 0 {
		summary.EstFinishTime = time.Duration(
			float64(t.metrics.Elapsed) * float64(t.numLaps) / float64(t.metrics.CompletedLaps),
		)
	}
	return summary
}

// LogSummary logs the race summary, matching log_summary.
func (t *RaceTracker) LogSummary(logger *slog.Logger) {
	summary := t.Summary()
	logger.Info("race summary",
		"completed_laps", summary.Metrics.CompletedLaps,
		"laps_remaining", summary.LapsRemaining,
		"elapsed", summary.Metrics.Elapsed,
		"total_distance_m", summary.Metrics.TotalDistanceM,
		"max_speed_mps", summary.Metrics.MaxSpeedMPS,
		"avg_speed_mps", summary.Metrics.AvgSpeedMPS,
		"lap_splits", summary.Metrics.LapSplits,
		"est_finish_time", summary.EstFinishTime,
	)
}
