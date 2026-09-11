package racetracker_test

import (
	"math"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/racetracker"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// fakeClock advances only when a test says so, replacing the real sleeps
// tests/unit/test_race_tracker.py uses to make elapsed time move.
type fakeClock struct{ now time.Time }

func (c *fakeClock) Now() time.Time {
	return c.now
}
func (c *fakeClock) advance(d time.Duration) {
	c.now = c.now.Add(d)
}

func newTracker(numLaps int) (*racetracker.RaceTracker, *fakeClock) {
	clock := &fakeClock{now: time.Unix(0, 0)}
	return racetracker.NewRaceTracker(numLaps, racetracker.WithClock(clock.Now)), clock
}

// TestRaceTracker_Initialization pins CurrentLap starting at 1 rather than 0:
// it is the lap being DRIVEN, while CompletedLaps counts finished ones.
func TestRaceTracker_Initialization(t *testing.T) {
	t.Parallel()

	tracker, _ := newTracker(3)
	metrics := tracker.Metrics()

	if metrics.CurrentLap != 1 {
		t.Fatalf("CurrentLap = %d, want 1", metrics.CurrentLap)
	}
	if metrics.CompletedLaps != 0 || metrics.TotalDistanceM != 0 {
		t.Fatalf("metrics = %+v, want a zeroed race", metrics)
	}
	if tracker.IsRaceComplete() {
		t.Fatal("IsRaceComplete() = true at construction")
	}
}

func TestRaceTracker_IncrementLap(t *testing.T) {
	t.Parallel()

	tracker, clock := newTracker(3)

	clock.advance(10 * time.Second)
	tracker.IncrementLap()
	clock.advance(12 * time.Second)
	tracker.IncrementLap()

	metrics := tracker.Metrics()
	if metrics.CompletedLaps != 2 {
		t.Fatalf("CompletedLaps = %d, want 2", metrics.CompletedLaps)
	}
	if metrics.CurrentLap != 3 {
		t.Fatalf("CurrentLap = %d, want 3", metrics.CurrentLap)
	}
	// Splits are elapsed-at-completion, not per-lap durations.
	want := []time.Duration{10 * time.Second, 22 * time.Second}
	if len(metrics.LapSplits) != len(want) {
		t.Fatalf("LapSplits = %v, want %v", metrics.LapSplits, want)
	}
	for i := range want {
		if metrics.LapSplits[i] != want[i] {
			t.Fatalf("LapSplits = %v, want %v", metrics.LapSplits, want)
		}
	}
}

func TestRaceTracker_IsRaceComplete(t *testing.T) {
	t.Parallel()

	tracker, _ := newTracker(2)
	tracker.IncrementLap()
	if tracker.IsRaceComplete() {
		t.Fatal("IsRaceComplete() = true after 1 of 2 laps")
	}
	tracker.IncrementLap()
	if !tracker.IsRaceComplete() {
		t.Fatal("IsRaceComplete() = false after 2 of 2 laps")
	}
}

// TestRaceTracker_UpdatePosition covers distance accumulating between
// samples and the first sample contributing none -- there is no previous
// position to measure from.
func TestRaceTracker_UpdatePosition(t *testing.T) {
	t.Parallel()

	tracker, clock := newTracker(3)

	tracker.UpdatePosition(trackmodel.Waypoint{X: 0, Y: 0}, 0.2, 0)
	if got := tracker.Metrics().TotalDistanceM; got != 0 {
		t.Fatalf("TotalDistanceM = %v after one sample, want 0", got)
	}

	clock.advance(time.Second)
	tracker.UpdatePosition(trackmodel.Waypoint{X: 3, Y: 4}, 0.4, 5)

	metrics := tracker.Metrics()
	if math.Abs(metrics.TotalDistanceM-5.0) > 1e-9 {
		t.Fatalf("TotalDistanceM = %v, want 5", metrics.TotalDistanceM)
	}
	if metrics.WaypointIndex != 5 {
		t.Fatalf("WaypointIndex = %d, want 5", metrics.WaypointIndex)
	}
	if metrics.MaxSpeedMPS != 0.4 {
		t.Fatalf("MaxSpeedMPS = %v, want 0.4", metrics.MaxSpeedMPS)
	}
	if math.Abs(metrics.AvgSpeedMPS-0.3) > 1e-9 {
		t.Fatalf("AvgSpeedMPS = %v, want 0.3", metrics.AvgSpeedMPS)
	}
	if metrics.Elapsed != time.Second {
		t.Fatalf("Elapsed = %v, want 1s", metrics.Elapsed)
	}
}

// TestRaceTracker_MaxSpeedIsAPeak guards the running-mean rewrite: max must
// keep the highest sample even after slower ones arrive.
func TestRaceTracker_MaxSpeedIsAPeak(t *testing.T) {
	t.Parallel()

	tracker, _ := newTracker(3)
	for _, speed := range []float64{0.1, 0.9, 0.2} {
		tracker.UpdatePosition(trackmodel.Waypoint{}, speed, 0)
	}

	metrics := tracker.Metrics()
	if metrics.MaxSpeedMPS != 0.9 {
		t.Fatalf("MaxSpeedMPS = %v, want 0.9", metrics.MaxSpeedMPS)
	}
	if math.Abs(metrics.AvgSpeedMPS-0.4) > 1e-9 {
		t.Fatalf("AvgSpeedMPS = %v, want 0.4", metrics.AvgSpeedMPS)
	}
}

func TestRaceTracker_Incidents(t *testing.T) {
	t.Parallel()

	tracker, _ := newTracker(3)
	tracker.RecordEscapeManeuver()
	tracker.RecordEscapeManeuver()
	tracker.RecordStuckDetection()
	tracker.RecordCollisionWarning()

	metrics := tracker.Metrics()
	if metrics.EscapeManeuvers != 2 || metrics.StuckDetections != 1 ||
		metrics.CollisionWarnings != 1 {
		t.Fatalf("incident counts = %+v, want 2/1/1", metrics)
	}
}

// TestRaceTracker_Progress covers the lap-relative distance reset and the
// pre-first-sample case, where Python reports zero rather than the total.
func TestRaceTracker_Progress(t *testing.T) {
	t.Parallel()

	tracker, _ := newTracker(3)

	if got := tracker.Progress().DistanceM; got != 0 {
		t.Fatalf("DistanceM = %v before any position, want 0", got)
	}

	tracker.UpdatePosition(trackmodel.Waypoint{X: 0, Y: 0}, 0.2, 0)
	tracker.UpdatePosition(trackmodel.Waypoint{X: 10, Y: 0}, 0.2, 1)
	tracker.IncrementLap()
	tracker.UpdatePosition(trackmodel.Waypoint{X: 13, Y: 0}, 0.2, 2)

	progress := tracker.Progress()
	// Distance since the lap began, not since the race began.
	if math.Abs(progress.DistanceM-3.0) > 1e-9 {
		t.Fatalf("DistanceM = %v, want 3 since the lap started", progress.DistanceM)
	}
	if math.Abs(progress.TotalDistanceM-13.0) > 1e-9 {
		t.Fatalf("TotalDistanceM = %v, want 13", progress.TotalDistanceM)
	}
	if progress.LapNumber != 2 {
		t.Fatalf("LapNumber = %d, want 2", progress.LapNumber)
	}
}

func TestRaceTracker_Summary(t *testing.T) {
	t.Parallel()

	tracker, clock := newTracker(3)

	// Before any lap there is nothing to extrapolate from.
	if got := tracker.Summary().EstFinishTime; got != 0 {
		t.Fatalf("EstFinishTime = %v before the first lap, want 0", got)
	}

	clock.advance(30 * time.Second)
	tracker.UpdatePosition(trackmodel.Waypoint{}, 0.2, 0)
	tracker.IncrementLap()

	summary := tracker.Summary()
	if summary.LapsRemaining != 2 {
		t.Fatalf("LapsRemaining = %d, want 2", summary.LapsRemaining)
	}
	if summary.EstFinishTime != 90*time.Second {
		t.Fatalf("EstFinishTime = %v, want 90s (30s x 3 laps / 1 done)", summary.EstFinishTime)
	}
}

// TestRaceTracker_MetricsDoesNotAliasSplits guards the defensive copy: a
// caller appending to the returned slice must not corrupt the tracker.
func TestRaceTracker_MetricsDoesNotAliasSplits(t *testing.T) {
	t.Parallel()

	tracker, _ := newTracker(3)
	tracker.IncrementLap()

	metrics := tracker.Metrics()
	metrics.LapSplits = append(metrics.LapSplits, 999*time.Second)

	if got := len(tracker.Metrics().LapSplits); got != 1 {
		t.Fatalf("tracker LapSplits length = %d, want 1 -- the caller mutated internal state", got)
	}
}

// TestRaceTracker_LapsRemainingFloorsAtZero covers overrunning the lap
// target, which max(0, ...) exists to keep from reporting a negative.
func TestRaceTracker_LapsRemainingFloorsAtZero(t *testing.T) {
	t.Parallel()

	tracker, _ := newTracker(1)
	tracker.IncrementLap()
	tracker.IncrementLap()

	if got := tracker.Summary().LapsRemaining; got != 0 {
		t.Fatalf("LapsRemaining = %d, want 0", got)
	}
}
