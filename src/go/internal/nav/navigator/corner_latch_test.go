package navigator

import (
	"math"
	"testing"
)

// latchThreshold matches pursuit.CORNER_TURN_THRESHOLD_RAD; the caller
// supplies it.
const latchThreshold = 0.35

const latchTolerance = 1e-9

func assertClose(t *testing.T, got, want float64, msg string) {
	t.Helper()
	if math.Abs(got-want) > latchTolerance {
		t.Errorf("%s: got %v, want %v", msg, got, want)
	}
}

// Ports tests/unit/test_corner_latch.py. With no corner in play the latch
// must be invisible.
func TestCornerLatchStraightReturnsRawReading(t *testing.T) {
	t.Parallel()
	var latch CornerLatch
	for range 20 {
		assertClose(t, latch.Update(0.0, 0.0, latchThreshold), 0.0, "straight")
	}
	if latch.IsLatched() {
		t.Error("armed on a straight")
	}
}

// 0.197 rad is what a NARROW corridor's corner actually previewed (measured
// on run_20260830_013702). It is below the shipped threshold, so the latch
// must not arm -- holding a signal that never armed would turn a threshold
// question into a latch question and hide the real issue.
func TestCornerLatchBelowThresholdNeverArms(t *testing.T) {
	t.Parallel()
	var latch CornerLatch
	for range 10 {
		assertClose(t, latch.Update(0.197, 0.0, latchThreshold), 0.197, "below threshold")
	}
	if latch.IsLatched() {
		t.Error("armed below the threshold")
	}
}

// The failure this exists for: armed on approach, decayed mid-corner.
//
// Sequence from run_20260830_014612's first corner (west -> south, 6.1 s),
// where the raw signal ran 1.373 -> 0.980 -> 0.590 -> 0.197 -> 0.000 and the
// lookahead went long two seconds BEFORE the corner.
func TestCornerLatchPreviewSurvivesDecayingToZero(t *testing.T) {
	t.Parallel()
	var latch CornerLatch
	assertClose(t, latch.Update(1.373, 0.00, latchThreshold), 1.373, "approach")
	assertClose(t, latch.Update(0.980, 0.05, latchThreshold), 0.980, "approach")
	assertClose(t, latch.Update(0.590, 0.10, latchThreshold), 0.590, "approach")
	// Below threshold now, but the chassis has barely turned.
	assertClose(t, latch.Update(0.197, 0.15, latchThreshold), 1.373, "decayed, held")
	assertClose(t, latch.Update(0.000, 0.30, latchThreshold), 1.373, "zeroed, held")
	if !latch.IsLatched() {
		t.Error("released mid-arc")
	}
}

func TestCornerLatchReleasesOnceTheTurnHasBeenDriven(t *testing.T) {
	t.Parallel()
	var latch CornerLatch
	latch.Update(1.0, 0.0, latchThreshold)
	assertClose(t, latch.Update(0.0, 0.50, latchThreshold), 1.0, "released half way round")
	// 0.8 of the previewed 1.0 rad is the completion test.
	assertClose(t, latch.Update(0.0, 0.85, latchThreshold), 1.0, "release tick still reports held")
	if latch.IsLatched() {
		t.Error("still latched after the turn was driven")
	}
	assertClose(t, latch.Update(0.0, 0.90, latchThreshold), 0.0, "still held after the turn was driven")
}

// A preview still growing when it arms must not be pinned low.
func TestCornerLatchHoldsTheLargestPreviewNotTheLast(t *testing.T) {
	t.Parallel()
	var latch CornerLatch
	latch.Update(0.40, 0.00, latchThreshold)
	latch.Update(1.20, 0.02, latchThreshold)
	latch.Update(0.50, 0.04, latchThreshold)
	// Turned 0.5 rad: past 0.8*0.5 but nowhere near 0.8*1.2.
	assertClose(t, latch.Update(0.0, 0.50, latchThreshold), 1.20, "pinned to the last preview")
}

// Yaw is wrapped, so a corner straddling +-pi must not read as zero.
func TestCornerLatchWrappingPastPiStillMeasuresTheTurn(t *testing.T) {
	t.Parallel()
	var latch CornerLatch
	latch.Update(1.0, 3.0, latchThreshold)
	held := latch.Update(0.0, -3.0, latchThreshold)

	turned := math.Abs(math.Atan2(math.Sin(-3.0-3.0), math.Cos(-3.0-3.0)))
	if math.Abs(turned-0.2832) > 1e-3 {
		t.Fatalf("sanity: this is a small turn, not 6 rad; got %v", turned)
	}
	assertClose(t, held, 1.0, "released on a wrap artefact")
}

// A failed escape can spin; |yaw - yawAtArm| alone is not monotone. Without
// the accumulated-yaw backstop a spin re-satisfies the release test only
// periodically, so the latch could hold for a long time.
func TestCornerLatchSpinningRobotReleasesWithinOneRevolution(t *testing.T) {
	t.Parallel()
	var latch CornerLatch
	// Previewed turn larger than any real corner.
	latch.Update(6.0, 0.0, latchThreshold)

	yaw := 0.0
	for range 200 {
		yaw = math.Atan2(math.Sin(yaw+0.1), math.Cos(yaw+0.1))
		latch.Update(0.0, yaw, latchThreshold)
		if !latch.IsLatched() {
			break
		}
	}
	if latch.IsLatched() {
		t.Error("held past a full revolution")
	}
}

func TestCornerLatchResetClearsACornerInProgress(t *testing.T) {
	t.Parallel()
	var latch CornerLatch
	latch.Update(1.0, 0.0, latchThreshold)
	if !latch.IsLatched() {
		t.Fatal("did not arm")
	}
	latch.Reset()
	if latch.IsLatched() {
		t.Error("reset did not clear")
	}
	assertClose(t, latch.Update(0.0, 0.0, latchThreshold), 0.0, "after reset")
}
