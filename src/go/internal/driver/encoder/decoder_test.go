package encoder_test

import (
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/driver/encoder"
)

// abStates walks the quadrature Gray sequence 00 -> 01 -> 11 -> 10, the
// order the A/B channels take for one full forward cycle.
var abStates = [4][2]bool{
	{false, false},
	{false, true},
	{true, true},
	{true, false},
}

// sampleCycles feeds n full cycles through d, forward when forward is true
// and backward otherwise, and returns the resulting count.
func sampleCycles(d *encoder.Decoder, cycles int, forward bool) int64 {
	var counts int64
	for range cycles {
		for step := range abStates {
			index := step
			if !forward {
				index = len(abStates) - 1 - step
			}
			counts = d.Sample(abStates[index][0], abStates[index][1])
		}
	}
	return counts
}

func TestDecoder_ForwardCycleCountsFourEdges(t *testing.T) {
	t.Parallel()

	var d encoder.Decoder
	// Seed on 10, the state a forward cycle ENDS on, so the walk's opening
	// 00 is a real transition rather than a repeat of the seed and all 12
	// transitions across 3 cycles are counted.
	d.Sample(true, false)

	if got := sampleCycles(&d, 3, true); got != 12 {
		t.Fatalf("forward 3 cycles: counts = %d, want 12 (4x decoding)", got)
	}
}

func TestDecoder_ReverseCountsNegative(t *testing.T) {
	t.Parallel()

	var d encoder.Decoder
	d.Sample(false, false)

	if got := sampleCycles(&d, 3, false); got != -12 {
		t.Fatalf("reverse 3 cycles: counts = %d, want -12", got)
	}
}

func TestDecoder_ForwardThenReverseReturnsToZero(t *testing.T) {
	t.Parallel()

	var d encoder.Decoder
	d.Sample(false, false)

	sampleCycles(&d, 5, true)
	if got := sampleCycles(&d, 5, false); got != 0 {
		t.Fatalf("forward then reverse: counts = %d, want 0", got)
	}
}

func TestDecoder_FirstSampleOnlySeedsState(t *testing.T) {
	t.Parallel()

	var d encoder.Decoder
	// Without the seeding rule, an opening sample of 11 against a zeroed
	// reference state would be read as a two-bit jump and, worse, a
	// naive decoder could invent a direction for it.
	if got := d.Sample(true, true); got != 0 {
		t.Fatalf("first sample: counts = %d, want 0", got)
	}
}

func TestDecoder_SkippedStateContributesNothing(t *testing.T) {
	t.Parallel()

	var d encoder.Decoder
	d.Sample(false, false)
	// 00 -> 11 is two bits at once: a missed edge or bounce. It carries no
	// reliable direction, so it must not become phantom travel.
	if got := d.Sample(true, true); got != 0 {
		t.Fatalf("skipped state: counts = %d, want 0", got)
	}
	// Decoding must resume normally from the new state, not stay wedged.
	if got := d.Sample(true, false); got != 1 {
		t.Fatalf("after skipped state: counts = %d, want 1", got)
	}
}

func TestDecoder_RepeatedSampleIsNoOp(t *testing.T) {
	t.Parallel()

	var d encoder.Decoder
	d.Sample(false, false)
	d.Sample(false, true)
	before := d.Counts()
	d.Sample(false, true)
	d.Sample(false, true)
	if got := d.Counts(); got != before {
		t.Fatalf("repeated identical samples: counts = %d, want %d", got, before)
	}
}

func TestDecoder_ResetZeroesAndReseeds(t *testing.T) {
	t.Parallel()

	var d encoder.Decoder
	d.Sample(false, false)
	sampleCycles(&d, 2, true)

	d.Reset()
	if got := d.Counts(); got != 0 {
		t.Fatalf("after Reset: counts = %d, want 0", got)
	}
	// The first post-Reset sample re-establishes state rather than
	// differencing against the level held before the reset.
	if got := d.Sample(true, true); got != 0 {
		t.Fatalf("first sample after Reset: counts = %d, want 0", got)
	}
	if got := d.Sample(true, false); got != 1 {
		t.Fatalf("second sample after Reset: counts = %d, want 1", got)
	}
}
