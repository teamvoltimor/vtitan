package encoder

// Decoder is the pure A/B quadrature state machine: feed it successive
// channel levels and it accumulates a signed count. It is deliberately
// separate from the GPIO plumbing in quadrature.go so the decode logic is
// testable on any platform, the same split button.Evaluator draws against
// button.Driver.
//
// Python delegates this half to gpiozero's RotaryEncoder, so there is no
// line-for-line oracle to port; what is ported is the CONTRACT that
// counts_per_rev is calibrated against whatever the decoder emits. See
// CountsPerEdge for the consequence.
//
// Not safe for concurrent use.
type Decoder struct {
	haveState bool
	state     uint8
	counts    int64
}

// CountsPerEdge records that this decoder counts every valid quadrature
// edge -- "4x decoding", four counts per full A/B cycle.
//
// This matters because counts_per_rev is BENCH-CALIBRATED against the
// decoder that produced it (60 counts/rev, live-verified 2026-08-29
// against gpiozero's RotaryEncoder), not derived from the encoder's
// datasheet. If gpiozero counts at a different rate than 4x, distance from
// this driver is off by exactly that ratio. Re-run
// platform/robot/scripts/hardware/calibrate_encoder.py against THIS driver
// before trusting its distance on hardware -- the value is a measurement of
// a driver-plus-drivetrain pair, not of the encoder alone.
const CountsPerEdge = 1

// quadratureTransitions maps a (previous << 2) | current two-bit channel
// state onto the count delta for that transition. State is (A << 1) | B, so
// forward rotation walks the Gray sequence 00 -> 01 -> 11 -> 10 and reverse
// walks it backwards. Entries for "no change" and for the two-bits-changed
// transitions that a missed edge or contact bounce produces are 0: a
// transition that skipped a state carries no reliable direction, and
// guessing one would inject phantom travel into the odometry.
var quadratureTransitions = [16]int64{
	0, 1, -1, 0,
	-1, 0, 0, 1,
	1, 0, 0, -1,
	0, -1, 1, 0,
}

// Sample folds in one reading of the A and B channels and returns the
// accumulated count. The first sample after construction or Reset only
// establishes the reference state and contributes no count.
func (d *Decoder) Sample(a, b bool) int64 {
	state := uint8(0)
	if a {
		state |= 0b10
	}
	if b {
		state |= 0b01
	}

	if !d.haveState {
		d.haveState = true
		d.state = state
		return d.counts
	}

	d.counts += quadratureTransitions[d.state<<2|state]
	d.state = state
	return d.counts
}

// Counts returns the accumulated signed count without sampling.
func (d *Decoder) Counts() int64 {
	return d.counts
}

// Reset zeroes the count and forgets the reference state, so the next
// Sample re-establishes it rather than differencing against a level read
// before the reset.
func (d *Decoder) Reset() {
	d.haveState = false
	d.state = 0
	d.counts = 0
}
