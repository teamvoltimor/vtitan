package bayexit_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/bayexit"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
)

// There is no Python oracle test file for bay_exit.py (none exists in the
// source tree, confirmed against platform/robot/tests), so these tests pin
// the module's own documented behavioral contracts directly.

const creepSpeedMPS = 0.15

// uniformScan returns a 360-ray scan of constant range, with angles evenly
// spaced over a full turn -- enough for NearestRay/ForwardClearance to have
// real bearings to search.
func uniformScan(rangeM float64) controllers.LidarScan {
	const n = 360
	rangesM := make([]float64, n)
	anglesRad := make([]float64, n)
	for i := range n {
		rangesM[i] = rangeM
		anglesRad[i] = -math.Pi + 2*math.Pi*float64(i)/float64(n)
	}
	return controllers.LidarScan{RangesM: rangesM, AnglesRad: anglesRad}
}

// scanWithSides is a uniform far scan, with every ray in the +90deg
// (left) and -90deg (right) sectors overridden -- resolveOpenSide now scores
// a whole BayExitOpenSideSectorDeg-wide sector (valid fraction times median
// range) rather than a single nearest ray, so a single overridden ray would
// be swamped by the uniform background and never move the score.
func scanWithSides(leftM, rightM float64) controllers.LidarScan {
	scan := uniformScan(5.0)
	const halfWidthRad = 15.0 * math.Pi / 180.0
	for i, a := range scan.AnglesRad {
		if math.Abs(a-math.Pi/2) <= halfWidthRad {
			scan.RangesM[i] = leftM
		}
		if math.Abs(a+math.Pi/2) <= halfWidthRad {
			scan.RangesM[i] = rightM
		}
	}
	return scan
}

func TestIsClear_BelowThresholdIsNotClear(t *testing.T) {
	t.Parallel()

	cfg := bayexit.DefaultConfig()
	scan := uniformScan(cfg.Follower.MinForwardClearanceM - 0.05)
	if bayexit.IsClear(scan, cfg) {
		t.Error("IsClear() = true, want false below MinForwardClearanceM")
	}
}

func TestIsClear_AboveThresholdIsClear(t *testing.T) {
	t.Parallel()

	cfg := bayexit.DefaultConfig()
	scan := uniformScan(cfg.Follower.MinForwardClearanceM + 0.50)
	if !bayexit.IsClear(scan, cfg) {
		t.Error("IsClear() = false, want true above MinForwardClearanceM")
	}
}

// TestCommand_LegacyForwardLegSteersTowardTheOpenSide matches the
// documented contract: once the reverse leg (BayExitReverseM) is behind
// it, the legacy exit steers toward whichever side resolveOpenSide found
// open, positive (left) when open is left.
func TestCommand_LegacyForwardLegSteersTowardTheOpenSide(t *testing.T) {
	t.Parallel()

	cfg := bayexit.DefaultConfig()
	cfg.Follower.BayExitCycle = false
	cfg.Follower.BayExitClearanceGuard = false
	cfg.Follower.BayExitReverseM = 0.01 // trivially small: reach the forward leg fast

	for _, openLeft := range []bool{true, false} {
		var left, right float64 = 1.0, 0.30
		if !openLeft {
			left, right = 0.30, 1.0
		}
		b := bayexit.New()
		scan := scanWithSides(left, right)

		var cmd struct {
			SpeedMPS, SteeringNorm float64
		}
		travelled := 0.0
		for range 20 {
			travelled -= 0.01 // reversing: signed odometry counts DOWN
			c := b.Command(scan, travelled, creepSpeedMPS, cfg, nil)
			cmd.SpeedMPS, cmd.SteeringNorm = c.SpeedMPS, c.SteeringNorm
		}
		if openLeft && cmd.SteeringNorm <= 0 {
			t.Errorf("open left: SteeringNorm = %v, want > 0", cmd.SteeringNorm)
		}
		if !openLeft && cmd.SteeringNorm >= 0 {
			t.Errorf("open right: SteeringNorm = %v, want < 0", cmd.SteeringNorm)
		}
	}
}

// TestCommand_ResolveOpenSideLatchesAfterFirstTick matches
// BayExitLatchDirection's documented contract: once the open side is read
// on the first tick, later ticks keep it even if the rays would now read
// the other way.
func TestCommand_ResolveOpenSideLatchesAfterFirstTick(t *testing.T) {
	t.Parallel()

	cfg := bayexit.DefaultConfig()
	cfg.Follower.BayExitLatchDirection = true
	b := bayexit.New()

	// Ticks 1..BayExitOpenSideVotes: open is left, polled before the latch
	// takes (see BayExitOpenSideVotes).
	scan := scanWithSides(1.0, 0.30)
	travelled := 0.0
	var latched struct{ SteeringNorm float64 }
	for range cfg.Follower.BayExitOpenSideVotes {
		c := b.Command(scan, travelled, creepSpeedMPS, cfg, nil)
		latched.SteeringNorm = c.SteeringNorm
		travelled -= 0.01
	}

	// Next tick: rays now say open is RIGHT -- the latch must ignore this.
	scan2 := scanWithSides(0.30, 1.0)
	c2 := b.Command(scan2, travelled, creepSpeedMPS, cfg, nil)

	if sign(latched.SteeringNorm) != sign(c2.SteeringNorm) {
		t.Errorf(
			"steering sign changed across the latch: latched=%v post-latch=%v, want same sign",
			latched.SteeringNorm, c2.SteeringNorm,
		)
	}
	if flips := b.OpenFlips(); flips != 0 {
		t.Errorf("OpenFlips() = %v, want 0 (latched)", flips)
	}
}

// TestCommand_ResolveOpenSideCountsFlipsWhenNotLatched matches the
// documented "count the flips rather than assuming stability" contract.
func TestCommand_ResolveOpenSideCountsFlipsWhenNotLatched(t *testing.T) {
	t.Parallel()

	cfg := bayexit.DefaultConfig()
	cfg.Follower.BayExitLatchDirection = false
	b := bayexit.New()

	scan1 := scanWithSides(1.0, 0.30)
	b.Command(scan1, 0.0, creepSpeedMPS, cfg, nil)

	scan2 := scanWithSides(0.30, 1.0)
	b.Command(scan2, -0.01, creepSpeedMPS, cfg, nil)

	if flips := b.OpenFlips(); flips != 1 {
		t.Errorf("OpenFlips() = %v, want 1", flips)
	}
}

func sign(v float64) int {
	switch {
	case v > 0:
		return 1
	case v < 0:
		return -1
	default:
		return 0
	}
}

// TestCommand_CycleUnobstructedFirstLegNeverInternallyTransitions matches
// the documented DEFAULT-model contract: bay_exit.py's own
// BAY_EXIT_FORWARD_M docstring states that under the legal (non
// --solid-walls) contact model, "the manoeuvre is released by IsClear
// after ~22 ticks on a single forward arc and the reverse leg never runs."
//
// This falls out of the leg-start bookkeeping directly: the very first
// forward leg has no anchored start (legStartM is nil until the first ever
// beginLeg call), so its distance check reads travelledM against ITSELF
// and is permanently zero -- the leg can only end via the stall backstop.
// Under continuous, unobstructed forward motion (real wheel travel every
// tick, well above legStallEpsilonM), stall never fires, so the leg simply
// never transitions on its own; in production it is the CALLER's separate
// IsClear(...) check (LIDAR forward clearance, nothing to do with this
// leg-tracking) that stops the manoeuvre once the chassis has genuinely
// left the pocket.
func TestCommand_CycleUnobstructedFirstLegNeverInternallyTransitions(t *testing.T) {
	t.Parallel()

	cfg := bayexit.DefaultConfig()
	cfg.Follower.BayExitCycle = true
	cfg.Follower.BayExitClearanceGuard = false
	b := bayexit.New()
	scan := scanWithSides(1.0, 0.30)

	travelled := 0.0
	for range 500 {
		travelled += creepSpeedMPS * cfg.Follower.CornerSpeedScale / cfg.ControlHz
		b.Command(scan, travelled, creepSpeedMPS, cfg, nil)
	}
	reverseTicks, forwardTicks, _ := b.Legs()
	if reverseTicks != 0 {
		t.Errorf("reverseTicks = %v, want 0 (unobstructed first leg never internally transitions)", reverseTicks)
	}
	if forwardTicks != 500 {
		t.Errorf("forwardTicks = %v, want 500 (every tick stayed on the first forward leg)", forwardTicks)
	}
	if cycles := b.Cycles(); cycles != 0 {
		t.Errorf("Cycles() = %v, want 0", cycles)
	}
}

// TestCommand_CycleStalledLegTransitionsAndAnchorsTheNextOne is the
// complementary case: a genuinely jammed leg (no wheel travel for
// BayExitLegStallTicks, matching --solid-walls grinding against a fin) DOES
// end via the stall backstop, and that first beginLeg call anchors
// legStartM so every leg after it is bookkept normally.
func TestCommand_CycleStalledLegTransitionsAndAnchorsTheNextOne(t *testing.T) {
	t.Parallel()

	cfg := bayexit.DefaultConfig()
	cfg.Follower.BayExitCycle = true
	cfg.Follower.BayExitClearanceGuard = false
	b := bayexit.New()
	scan := scanWithSides(1.0, 0.30)

	// Hold well past both the stall threshold AND the servo settle the
	// leg switch then budgets (computed from the swing between the
	// forward and reverse target angles) -- the settle phase commands
	// zero speed and returns before reverseTicks would increment.
	for range 40 {
		b.Command(scan, 0.0, creepSpeedMPS, cfg, nil)
	}
	if reverseTicks, _, _ := b.Legs(); reverseTicks == 0 {
		t.Fatal("the stall backstop never ended the jammed first forward leg")
	}
}

// TestCommand_GuardedCommandRecordsGuardStats: GuardStats must report a
// real predicted gap once the guard has run.
func TestCommand_GuardedCommandRecordsGuardStats(t *testing.T) {
	t.Parallel()

	cfg := bayexit.DefaultConfig()
	cfg.Follower.BayExitClearanceGuard = true
	b := bayexit.New()
	scan := scanWithSides(1.0, 0.30)

	b.Command(scan, 0.0, creepSpeedMPS, cfg, nil)
	_, minGap, ok, _ := b.GuardStats()
	if !ok {
		t.Fatal("GuardStats() ok = false after Command(), want a recorded predicted gap")
	}
	if minGap <= 0 {
		t.Errorf("GuardStats() min gap = %v, want > 0 on the very first tick", minGap)
	}
}

// TestCommand_GuardedCommandFlipsBeforePredictedContact is the core safety
// property BayExitClearanceGuard exists for. Driven SELF-CONSISTENTLY --
// each tick's travelled_m is advanced by the PREVIOUS tick's own returned
// speed, matching what a real robot faithfully executing the command would
// report back -- the guard must flip the leg at least once (the chassis
// cannot otherwise reach open ground from a standing start inside the
// pocket) and the min predicted gap it ever recorded must never fall
// meaningfully below -MarginM: the prediction that triggers a flip is
// itself bounded by one tick's motion past the margin, not an unbounded
// overshoot.
func TestCommand_GuardedCommandFlipsBeforePredictedContact(t *testing.T) {
	t.Parallel()

	cfg := bayexit.DefaultConfig()
	cfg.Follower.BayExitClearanceGuard = true
	b := bayexit.New()
	scan := scanWithSides(1.0, 0.30)

	travelled := 0.0
	minGapSeen := math.Inf(1)
	for range 300 {
		cmd := b.Command(scan, travelled, creepSpeedMPS, cfg, nil)
		travelled += cmd.SpeedMPS / cfg.ControlHz
		if _, minGap, ok, _ := b.GuardStats(); ok && minGap < minGapSeen {
			minGapSeen = minGap
		}
	}
	flips, _, _, _ := b.GuardStats()
	if flips == 0 {
		t.Error("guard never flipped a leg over 300 self-consistent ticks")
	}
	// One tick's worth of travel past the margin is the loosest bound that
	// still catches an unbounded overshoot; the shipped speed/control-rate
	// combination moves well under a centimetre per tick.
	oneTickSlackM := creepSpeedMPS / cfg.ControlHz
	if minGapSeen < -cfg.Follower.BayExitClearanceMarginM-oneTickSlackM {
		t.Errorf(
			"min predicted gap = %v, want >= -(margin + one tick of travel) = %v",
			minGapSeen, -cfg.Follower.BayExitClearanceMarginM-oneTickSlackM,
		)
	}
}

// TestCommand_GuardedCommandToleranceLetsTheGuardAcceptMoreOverlap ports the
// BAY_EXIT_CLEARANCE_TOLERANCE_M behavior: subtracted from
// BayExitClearanceMarginM, so a positive tolerance shrinks the effective
// threshold the guard flips a leg at, letting the predicted gap run deeper
// negative (a confidently-inside-a-fin overlap) before a flip is forced.
// Ships 0.0, which must reproduce
// TestCommand_GuardedCommandFlipsBeforePredictedContact's behavior exactly
// -- this only pins the NON-default value actually changes something.
func TestCommand_GuardedCommandToleranceLetsTheGuardAcceptMoreOverlap(t *testing.T) {
	t.Parallel()

	const toleranceM = 0.01

	runMinGap := func(toleranceM float64) float64 {
		cfg := bayexit.DefaultConfig()
		cfg.Follower.BayExitClearanceGuard = true
		cfg.Follower.BayExitClearanceToleranceM = toleranceM
		b := bayexit.New()
		scan := scanWithSides(1.0, 0.30)

		travelled := 0.0
		minGapSeen := math.Inf(1)
		for range 300 {
			cmd := b.Command(scan, travelled, creepSpeedMPS, cfg, nil)
			travelled += cmd.SpeedMPS / cfg.ControlHz
			if _, minGap, ok, _ := b.GuardStats(); ok && minGap < minGapSeen {
				minGapSeen = minGap
			}
		}
		return minGapSeen
	}

	withoutTolerance := runMinGap(0.0)
	withTolerance := runMinGap(toleranceM)

	if withTolerance >= withoutTolerance {
		t.Errorf(
			"min predicted gap with tolerance = %v, want < without-tolerance's %v "+
				"(a positive tolerance must let the guard accept a deeper predicted overlap)",
			withTolerance, withoutTolerance,
		)
	}
}
