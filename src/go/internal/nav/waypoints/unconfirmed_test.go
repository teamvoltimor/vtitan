package waypoints_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/waypoints"
)

// blindNarrowWidthM is the width a blind round assumes for every corridor,
// and wideTruthWidthM is what a wide one actually measures. The 0.30 m step
// this whole mechanism exists to shrink is the difference between the lines
// planned from these two, since both hypotheses share the fixed outer wall.
const (
	blindNarrowWidthM = 0.6
	wideTruthWidthM   = 1.0
)

func TestCenterBiasForCorridor_UnconfirmedNarrowTakesTheInnerBias(t *testing.T) {
	t.Parallel()

	cfg := waypoints.DefaultConfig()

	confirmed := waypoints.CenterBiasForCorridor(blindNarrowWidthM, cfg, nil, true)
	unconfirmed := waypoints.CenterBiasForCorridor(blindNarrowWidthM, cfg, nil, false)

	if confirmed != cfg.NarrowCenterBiasM {
		t.Fatalf("confirmed narrow bias = %g, want NarrowCenterBiasM (%g)",
			confirmed, cfg.NarrowCenterBiasM)
	}
	if unconfirmed != cfg.UnconfirmedWidthInnerBiasM {
		t.Fatalf("unconfirmed narrow bias = %g, want UnconfirmedWidthInnerBiasM (%g)",
			unconfirmed, cfg.UnconfirmedWidthInnerBiasM)
	}
	// Positive is toward the INNER block. Pre-positioning outward would
	// enlarge the step instead of shrinking it.
	if unconfirmed <= 0 {
		t.Fatalf("unconfirmed narrow bias = %g, want positive (inner)", unconfirmed)
	}
}

func TestCenterBiasForCorridor_UnconfirmedShrinksTheBeliefStep(t *testing.T) {
	t.Parallel()

	cfg := waypoints.DefaultConfig()
	const maxCoordM = 1.5

	// The planned centreline y for a north corridor is MAX - width/2 - bias.
	lineFor := func(widthM float64, confirmed bool) float64 {
		return maxCoordM - widthM/2 - waypoints.CenterBiasForCorridor(widthM, cfg, nil, confirmed)
	}

	// Before: the blind prior planned with the CONFIRMED narrow bias.
	stepBefore := math.Abs(lineFor(blindNarrowWidthM, true) - lineFor(wideTruthWidthM, true))
	// After: the blind prior is pre-positioned inward while still a guess.
	stepAfter := math.Abs(lineFor(blindNarrowWidthM, false) - lineFor(wideTruthWidthM, true))

	if stepAfter >= stepBefore {
		t.Fatalf("belief step did not shrink: before %.3f m, after %.3f m", stepBefore, stepAfter)
	}
	if math.Abs(stepBefore-stepAfter-cfg.UnconfirmedWidthInnerBiasM) > 1e-12 {
		t.Fatalf("step shrank by %.4f m, want exactly UnconfirmedWidthInnerBiasM (%.4f m)",
			stepBefore-stepAfter, cfg.UnconfirmedWidthInnerBiasM)
	}
	// The step cannot be cancelled: the remainder is what a plan that must
	// still fit a genuinely narrow corridor cannot give back.
	if stepAfter <= 0 {
		t.Fatalf("step fully cancelled (%.4f m) -- that would put the line on the "+
			"inner wall of a truly narrow corridor", stepAfter)
	}
}

func TestCenterBiasForCorridor_WideIsUnaffectedByConfirmedness(t *testing.T) {
	t.Parallel()

	cfg := waypoints.DefaultConfig()
	// The bias only substitutes for the NARROW branch: a corridor already
	// believed wide has nothing to pre-position toward.
	if got, want := waypoints.CenterBiasForCorridor(wideTruthWidthM, cfg, nil, false),
		waypoints.CenterBiasForCorridor(wideTruthWidthM, cfg, nil, true); got != want {
		t.Fatalf("wide bias unconfirmed = %g, confirmed = %g; want equal", got, want)
	}
}

func TestCenterBiasForCorridor_OverrideBeatsConfirmedness(t *testing.T) {
	t.Parallel()

	cfg := waypoints.DefaultConfig()
	override := 0.0
	// The Obstacles override was swept and measured as ONE number. Letting
	// confirmed-ness substitute underneath it would silently make it mean two.
	if got := waypoints.CenterBiasForCorridor(blindNarrowWidthM, cfg, &override, false); got != 0.0 {
		t.Fatalf("override with unconfirmed width = %g, want 0.0", got)
	}
}

func TestUnconfirmedSections_ConfirmedIsTheComplement(t *testing.T) {
	t.Parallel()

	all := waypoints.AllUnconfirmed()
	none := waypoints.AllConfirmed()
	for _, section := range []trackmodel.Section{
		trackmodel.North, trackmodel.South, trackmodel.East, trackmodel.West,
	} {
		if !all.Contains(section) || all.Confirmed(section) {
			t.Errorf("AllUnconfirmed: %v reads as confirmed", section)
		}
		if none.Contains(section) || !none.Confirmed(section) {
			t.Errorf("AllConfirmed: %v reads as unconfirmed", section)
		}
	}
}

func TestUnconfirmedSections_SetTogglesOneSection(t *testing.T) {
	t.Parallel()

	set := waypoints.AllUnconfirmed()
	set.Set(trackmodel.East, false)

	if !set.Confirmed(trackmodel.East) {
		t.Error("East did not become confirmed")
	}
	if set.Confirmed(trackmodel.North) {
		t.Error("confirming East also confirmed North")
	}
}
