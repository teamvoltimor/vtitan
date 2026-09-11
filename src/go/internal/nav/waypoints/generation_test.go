package waypoints_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/waypoints"
)

const chassisWidthM = 0.194 // robot.toml's [chassis] width

func TestValidatePathFeasibility_BiasConsumesMargin(t *testing.T) {
	t.Parallel()

	centred := waypoints.ValidatePathFeasibility(narrowWidthM, 0.0, chassisWidthM)
	biased := waypoints.ValidatePathFeasibility(narrowWidthM, 0.05, chassisWidthM)

	if !centred.IsFeasible || !biased.IsFeasible {
		t.Fatalf(
			"expected both feasible: centred=%v biased=%v",
			centred.IsFeasible,
			biased.IsFeasible,
		)
	}
	// Biasing 0.05 off center spends 0.05 at each wall.
	if got := centred.MarginM - biased.MarginM; math.Abs(got-0.10) > 1e-9 {
		t.Errorf("margin difference = %v, want 0.10", got)
	}
}

func TestValidatePathFeasibility_BiasDirectionDoesNotMatter(t *testing.T) {
	t.Parallel()

	pos := waypoints.ValidatePathFeasibility(narrowWidthM, 0.05, chassisWidthM)
	neg := waypoints.ValidatePathFeasibility(narrowWidthM, -0.05, chassisWidthM)
	if math.Abs(pos.MarginM-neg.MarginM) > 1e-9 {
		t.Errorf("MarginM differs by bias sign: +0.05=%v -0.05=%v", pos.MarginM, neg.MarginM)
	}
}

func TestValidatePathFeasibility_RejectsATooNarrowCorridor(t *testing.T) {
	t.Parallel()

	got := waypoints.ValidatePathFeasibility(chassisWidthM*0.75, 0.0, chassisWidthM)
	if got.IsFeasible {
		t.Fatal("IsFeasible = true, want false (corridor narrower than the chassis)")
	}
	if got.Reason == "" {
		t.Error("Reason is empty, want an explanation")
	}
}

func TestCenterBiasForCorridor_NarrowVsWideSplit(t *testing.T) {
	t.Parallel()

	cfg := waypoints.DefaultConfig()
	cfg.NarrowCenterBiasM = 0.05
	cfg.WideCenterBiasM = 0.08

	if got := waypoints.CenterBiasForCorridor(narrowWidthM, cfg, nil, true); got != 0.05 {
		t.Errorf("narrow corridor bias = %v, want 0.05", got)
	}
	if got := waypoints.CenterBiasForCorridor(wideWidthM, cfg, nil, true); got != 0.08 {
		t.Errorf("wide corridor bias = %v, want 0.08", got)
	}
}

func TestCenterBiasForCorridor_EachWidthClassTakesItsOwnSide(t *testing.T) {
	t.Parallel()

	base := waypoints.DefaultConfig()
	base.NarrowCenterBiasM = 0.05
	base.WideCenterBiasM = 0.05

	flipped := base
	flipped.NarrowCenterBiasSide = trackmodel.Outer

	// Flipping only the narrow side must move a narrow corridor's shift...
	got := waypoints.CenterBiasForCorridor(narrowWidthM, flipped, nil, true)
	want := -waypoints.CenterBiasForCorridor(narrowWidthM, base, nil, true)
	if got != want {
		t.Errorf("narrow bias after flipping narrow side = %v, want %v", got, want)
	}
	// ...and leave a wide corridor's alone.
	got = waypoints.CenterBiasForCorridor(wideWidthM, flipped, nil, true)
	want = waypoints.CenterBiasForCorridor(wideWidthM, base, nil, true)
	if got != want {
		t.Errorf("wide bias after flipping narrow side = %v, want unchanged %v", got, want)
	}
}

func TestCenterBiasForCorridor_OverrideAppliesUniformlyOnTheWideSide(t *testing.T) {
	t.Parallel()

	cfg := waypoints.DefaultConfig()
	cfg.NarrowCenterBiasM = 0.0 // would otherwise apply to the narrow corridor below
	cfg.WideCenterBiasSide = trackmodel.Outer

	override := 0.15
	// Even a NARROW-width corridor takes the override uniformly, on the
	// WIDE side setting -- not the narrow split.
	got := waypoints.CenterBiasForCorridor(narrowWidthM, cfg, &override, true)
	if got != -0.15 {
		t.Errorf("overridden bias = %v, want -0.15 (magnitude 0.15, outer side)", got)
	}
}
