package scenario

import (
	"context"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/startconditions"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/waypoints"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/harness"
)

// noiselessConfig is the harness with LIDAR noise and dropout off, so a blind
// run's width readings are the geometry rather than the sampler -- these
// tests are about what the runner WITHHOLDS, not about sensor realism.
func noiselessConfig() harness.Config {
	cfg := harness.DefaultConfig()
	cfg.LidarNoiseStd = 0.0
	cfg.InvalidRayRate = 0.0
	return cfg
}

func TestNativeRunner_BlindPlansFromThePriorNotTheTruth(t *testing.T) {
	t.Parallel()

	cfg := noiselessConfig()
	base := waypoints.PlannerInput{
		MaxCoordM:     cfg.TrackMaxCoordM,
		ChassisWidthM: cfg.ChassisWidthM,
	}

	blind, err := newBlindSetup(
		base, trackmodel.Counterclockwise, false, cfg, waypoints.DefaultConfig(), startconditions.DefaultConfig(), nil,
	)
	if err != nil {
		t.Fatalf("newBlindSetup: %v", err)
	}
	if len(blind.Waypoints) == 0 {
		t.Fatal("blind setup produced an empty path")
	}
	if blind.Layout == nil {
		t.Fatal("blind setup produced no layout-belief loop")
	}

	// The prior is NARROW everywhere, so the south corridor's line sits at
	// blindNarrowWidthM/2 plus the unconfirmed inner bias -- NOT at the
	// 1.0 m truth's half-width. If the runner leaked the true geometry in,
	// the southernmost waypoint would sit at 0.5 instead.
	wantY := blindNarrowWidthM/2 + waypoints.DefaultConfig().UnconfirmedWidthInnerBiasM
	minY := blind.Waypoints[0].Y
	for _, wp := range blind.Waypoints {
		minY = min(minY, wp.Y)
	}
	if diff := minY - wantY; diff < -0.05 || diff > 0.05 {
		t.Fatalf("southernmost planned y = %.3f, want ~%.3f (the narrow prior's "+
			"line, pre-positioned inward); a value near %.3f would mean the true "+
			"width leaked into a blind plan",
			minY, wantY, 1.0/2)
	}
}

func TestNativeRunner_BlindObstaclesUsesTheRuleWidthPrior(t *testing.T) {
	t.Parallel()

	cfg := noiselessConfig()
	base := waypoints.PlannerInput{
		MaxCoordM:     cfg.TrackMaxCoordM,
		ChassisWidthM: cfg.ChassisWidthM,
	}

	// Obstacles corridors are 1.0 m by RULE, so believing that is prior
	// knowledge rather than a guess -- and the bias is the explicit override,
	// not the narrow/wide split.
	blind, err := newBlindSetup(
		base, trackmodel.Counterclockwise, true, cfg, waypoints.DefaultConfig(),
		startconditions.DefaultConfig(), blindCenterBiasM(true),
	)
	if err != nil {
		t.Fatalf("newBlindSetup(obstacles): %v", err)
	}

	wantY := obstaclesCorridorWidthM/2 + obstaclesCenterBiasM
	minY := blind.Waypoints[0].Y
	for _, wp := range blind.Waypoints {
		minY = min(minY, wp.Y)
	}
	if diff := minY - wantY; diff < -0.05 || diff > 0.05 {
		t.Fatalf("southernmost planned y = %.3f, want ~%.3f (the 1.0 m rule width "+
			"biased by the Obstacles override)", minY, wantY)
	}
}

func TestNativeRunner_BlindRunInfersItsOwnDirection(t *testing.T) {
	t.Parallel()

	sc := writeTempMetadata(t)
	cfg := noiselessConfig()
	runner := NewNativeRunner(NativeRunnerConfig{
		Harness: &cfg, Seed: 1, MaxSteps: 4000, Blind: true,
	})

	res, err := runner.Run(context.Background(), sc)
	if err != nil {
		t.Fatalf("blind run returned error: %v", err)
	}

	// The navigator starts with Direction nil and creeps until LIDAR settles
	// one. A run that never leaves the creep never travels, so any real
	// distance is evidence the bootstrap completed -- and it must, since
	// nothing else hands it a direction.
	if res.Steps == 0 {
		t.Fatal("blind run produced no ticks")
	}
	if res.DistanceM <= 0 {
		t.Fatalf("blind run travelled %.3f m; the direction bootstrap never "+
			"settled, so the robot never left the creep", res.DistanceM)
	}
}

func TestNativeRunner_SightedRunIsUnaffected(t *testing.T) {
	t.Parallel()

	sc := writeTempMetadata(t)
	cfg := noiselessConfig()

	// The blind flag defaults off, and a sighted run must be byte-identical
	// to what it was before blind mode existed -- every recorded corpus
	// number was measured sighted, so a silent change there would invalidate
	// all of them at once.
	first, err := NewNativeRunner(NativeRunnerConfig{
		Harness: &cfg, Seed: 1, MaxSteps: 4000,
	}).Run(context.Background(), sc)
	if err != nil {
		t.Fatalf("sighted run: %v", err)
	}
	second, err := NewNativeRunner(NativeRunnerConfig{
		Harness: &cfg, Seed: 1, MaxSteps: 4000, Blind: false,
	}).Run(context.Background(), sc)
	if err != nil {
		t.Fatalf("sighted run (explicit Blind=false): %v", err)
	}

	if first.Steps != second.Steps || first.DistanceM != second.DistanceM {
		t.Fatalf("Blind=false differs from the zero value: steps %d vs %d, "+
			"distance %.6f vs %.6f", first.Steps, second.Steps,
			first.DistanceM, second.DistanceM)
	}
}
