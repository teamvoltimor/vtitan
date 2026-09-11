//go:build integration

package scenario_test

// Obstacles/parking parity gate. The sibling native_runner_integration_test.go
// gates the OPEN corpus; this one gates the Obstacles corpus, whose Result
// fields (traffic signs, pass-side, parking) the Open corpus never exercises.
// Until now the parking wiring was only cross-checked by eye against
// separately-documented Python conclusions -- that is evidence, but it is not
// a scenario-by-scenario A/B, and nothing failed when the two drifted.
//
// Gating follows the Open gate's policy exactly:
//   - behind the "integration" build tag, so `go test ./...` never runs it;
//   - NativeRunner always runs (no external interpreter needed);
//   - the Python oracle runs only when VTITAN_SIM_PYTHON is set, otherwise
//     the test asserts the native runner's own invariants over the corpus.
//
// Run:
//
//	go test -tags=integration ./internal/sim/scenario/... -run TestObstacles
//	VTITAN_SIM_PYTHON=python3 go test -tags=integration ./internal/sim/scenario/... -run TestObstacles_ParityVsPython

import (
	"fmt"
	"math"
	"os"
	"path/filepath"
	"slices"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/parking"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/corpus"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/scenario"
)

// obstaclesMaxSteps bounds a single Obstacles run. It matches
// NativeRunnerConfig's own parity default (4000 steps, ~200 s at 20 Hz) so
// the gate scores the same runs sim-runner does -- raising it here would
// make this gate and every recorded corpus sweep describe different runs.
const obstaclesMaxSteps = 4000

// paritySampleSize is how many scenarios the per-scenario parity subtest
// covers by default. The full corpus is 256 and each Python run is a
// subprocess, so the default is a sample; VTITAN_OBSTACLES_SAMPLE=0 runs all
// of them.
//
// This is a SAMPLE, not a screen: it does not license a claim about the
// corpus. Corpus-wide invariants are asserted separately, natively, over
// every scenario (TestObstacles_NativeCorpusInvariants).
const paritySampleSize = 12

func TestObstacles_ParityVsPython(t *testing.T) {
	t.Parallel()

	scenarios := loadObstaclesCorpus(t)
	if sample := obstaclesSampleSize(t); sample > 0 && len(scenarios) > sample {
		scenarios = scenarios[:sample]
	}

	native := scenario.NewNativeRunner(scenario.NativeRunnerConfig{MaxSteps: obstaclesMaxSteps})
	pyRunner := obstaclesPythonRunner(t)

	for _, sc := range scenarios {
		t.Run(sc.ID, func(t *testing.T) {
			t.Parallel()

			nativeRes, err := native.Run(t.Context(), sc)
			if err != nil {
				t.Fatalf("NativeRunner.Run(%s): %v", sc.ID, err)
			}
			assertResultPopulated(t, "native", sc.ID, nativeRes)
			assertObstaclesInvariants(t, "native", sc.ID, nativeRes)

			if pyRunner == nil {
				return
			}

			pyRes, err := pyRunner.Run(t.Context(), sc)
			if err != nil {
				t.Fatalf("SubprocessRunner.Run(%s): %v", sc.ID, err)
			}
			assertResultPopulated(t, "python", sc.ID, pyRes)

			assertResultParity(t, sc.ID, nativeRes, pyRes)
			assertObstaclesParity(t, sc.ID, nativeRes, pyRes)
		})
	}
}

// TestObstacles_NativeCorpusInvariants runs the WHOLE corpus through the
// native runner and asserts the properties that must hold for every scenario
// regardless of how well the robot drives, then logs the outcome
// distribution.
//
// It deliberately asserts invariants rather than pinning counts. A count
// ("2 of 256 score partial credit") is a measurement of today's navigator:
// pinning it makes every genuine nav improvement look like a regression, and
// sweep results are not comparable across time anyway. The invariants below
// are properties of the SCORING, which must not change silently.
func TestObstacles_NativeCorpusInvariants(t *testing.T) {
	t.Parallel()

	scenarios := loadObstaclesCorpus(t)

	// Reuse the Orchestrator rather than looping: it already owns the
	// bounded-concurrency fan-out sim-runner uses, and a serial loop over
	// 256 scenarios takes tens of minutes where the concurrent one takes
	// seconds.
	orchestrator, err := scenario.NewOrchestrator(
		scenario.NewNativeRunner(scenario.NativeRunnerConfig{MaxSteps: obstaclesMaxSteps}),
		scenario.OrchestratorConfig{},
	)
	if err != nil {
		t.Fatalf("NewOrchestrator: %v", err)
	}

	report, err := orchestrator.Run(t.Context(), scenarios)
	if err != nil {
		t.Fatalf("Orchestrator.Run over %d scenarios: %v", len(scenarios), err)
	}

	var (
		parkingScenarios int
		fullCredit       int
		partialCredit    int
		passSide         int
		collided         int
	)
	surfaces := make(map[string]int)

	for _, sr := range report.Results {
		if sr.Err != nil {
			t.Errorf("%s: %v", sr.Scenario.ID, sr.Err)
			continue
		}
		res := sr.Result
		assertObstaclesInvariants(t, "native", sr.Scenario.ID, res)

		surfaces[res.TerminalSurface]++
		if res.Collided {
			collided++
		}
		if res.PassSideViolation {
			passSide++
		}
		if res.ParkPoints != nil {
			parkingScenarios++
			switch *res.ParkPoints {
			case parking.FullParkPoints:
				fullCredit++
			case parking.PartialParkPoints:
				partialCredit++
			}
		}
	}

	if parkingScenarios == 0 {
		t.Fatal("no scenario in the corpus attached a ParkController; " +
			"the parking wiring is not being exercised at all")
	}
	t.Logf("obstacles corpus: %d scenarios, %d with parking (%d full / %d partial credit), "+
		"%d pass-side violations, %d collided, terminal surfaces %v",
		len(scenarios), parkingScenarios, fullCredit, partialCredit, passSide, collided, surfaces)
}

// assertObstaclesInvariants checks the properties every Obstacles result must
// satisfy on its own, independent of the Python oracle.
func assertObstaclesInvariants(t *testing.T, which, id string, r scenario.Result) {
	t.Helper()

	// Parked and ParkPoints describe the same maneuver from two angles (did
	// the controller finish / what would a judge award). One present without
	// the other means a scenario grew a parking lot in one code path but not
	// the other.
	if (r.Parked == nil) != (r.ParkPoints == nil) {
		t.Errorf("%s %s: Parked=%v but ParkPoints=%v -- both must be set or neither",
			which, id, r.Parked, r.ParkPoints)
	}

	if r.ParkPoints != nil {
		points := *r.ParkPoints
		if points != 0 && points != parking.PartialParkPoints && points != parking.FullParkPoints {
			t.Errorf("%s %s: ParkPoints = %d, want one of 0/%d/%d (the WRO tiers)",
				which, id, points, parking.PartialParkPoints, parking.FullParkPoints)
		}
		// Rule 9.24.7 as ruled 2026-09-03: contact with a parking-lot
		// limitation voids ALL parking points. A run that ended ON a fin
		// scoring credit would mean the scorer's Touched veto and the
		// simulator's terminal surface disagree about the same contact.
		if r.TerminalSurface == "parking_lot" && points > 0 {
			t.Errorf("%s %s: ended on a parking-lot fin but scored %d points; "+
				"touching a lot limitation voids all parking points (9.24.7)",
				which, id, points)
		}
	}

	// PassSideViolation and the sign list must agree -- a diagnostic that
	// filters on one and reports the other would silently disagree with itself.
	if r.PassSideViolation != (len(r.PassSideViolationSigns) > 0) {
		t.Errorf("%s %s: PassSideViolation=%v but %d violating signs listed",
			which, id, r.PassSideViolation, len(r.PassSideViolationSigns))
	}

	// TerminalSurface must name a real surface. It used to be hardcoded to
	// "outer_wall" for every contact, which made a fin touch and a wall
	// scrape indistinguishable to anything filtering on it.
	if !slices.Contains(knownContactSurfaces, r.TerminalSurface) {
		t.Errorf("%s %s: TerminalSurface = %q, want one of %v",
			which, id, r.TerminalSurface, knownContactSurfaces)
	}
	if r.Collided != (r.TerminalSurface != "none") {
		t.Errorf("%s %s: Collided=%v but TerminalSurface=%q",
			which, id, r.Collided, r.TerminalSurface)
	}
}

// knownContactSurfaces are the values of Python's ContactSurface StrEnum
// (src/simulation/track_model.py), which collision.ContactSurface.String()
// mirrors.
var knownContactSurfaces = []string{"none", "outer_wall", "inner_wall", "obstacle", "parking_lot"}

// assertObstaclesParity compares the fields the Open gate does not: the ones
// only an Obstacles scenario populates.
//
// ParkPoints is deliberately NOT compared. It has no counterpart in Python's
// SimResult -- the 15/7/0 scorer was ported standalone and never wired into
// the Python simulator -- so a comparison would assert against a field the
// oracle does not produce. assertObstaclesInvariants covers it instead.
func assertObstaclesParity(t *testing.T, id string, native, py scenario.Result) {
	t.Helper()

	if native.TerminalSurface != py.TerminalSurface {
		t.Errorf("%s: TerminalSurface mismatch native=%q python=%q",
			id, native.TerminalSurface, py.TerminalSurface)
	}
	if native.PassSideViolation != py.PassSideViolation {
		t.Errorf("%s: PassSideViolation mismatch native=%v python=%v",
			id, native.PassSideViolation, py.PassSideViolation)
	}
	// The sign INDICES matter, not just the count: passing the wrong sign on
	// the wrong side is a different bug from passing a different sign, and
	// the 2026-09-03 travel-relative rule fix turned on exactly which sign
	// each side applied to.
	if !slices.Equal(native.PassSideViolationSigns, py.PassSideViolationSigns) {
		t.Errorf("%s: PassSideViolationSigns mismatch native=%v python=%v",
			id, native.PassSideViolationSigns, py.PassSideViolationSigns)
	}
	if !boolPtrEqual(native.Parked, py.Parked) {
		t.Errorf("%s: Parked mismatch native=%s python=%s",
			id, formatBoolPtr(native.Parked), formatBoolPtr(py.Parked))
	}
	if native.ContactCount != py.ContactCount {
		t.Errorf("%s: ContactCount mismatch native=%d python=%d",
			id, native.ContactCount, py.ContactCount)
	}
	if math.Abs(native.DistanceM-py.DistanceM) > resultDistanceTolM {
		t.Errorf("%s: DistanceM mismatch native=%.3f python=%.3f (tol %.3f)",
			id, native.DistanceM, py.DistanceM, resultDistanceTolM)
	}
}

// resultDistanceTolM is the absolute tolerance on path length between the two
// runners, scaled from resultSimTimeTolS at the fastest rung of the shipped
// Open speed ladder (0.50 m/s): the same per-tick offset that tolerance
// allows, expressed as distance.
const resultDistanceTolM = resultSimTimeTolS * 0.50

// boolPtrEqual compares two optional booleans, treating nil (no parking lot
// in this scenario) as distinct from false (parking attempted and failed).
func boolPtrEqual(a, b *bool) bool {
	if a == nil || b == nil {
		return a == nil && b == nil
	}
	return *a == *b
}

func formatBoolPtr(v *bool) string {
	if v == nil {
		return "none"
	}
	return fmt.Sprintf("%v", *v)
}

// obstaclesPythonRunner builds the Python oracle runner, or nil when
// VTITAN_SIM_PYTHON is unset (the same skip policy as the Open gate).
func obstaclesPythonRunner(t *testing.T) *scenario.SubprocessRunner {
	t.Helper()

	python := os.Getenv("VTITAN_SIM_PYTHON")
	if python == "" {
		t.Log("VTITAN_SIM_PYTHON unset: skipping the Python oracle; " +
			"asserting native-runner invariants only")
		return nil
	}

	var baseArgs []string
	if raw := os.Getenv("VTITAN_SIM_BASE_ARGS"); raw != "" {
		baseArgs = splitComma(raw)
	}
	robotDir := robotRepoDir(t)
	scriptPath := filepath.Join(robotDir, "scripts", "sim", "run_scenario.py")
	if _, err := os.Stat(scriptPath); err != nil {
		t.Fatalf("python sim script missing at %s: %v", scriptPath, err)
	}

	runner, err := scenario.NewSubprocessRunner(scenario.Config{
		Command:    python,
		BaseArgs:   baseArgs,
		ScriptPath: scriptPath,
		WorkDir:    robotDir,
		ExtraArgs:  []string{"--max-steps", fmt.Sprint(obstaclesMaxSteps)},
	})
	if err != nil {
		t.Fatalf("NewSubprocessRunner: %v", err)
	}
	return runner
}

// loadObstaclesCorpus resolves and loads the Obstacles scenario corpus,
// skipping the test when it is absent.
//
// The corpus lives under platform/robot/.corpus/, which is GITIGNORED: a
// fresh checkout does not have it, so a missing corpus is a skip rather than
// a failure. VTITAN_OBSTACLES_CORPUS overrides the location.
func loadObstaclesCorpus(t *testing.T) []corpus.Scenario {
	t.Helper()

	dir := os.Getenv("VTITAN_OBSTACLES_CORPUS")
	if dir == "" {
		dir = filepath.Join(robotRepoDir(t), ".corpus", "obstacles", "scenarios")
	}
	if _, err := os.Stat(dir); err != nil {
		t.Skipf("obstacles corpus not present at %s (it is gitignored); "+
			"set VTITAN_OBSTACLES_CORPUS to run this gate: %v", dir, err)
	}

	scenarios, err := corpus.Load(dir)
	if err != nil {
		t.Fatalf("corpus.Load(%s): %v", dir, err)
	}
	if len(scenarios) == 0 {
		t.Fatalf("no Obstacles scenarios in %s", dir)
	}
	return scenarios
}

// obstaclesSampleSize returns how many scenarios the per-scenario parity
// subtest should cover: VTITAN_OBSTACLES_SAMPLE when set (0 meaning all),
// else paritySampleSize.
func obstaclesSampleSize(t *testing.T) int {
	t.Helper()

	raw := os.Getenv("VTITAN_OBSTACLES_SAMPLE")
	if raw == "" {
		return paritySampleSize
	}
	var size int
	if _, err := fmt.Sscanf(raw, "%d", &size); err != nil || size < 0 {
		t.Fatalf("VTITAN_OBSTACLES_SAMPLE = %q, want a non-negative integer", raw)
	}
	return size
}
