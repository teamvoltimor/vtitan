//go:build integration

package scenario_test

// NativeRunner vs SubprocessRunner parity gate. Runs a small corpus of Open
// Challenge scenario metadata files through BOTH the Go-native runner and the
// Python subprocess runner (the frozen parity oracle), then asserts their
// scenario.Result fields match within tolerance.
//
// Gating:
//   - The whole file is behind the "integration" build tag so `go test ./...`
//     (no tag) never compiles or runs it; CI without a Python/pixi env stays
//     green (see go-architect §9).
//   - NativeRunner ALWAYS runs (it needs no external interpreter) so the
//     native-sim path is still exercised under the integration tag.
//   - SubprocessRunner (Python) only runs when VTITAN_SIM_PYTHON is set; if it
//     is unset the Python side is skipped and the test only confirms the
//     native runner produces a self-consistent, populated Result over the
//     corpus (not a no-op). This matches the existing
//     subprocess_runner_integration_test.go skip policy.
//
// Run:
//
//	go test -tags=integration ./internal/sim/scenario/... -run TestNativeRunner_ParityVsPython
//	VTITAN_SIM_PYTHON=python3 go test -tags=integration ./internal/sim/scenario/... -run TestNativeRunner_ParityVsPython

import (
	"math"
	"os"
	"path/filepath"
	"runtime"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/corpus"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/scenario"
)

// resultSimTimeTolS is the absolute tolerance on sim_time_s between the two
// runners. The native runner uses hardcoded parity defaults for the
// kinematics/no-progress policy (native_runner.go) and a ground-truth LIDAR,
// so a small wall-clock-step offset vs the Python sim is expected before those
// constants are sourced from a shared profile.
const resultSimTimeTolS = 0.2

func TestNativeRunner_ParityVsPython(t *testing.T) {
	t.Parallel()

	corpusDir := openCorpusDir(t)

	scenarios, err := corpus.Load(corpusDir)
	if err != nil {
		t.Fatalf("corpus.Load(%s): %v", corpusDir, err)
	}
	if len(scenarios) == 0 {
		t.Fatalf("no Open scenarios in %s", corpusDir)
	}

	// Bound the corpus so the integration test stays fast; the full Open set
	// is small (10) but keep a sane cap for ad-hoc local runs.
	const maxScenarios = 5
	if len(scenarios) > maxScenarios {
		scenarios = scenarios[:maxScenarios]
	}

	native := scenario.NewNativeRunner(scenario.NativeRunnerConfig{MaxSteps: 4000})

	python := os.Getenv("VTITAN_SIM_PYTHON")
	var pyRunner *scenario.SubprocessRunner
	if python != "" {
		var baseArgs []string
		if raw := os.Getenv("VTITAN_SIM_BASE_ARGS"); raw != "" {
			baseArgs = splitComma(raw)
		}
		robotDir := robotRepoDir(t)
		scriptPath := filepath.Join(robotDir, "scripts", "sim", "run_scenario.py")
		if _, statErr := os.Stat(scriptPath); statErr != nil {
			t.Fatalf("python sim script missing at %s: %v", scriptPath, statErr)
		}
		pyRunner, err = scenario.NewSubprocessRunner(scenario.Config{
			Command:    python,
			BaseArgs:   baseArgs,
			ScriptPath: scriptPath,
			WorkDir:    robotDir,
			ExtraArgs:  []string{"--max-steps", "4000"},
		})
		if err != nil {
			t.Fatalf("NewSubprocessRunner: %v", err)
		}
	} else {
		t.Log("VTITAN_SIM_PYTHON unset: skipping Python side; asserting native runner output only")
	}

	for _, sc := range scenarios {
		sc := sc
		t.Run(sc.ID, func(t *testing.T) {
			t.Parallel()

			nativeRes, nerr := native.Run(t.Context(), sc)
			if nerr != nil {
				t.Fatalf("NativeRunner.Run(%s): %v", sc.ID, nerr)
			}
			assertResultPopulated(t, "native", sc.ID, nativeRes)

			if pyRunner == nil {
				return
			}

			pyRes, perr := pyRunner.Run(t.Context(), sc)
			if perr != nil {
				t.Fatalf("SubprocessRunner.Run(%s): %v", sc.ID, perr)
			}
			assertResultPopulated(t, "python", sc.ID, pyRes)

			assertResultParity(t, sc.ID, nativeRes, pyRes)
		})
	}
}

func assertResultPopulated(t *testing.T, which, id string, r scenario.Result) {
	t.Helper()
	if r.TargetLaps == 0 {
		t.Errorf("%s %s: TargetLaps == 0 (unpopulated Result)", which, id)
	}
	if r.Steps == 0 {
		t.Errorf("%s %s: Steps == 0 (runner produced no ticks)", which, id)
	}
	if !r.Collided && !r.TimedOut && !r.Stuck && r.LapsCompleted < r.TargetLaps {
		// A clean in-progress result is fine, but SimTimeS must advance.
		if r.SimTimeS <= 0 {
			t.Errorf("%s %s: SimTimeS == 0 for a non-terminal/non-lap result", which, id)
		}
	}
}

func assertResultParity(t *testing.T, id string, native, py scenario.Result) {
	t.Helper()
	if native.Collided != py.Collided {
		t.Errorf("%s: Collided mismatch native=%v python=%v", id, native.Collided, py.Collided)
	}
	if native.TimedOut != py.TimedOut {
		t.Errorf("%s: TimedOut mismatch native=%v python=%v", id, native.TimedOut, py.TimedOut)
	}
	if native.Stuck != py.Stuck {
		t.Errorf("%s: Stuck mismatch native=%v python=%v", id, native.Stuck, py.Stuck)
	}
	if native.LapsCompleted != py.LapsCompleted {
		t.Errorf("%s: LapsCompleted mismatch native=%d python=%d", id, native.LapsCompleted, py.LapsCompleted)
	}
	if math.Abs(native.SimTimeS-py.SimTimeS) > resultSimTimeTolS {
		t.Errorf("%s: SimTimeS mismatch native=%.3f python=%.3f (tol %.3f)",
			id, native.SimTimeS, py.SimTimeS, resultSimTimeTolS)
	}
}

// openCorpusDir resolves a directory of Open Challenge scenario metadata
// files, preferring VTITAN_OPEN_CORPUS and falling back to the gazebo
// generator's training_data/open/scenarios (checked out alongside this module).
func openCorpusDir(t *testing.T) string {
	t.Helper()
	if override := os.Getenv("VTITAN_OPEN_CORPUS"); override != "" {
		return override
	}
	_, thisFile, _, ok := runtime.Caller(0)
	if !ok {
		t.Fatal("runtime.Caller(0) failed")
	}
	// this file: platform/robot-go/internal/sim/scenario/native_runner_integration_test.go
	// four levels up -> platform ; join gazebo generator open scenarios.
	platformDir := filepath.Join(filepath.Dir(thisFile), "..", "..", "..", "..")
	return filepath.Join(platformDir, "gazebo", "generator", "training_data", "open", "scenarios")
}

func splitComma(s string) []string {
	var out []string
	cur := ""
	for _, r := range s {
		if r == ',' {
			out = append(out, cur)
			cur = ""
			continue
		}
		cur += string(r)
	}
	out = append(out, cur)
	return out
}

var _ = filepath.Join
