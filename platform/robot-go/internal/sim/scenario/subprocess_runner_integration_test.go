//go:build integration

package scenario_test

// Real end-to-end check against the actual scripts/sim/run_scenario.py and a
// real Python interpreter/pixi environment, gated behind the "integration"
// build tag (go-architect §9's convention for tests needing an external
// dependency `go test ./...` shouldn't require by default — here that
// dependency is a Python/pixi environment rather than Docker).
//
// Run with:
//
//	go test -tags=integration ./internal/sim/scenario/... -run TestSubprocessRunner_Integration
//
// Set VTITAN_SIM_PYTHON to the interpreter to invoke (e.g. "python3", or
// "pixi" together with VTITAN_SIM_BASE_ARGS="run,-e,dev,python" for the
// project's managed pixi environment). Skipped if unset, so CI/dev machines
// without the sim environment provisioned don't fail this test by omission.

import (
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/corpus"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/scenario"
)

func TestSubprocessRunner_Integration_RealPythonSimulator(t *testing.T) {
	t.Parallel()

	python := os.Getenv("VTITAN_SIM_PYTHON")
	if python == "" {
		t.Skip("VTITAN_SIM_PYTHON not set; skipping real-Python integration test")
	}
	var baseArgs []string
	if raw := os.Getenv("VTITAN_SIM_BASE_ARGS"); raw != "" {
		baseArgs = strings.Split(raw, ",")
	}

	robotDir := robotRepoDir(t)
	scriptPath := filepath.Join(robotDir, "scripts", "sim", "run_scenario.py")
	fixture := filepath.Join(robotDir, "tests", "fixtures", "scenarios", "obstacles", "scenario_0000_metadata.json")
	if _, err := os.Stat(fixture); err != nil {
		t.Fatalf("fixture missing, is the worktree checked out correctly? %v", err)
	}

	runner, err := scenario.NewSubprocessRunner(scenario.Config{
		Command:    python,
		BaseArgs:   baseArgs,
		ScriptPath: scriptPath,
		WorkDir:    robotDir,
		ExtraArgs:  []string{"--max-steps", "200"},
	})
	if err != nil {
		t.Fatalf("NewSubprocessRunner() error = %v, want nil", err)
	}

	sc := corpus.Scenario{ID: "scenario_0000", MetadataPath: fixture}
	result, err := runner.Run(t.Context(), sc)
	if err != nil {
		t.Fatalf("Run() error = %v, want nil", err)
	}
	if result.TargetLaps == 0 {
		t.Errorf("Run() result = %+v, want a populated TargetLaps from the real simulator", result)
	}
}

// robotRepoDir finds platform/robot, sibling to this module's platform/robot-go.
func robotRepoDir(t *testing.T) string {
	t.Helper()
	_, thisFile, _, ok := runtime.Caller(0)
	if !ok {
		t.Fatal("runtime.Caller(0) failed")
	}
	// this file: platform/robot-go/internal/sim/scenario/subprocess_runner_integration_test.go
	// dir(thisFile) = .../platform/robot-go/internal/sim/scenario ; four levels up is .../platform
	platformDir := filepath.Join(filepath.Dir(thisFile), "..", "..", "..", "..")
	return filepath.Join(platformDir, "robot")
}
