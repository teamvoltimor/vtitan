package scenario_test

// These tests exercise SubprocessRunner's process-boundary plumbing (arg
// construction, env, exit-code handling, stdout parsing) against the Go
// test binary itself re-executed as a fake "run_scenario.py" — the standard
// os/exec self-exec test trick — rather than against the real Python
// script. That keeps this test fast and independent of a pixi/Python
// environment being present. A real end-to-end check against
// scripts/sim/run_scenario.py belongs in the //go:build integration test
// instead (subprocess_runner_integration_test.go), gated the same way
// go-architect §9 gates Docker-backed integration tests.

import (
	"fmt"
	"os"
	"strings"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/sim/corpus"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/scenario"
)

// TestHelperProcess is not a real test — it's re-executed as a subprocess by
// the tests below (via os.Args[0] + -test.run=TestHelperProcess) to stand
// in for scripts/sim/run_scenario.py. It only does anything when
// GO_WANT_HELPER_PROCESS=1 is set in its environment; otherwise `go test`
// running it normally is a no-op.
func TestHelperProcess(t *testing.T) { //nolint:paralleltest // re-executed as a subprocess, not a real test case
	if os.Getenv("GO_WANT_HELPER_PROCESS") != "1" {
		return
	}
	if os.Getenv("GO_HELPER_FAIL") == "1" {
		fmt.Fprintln(os.Stderr, "Traceback (most recent call last):\nValueError: bad scenario metadata")
		os.Exit(1)
	}
	fmt.Fprintln(os.Stdout, `{"scenario":"scenario_0000_metadata.json","target_laps":3,"laps_completed":3,`+
		`"collided":false,"timed_out":false,"stuck":false,"pass_side_violation":false,`+
		`"pass_side_violation_signs":[],"parked":true,"success":true,"over_time":false,`+
		`"steps":2600,"sim_time_s":130.0,"distance_m":11.4,"max_speed_mps":0.156,`+
		`"avg_speed_mps":0.09,"min_lidar_range_m":0.18,"collision_xy":null,`+
		`"final_pose":[1.5,0.25,0.0],"contact_count":0,"contact_time_s":0.0,`+
		`"terminal_surface":"none","lap_step_indices":[820,1690,2600]}`)
	os.Exit(0)
}

func helperRunnerConfig(t *testing.T) scenario.Config {
	t.Helper()
	return scenario.Config{
		Command:    os.Args[0],
		BaseArgs:   []string{"-test.run=^TestHelperProcess$", "--"},
		ScriptPath: "run_scenario.py",
		WorkDir:    t.TempDir(),
	}
}

func TestSubprocessRunner_Run_ParsesStdout(t *testing.T) {
	t.Setenv("GO_WANT_HELPER_PROCESS", "1")

	runner, err := scenario.NewSubprocessRunner(helperRunnerConfig(t))
	if err != nil {
		t.Fatalf("NewSubprocessRunner() error = %v, want nil", err)
	}

	sc := corpus.Scenario{ID: "scenario_0000", MetadataPath: "scenario_0000_metadata.json"}
	result, err := runner.Run(t.Context(), sc)
	if err != nil {
		t.Fatalf("Run() error = %v, want nil", err)
	}
	if !result.Success || result.LapsCompleted != 3 || result.TerminalSurface != "none" {
		t.Errorf("Run() result = %+v, want a successful 3-lap result", result)
	}
}

func TestSubprocessRunner_Run_ReportsNonZeroExitWithStderr(t *testing.T) {
	t.Setenv("GO_WANT_HELPER_PROCESS", "1")
	t.Setenv("GO_HELPER_FAIL", "1")

	runner, err := scenario.NewSubprocessRunner(helperRunnerConfig(t))
	if err != nil {
		t.Fatalf("NewSubprocessRunner() error = %v, want nil", err)
	}

	sc := corpus.Scenario{ID: "scenario_0001", MetadataPath: "scenario_0001_metadata.json"}
	_, err = runner.Run(t.Context(), sc)
	if err == nil {
		t.Fatal("Run() error = nil, want an error for a non-zero exit")
	}
	if !strings.Contains(err.Error(), "bad scenario metadata") {
		t.Errorf("Run() error = %v, want it to include the subprocess's stderr", err)
	}
}

func TestNewSubprocessRunner_RejectsMissingConfig(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name string
		cfg  scenario.Config
	}{
		{name: "missing command", cfg: scenario.Config{ScriptPath: "x.py", WorkDir: "."}},
		{name: "missing script path", cfg: scenario.Config{Command: "python3", WorkDir: "."}},
		{name: "missing work dir", cfg: scenario.Config{Command: "python3", ScriptPath: "x.py"}},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()
			if _, err := scenario.NewSubprocessRunner(tt.cfg); err == nil {
				t.Fatalf("NewSubprocessRunner(%+v) error = nil, want an error", tt.cfg)
			}
		})
	}
}
