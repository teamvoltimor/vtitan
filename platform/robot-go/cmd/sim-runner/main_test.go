package main

import (
	"bytes"
	"errors"
	"log/slog"
	"strings"
	"testing"

	"github.com/spf13/cobra"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/corpus"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/scenario"
)

// TestRootCmd_RequiredFlags exercises flag parsing through the real cobra
// command (newRootCmd), not a standalone parser — cobra's MarkFlagRequired
// is what enforces -corpus/-script/-workdir, so the test needs to go
// through the command to actually exercise that enforcement. RunE is
// swapped for a no-op that only reports success, since these cases are
// about flag validation, not the orchestrator itself running end to end.
func TestRootCmd_RequiredFlags(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name    string
		args    []string
		wantErr bool
	}{
		{
			name: "all required flags present",
			args: []string{"--corpus", "/tmp/corpus", "--script", "/tmp/run_scenario.py", "--workdir", "/tmp/robot"},
		},
		{name: "missing corpus", args: []string{"--script", "x.py", "--workdir", "."}, wantErr: true},
		{name: "missing script", args: []string{"--corpus", "x", "--workdir", "."}, wantErr: true},
		{name: "missing workdir", args: []string{"--corpus", "x", "--script", "x.py"}, wantErr: true},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			var cfg cliConfig
			logger := slog.New(slog.DiscardHandler)
			cmd := newRootCmd(&cfg, logger, &bytes.Buffer{})
			cmd.RunE = func(*cobra.Command, []string) error { return nil }
			cmd.SetArgs(tt.args)

			err := cmd.Execute()
			if tt.wantErr && err == nil {
				t.Fatal("Execute() error = nil, want an error")
			}
			if !tt.wantErr && err != nil {
				t.Fatalf("Execute() error = %v, want nil", err)
			}
		})
	}
}

func TestSplitCSV(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name string
		raw  string
		want []string
	}{
		{name: "empty", raw: "", want: nil},
		{name: "single", raw: "run", want: []string{"run"}},
		{name: "multiple", raw: "run,-e,dev,python", want: []string{"run", "-e", "dev", "python"}},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			got := splitCSV(tt.raw)
			if len(got) != len(tt.want) {
				t.Fatalf("splitCSV(%q) = %v, want %v", tt.raw, got, tt.want)
			}
			for i := range got {
				if got[i] != tt.want[i] {
					t.Fatalf("splitCSV(%q) = %v, want %v", tt.raw, got, tt.want)
				}
			}
		})
	}
}

func TestCountErrored(t *testing.T) {
	t.Parallel()

	report := scenario.Report{
		Results: []scenario.ScenarioResult{
			{Scenario: corpus.Scenario{ID: "a"}, Err: errors.New("boom")},
			{Scenario: corpus.Scenario{ID: "b"}},
			{Scenario: corpus.Scenario{ID: "c"}, Err: errors.New("boom again")},
		},
	}

	if got := countErrored(report); got != 2 {
		t.Errorf("countErrored() = %d, want 2", got)
	}
}

func TestPrintReport_TextIncludesSummary(t *testing.T) {
	t.Parallel()

	report := scenario.Report{
		Results: []scenario.ScenarioResult{
			{
				Scenario: corpus.Scenario{ID: "scenario_0000"},
				Result:   scenario.Result{Success: true, LapsCompleted: 3, TargetLaps: 3},
			},
			{
				Scenario: corpus.Scenario{ID: "scenario_0001"},
				Result:   scenario.Result{Collided: true, LapsCompleted: 1, TargetLaps: 3},
			},
		},
	}

	var buf bytes.Buffer
	printReport(&buf, report, false)
	out := buf.String()

	for _, want := range []string{"scenario_0000", "scenario_0001", "total", "succeeded", "collided"} {
		if !strings.Contains(out, want) {
			t.Errorf("printReport() output missing %q; got:\n%s", want, out)
		}
	}
}

func TestPrintReport_JSON(t *testing.T) {
	t.Parallel()

	report := scenario.Report{
		Results: []scenario.ScenarioResult{
			{Scenario: corpus.Scenario{ID: "scenario_0000"}, Result: scenario.Result{Success: true}},
		},
	}

	var buf bytes.Buffer
	printReport(&buf, report, true)

	if !strings.Contains(buf.String(), `"ID": "scenario_0000"`) {
		t.Errorf("printReport(json=true) output missing scenario ID; got:\n%s", buf.String())
	}
}
