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
// command (newRootCmd), not a standalone parser, so the cases go through the
// same binding validate then reads. RunE is swapped for validate alone,
// since these cases are about flag validation and not about the orchestrator
// running end to end.
//
// Which flags are required is conditional: --corpus is replaced by
// --open-space, and --script/--workdir address a subprocess that
// --runner native never starts. That is why none of them can be a cobra
// MarkFlagRequired.
func TestRootCmd_RequiredFlags(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name    string
		args    []string
		wantErr string
	}{
		{
			name: "python runner with a corpus and its script",
			args: []string{
				"--corpus", "/tmp/corpus",
				"--script", "/tmp/run_scenario.py",
				"--workdir", "/tmp/robot",
			},
		},
		{
			name:    "no corpus and no open space",
			args:    []string{"--script", "x.py", "--workdir", "."},
			wantErr: "one of --corpus or --open-space",
		},
		{
			name:    "python runner without a script",
			args:    []string{"--corpus", "x", "--workdir", "."},
			wantErr: "needs --script and --workdir",
		},
		{
			name:    "python runner without a workdir",
			args:    []string{"--corpus", "x", "--script", "x.py"},
			wantErr: "needs --script and --workdir",
		},
		{
			name: "native runner needs neither script nor workdir",
			args: []string{"--corpus", "x", "--runner", "native"},
		},
		{
			name: "generated open space needs no corpus path",
			args: []string{"--open-space", "full", "--runner", "native"},
		},
		{
			name:    "a corpus and a generated space are mutually exclusive",
			args:    []string{"--corpus", "x", "--open-space", "full", "--runner", "native"},
			wantErr: "mutually exclusive",
		},
		{
			name:    "unknown open space",
			args:    []string{"--open-space", "open999", "--runner", "native"},
			wantErr: "unknown --open-space",
		},
		{
			name:    "unknown runner",
			args:    []string{"--corpus", "x", "--runner", "rust"},
			wantErr: "unknown --runner",
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			var cfg cliConfig
			logger := slog.New(slog.DiscardHandler)
			cmd := newRootCmd(&cfg, logger, &bytes.Buffer{})
			cmd.RunE = func(*cobra.Command, []string) error { return validate(cfg) }
			cmd.SetArgs(tt.args)

			err := cmd.Execute()
			if tt.wantErr == "" {
				if err != nil {
					t.Fatalf("Execute() error = %v, want nil", err)
				}
				return
			}
			if err == nil {
				t.Fatalf("Execute() error = nil, want one mentioning %q", tt.wantErr)
			}
			if !strings.Contains(err.Error(), tt.wantErr) {
				t.Errorf("Execute() error = %q, want it to mention %q", err, tt.wantErr)
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
			{
				Scenario: corpus.Scenario{ID: "scenario_0000"},
				Result:   scenario.Result{Success: true},
			},
		},
	}

	var buf bytes.Buffer
	printReport(&buf, report, true)

	if !strings.Contains(buf.String(), `"ID": "scenario_0000"`) {
		t.Errorf("printReport(json=true) output missing scenario ID; got:\n%s", buf.String())
	}
}
