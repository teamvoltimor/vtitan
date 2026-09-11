package scenario_test

import (
	"context"
	"errors"
	"fmt"
	"sync/atomic"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/internal/sim/corpus"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/scenario"
)

// fakeOutcome is one scripted response for fakeRunner, keyed by scenario ID.
// A slice of named structs rather than a map[string]fakeOutcome, per project
// convention: the outcome for scenario N is looked up by a short linear
// scan below, which is clearer for a handful of table rows than a map would
// be and keeps the table itself readable top-to-bottom.
type fakeOutcome struct {
	scenarioID string
	result     scenario.Result
	err        error
	delay      time.Duration
}

// fakeRunner implements scenario.Runner without ever shelling out — it
// looks up a scripted fakeOutcome per scenario ID and tracks how many
// scenarios were in flight at once, so a test can assert the orchestrator's
// concurrency cap is actually honored.
type fakeRunner struct {
	outcomes []fakeOutcome

	maxInFlight atomic.Int64
	inFlight    atomic.Int64
}

func (r *fakeRunner) Run(ctx context.Context, sc corpus.Scenario) (scenario.Result, error) {
	inFlight := r.inFlight.Add(1)
	defer r.inFlight.Add(-1)
	for {
		observed := r.maxInFlight.Load()
		if inFlight <= observed || r.maxInFlight.CompareAndSwap(observed, inFlight) {
			break
		}
	}

	for i := range r.outcomes {
		outcome := &r.outcomes[i]
		if outcome.scenarioID != sc.ID {
			continue
		}
		if outcome.delay > 0 {
			select {
			case <-time.After(outcome.delay):
			case <-ctx.Done():
				return scenario.Result{}, fmt.Errorf("fakeRunner: %w", ctx.Err())
			}
		}
		return outcome.result, outcome.err
	}
	return scenario.Result{}, fmt.Errorf("fakeRunner: no scripted outcome for scenario %s", sc.ID)
}

func TestOrchestrator_Run_AggregatesInInputOrder(t *testing.T) {
	t.Parallel()

	runner := &fakeRunner{
		outcomes: []fakeOutcome{
			{
				scenarioID: "scenario_0002",
				result:     scenario.Result{Scenario: "scenario_0002", Success: true},
				delay:      15 * time.Millisecond,
			},
			{
				scenarioID: "scenario_0000",
				result:     scenario.Result{Scenario: "scenario_0000", Success: true},
				delay:      5 * time.Millisecond,
			},
			{scenarioID: "scenario_0001", result: scenario.Result{Scenario: "scenario_0001", Collided: true}},
		},
	}
	scenarios := []corpus.Scenario{
		{ID: "scenario_0000", MetadataPath: "scenario_0000_metadata.json"},
		{ID: "scenario_0001", MetadataPath: "scenario_0001_metadata.json"},
		{ID: "scenario_0002", MetadataPath: "scenario_0002_metadata.json"},
	}

	orch, err := scenario.NewOrchestrator(runner, scenario.OrchestratorConfig{Concurrency: 3})
	if err != nil {
		t.Fatalf("NewOrchestrator() error = %v, want nil", err)
	}

	report, err := orch.Run(t.Context(), scenarios)
	if err != nil {
		t.Fatalf("Run() error = %v, want nil", err)
	}
	if len(report.Results) != len(scenarios) {
		t.Fatalf("Run() returned %d results, want %d", len(report.Results), len(scenarios))
	}
	for i, want := range scenarios {
		if got := report.Results[i].Scenario.ID; got != want.ID {
			t.Errorf("Results[%d].Scenario.ID = %q, want %q (order not preserved)", i, got, want.ID)
		}
	}
}

func TestOrchestrator_Run_HonorsConcurrencyLimit(t *testing.T) {
	t.Parallel()

	const numScenarios = 6
	const concurrency = 2

	outcomes := make([]fakeOutcome, 0, numScenarios)
	scenarios := make([]corpus.Scenario, 0, numScenarios)
	for i := range numScenarios {
		id := fmt.Sprintf("scenario_%04d", i)
		scenarios = append(scenarios, corpus.Scenario{ID: id, MetadataPath: id + "_metadata.json"})
		outcomes = append(outcomes, fakeOutcome{
			scenarioID: id,
			result:     scenario.Result{Scenario: id, Success: true},
			delay:      10 * time.Millisecond,
		})
	}
	runner := &fakeRunner{outcomes: outcomes}

	orch, err := scenario.NewOrchestrator(runner, scenario.OrchestratorConfig{Concurrency: concurrency})
	if err != nil {
		t.Fatalf("NewOrchestrator() error = %v, want nil", err)
	}
	if _, runErr := orch.Run(t.Context(), scenarios); runErr != nil {
		t.Fatalf("Run() error = %v, want nil", runErr)
	}

	if got := runner.maxInFlight.Load(); got > concurrency {
		t.Errorf("max concurrent Run() calls = %d, want <= %d", got, concurrency)
	}
}

func TestOrchestrator_Run_PerScenarioErrorDoesNotAbortBatch(t *testing.T) {
	t.Parallel()

	boom := errors.New("boom")
	runner := &fakeRunner{
		outcomes: []fakeOutcome{
			{scenarioID: "scenario_0000", err: boom},
			{scenarioID: "scenario_0001", result: scenario.Result{Scenario: "scenario_0001", Success: true}},
		},
	}
	scenarios := []corpus.Scenario{
		{ID: "scenario_0000", MetadataPath: "scenario_0000_metadata.json"},
		{ID: "scenario_0001", MetadataPath: "scenario_0001_metadata.json"},
	}

	orch, err := scenario.NewOrchestrator(runner, scenario.OrchestratorConfig{})
	if err != nil {
		t.Fatalf("NewOrchestrator() error = %v, want nil", err)
	}
	report, err := orch.Run(t.Context(), scenarios)
	if err != nil {
		t.Fatalf("Run() error = %v, want nil (per-scenario errors must not abort the batch)", err)
	}
	if !errors.Is(report.Results[0].Err, boom) {
		t.Errorf("Results[0].Err = %v, want %v", report.Results[0].Err, boom)
	}
	if !report.Results[1].Result.Success {
		t.Errorf("Results[1].Result.Success = false, want true")
	}

	summary := report.Summarize()
	wantCounts := []scenario.OutcomeCount{
		{Name: "total", Count: 2},
		{Name: "errored", Count: 1},
		{Name: "succeeded", Count: 1},
		{Name: "collided", Count: 0},
		{Name: "timed_out", Count: 0},
		{Name: "stuck", Count: 0},
		{Name: "pass_side_violation", Count: 0},
	}
	if len(summary.Counts) != len(wantCounts) {
		t.Fatalf("Summarize() returned %d counts, want %d", len(summary.Counts), len(wantCounts))
	}
	for i, want := range wantCounts {
		if summary.Counts[i] != want {
			t.Errorf("Summarize().Counts[%d] = %+v, want %+v", i, summary.Counts[i], want)
		}
	}
}

func TestOrchestrator_Run_ContextCancellationStopsEarly(t *testing.T) {
	t.Parallel()

	outcomes := make([]fakeOutcome, 0, 4)
	scenarios := make([]corpus.Scenario, 0, 4)
	for i := range 4 {
		id := fmt.Sprintf("scenario_%04d", i)
		scenarios = append(scenarios, corpus.Scenario{ID: id, MetadataPath: id + "_metadata.json"})
		outcomes = append(outcomes, fakeOutcome{scenarioID: id, delay: time.Second})
	}
	runner := &fakeRunner{outcomes: outcomes}

	orch, err := scenario.NewOrchestrator(runner, scenario.OrchestratorConfig{Concurrency: 4})
	if err != nil {
		t.Fatalf("NewOrchestrator() error = %v, want nil", err)
	}

	ctx, cancel := context.WithTimeout(t.Context(), 20*time.Millisecond)
	defer cancel()

	if _, runErr := orch.Run(ctx, scenarios); runErr == nil {
		t.Fatal("Run() error = nil, want a context-deadline error")
	}
}

func TestNewOrchestrator_RejectsInvalidConfig(t *testing.T) {
	t.Parallel()

	if _, err := scenario.NewOrchestrator(&fakeRunner{}, scenario.OrchestratorConfig{Concurrency: -1}); err == nil {
		t.Fatal("NewOrchestrator() error = nil, want an error for negative concurrency")
	}
}
