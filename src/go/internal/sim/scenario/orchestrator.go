package scenario

import (
	"cmp"
	"context"
	"fmt"
	"runtime"
	"slices"

	"github.com/go-playground/validator/v10"
	"golang.org/x/sync/errgroup"

	"github.com/teamvoltimor/vtitan/src/go/internal/sim/corpus"
)

// OrchestratorConfig configures an Orchestrator's concurrency limit.
type OrchestratorConfig struct {
	// Concurrency caps how many scenarios run at once. Left at 0, it
	// defaults to runtime.NumCPU() — sim runs are CPU-bound (see the
	// "Simulation" section of docs/internal/plans/go-migration-plan.md),
	// so the host's core count is the sane default, but a deployment
	// running the orchestrator alongside other CPU-heavy work (e.g. CI
	// sharing a runner) may reasonably want to override it, so it is
	// never hardcoded past this default.
	Concurrency int `validate:"gte=0"`
}

// ScenarioResult pairs one Scenario with its outcome. Err is set when the
// Runner itself failed to produce a Result (a subprocess crash, a timeout,
// malformed output) — distinct from Result.Collided/TimedOut/Stuck/etc.,
// which describe a Result the Runner DID successfully produce, just one
// describing a failed run. Conflating the two would make a harness bug look
// like a scored failure or vice versa.
type ScenarioResult struct {
	Scenario corpus.Scenario
	Result   Result
	Err      error
}

// Report is the aggregated outcome of running a corpus. Results preserves
// the corpus's own input order (Orchestrator.Run reorders channel arrivals
// back into that order) rather than whatever order goroutines happened to
// finish in, so two runs of the same corpus produce comparably-ordered
// reports.
type Report struct {
	Results []ScenarioResult
}

// Summary reduces a Report to per-outcome counts. A slice of named counters
// rather than a map, per project convention: the outcome set is fixed and
// known at compile time, so a typed slice catches a typo in an outcome name
// that a map key would silently swallow.
type Summary struct {
	Counts []OutcomeCount
}

// OutcomeCount is one named tally in a Summary.
type OutcomeCount struct {
	Name  string
	Count int
}

// Orchestrator runs a corpus of scenarios concurrently through a Runner,
// bounded by a semaphore-style worker pool, and aggregates results via a
// channel read by a single collector goroutine rather than a shared map
// guarded by a mutex — matching the sim-orchestrator pattern in
// docs/internal/plans/go-migration-plan.md's "Workers / concurrency model".
type Orchestrator struct {
	runner      Runner
	concurrency int
}

// indexedResult carries a ScenarioResult's position in the input corpus
// through the results channel, so the collector can restore input order
// once every producer goroutine has finished.
type indexedResult struct {
	index int
	ScenarioResult
}

// summaryOutcomeIndex names each fixed slot in Summary.Counts (after the
// leading "total" slot at index 0), so Summarize below never indexes
// counts[] with a bare literal.
type summaryOutcomeIndex int

const (
	idxTotal summaryOutcomeIndex = iota
	idxErrored
	idxSucceeded
	idxCollided
	idxTimedOut
	idxStuck
	idxPassSideViolation
)

// Outcome names used in Summary.Counts, in the order they're reported.
const (
	outcomeTotal             = "total"
	outcomeErrored           = "errored"
	outcomeSucceeded         = "succeeded"
	outcomeCollided          = "collided"
	outcomeTimedOut          = "timed_out"
	outcomeStuck             = "stuck"
	outcomePassSideViolation = "pass_side_violation"
)

// Summarize tallies Results by outcome. A ScenarioResult with Err set counts
// only toward "errored" and "total" — its Result is a zero value and none of
// the other outcome fields are meaningful for it.
func (report Report) Summarize() Summary {
	counts := []OutcomeCount{
		idxTotal:             {Name: outcomeTotal, Count: len(report.Results)},
		idxErrored:           {Name: outcomeErrored, Count: 0},
		idxSucceeded:         {Name: outcomeSucceeded, Count: 0},
		idxCollided:          {Name: outcomeCollided, Count: 0},
		idxTimedOut:          {Name: outcomeTimedOut, Count: 0},
		idxStuck:             {Name: outcomeStuck, Count: 0},
		idxPassSideViolation: {Name: outcomePassSideViolation, Count: 0},
	}

	for i := range report.Results {
		sr := &report.Results[i]
		if sr.Err != nil {
			counts[idxErrored].Count++
			continue
		}
		if sr.Result.Success {
			counts[idxSucceeded].Count++
		}
		if sr.Result.Collided {
			counts[idxCollided].Count++
		}
		if sr.Result.TimedOut {
			counts[idxTimedOut].Count++
		}
		if sr.Result.Stuck {
			counts[idxStuck].Count++
		}
		if sr.Result.PassSideViolation {
			counts[idxPassSideViolation].Count++
		}
	}
	return Summary{Counts: counts}
}

// NewOrchestrator validates cfg and returns an Orchestrator driving runner.
func NewOrchestrator(runner Runner, cfg OrchestratorConfig) (*Orchestrator, error) {
	if err := validator.New().Struct(cfg); err != nil {
		return nil, fmt.Errorf("scenario: invalid orchestrator config: %w", err)
	}
	concurrency := cfg.Concurrency
	if concurrency == 0 {
		concurrency = runtime.NumCPU()
	}
	return &Orchestrator{runner: runner, concurrency: concurrency}, nil
}

// Run runs every scenario in scenarios through the Orchestrator's Runner,
// at most Concurrency at a time, and returns the aggregated Report.
//
// A per-scenario Runner error is captured on that scenario's ScenarioResult
// rather than aborting the batch — one flaky/crashing scenario shouldn't
// cancel every other scenario's run. Run itself only returns a non-nil error
// when ctx is canceled (by the caller, or by a Runner that hangs uncancellably
// past ctx's own deadline), which errgroup surfaces through gctx.Err().
func (o *Orchestrator) Run(ctx context.Context, scenarios []corpus.Scenario) (Report, error) {
	resultsCh := make(chan indexedResult)
	group, gctx := errgroup.WithContext(ctx)
	sem := make(chan struct{}, o.concurrency)

	for index, sc := range scenarios {
		group.Go(func() error {
			select {
			case sem <- struct{}{}:
			case <-gctx.Done():
				return fmt.Errorf("scenario orchestrator: %w", gctx.Err())
			}
			defer func() { <-sem }()

			result, err := o.runner.Run(gctx, sc)
			sr := indexedResult{
				index:          index,
				ScenarioResult: ScenarioResult{Scenario: sc, Result: result, Err: err},
			}

			select {
			case resultsCh <- sr:
				return nil
			case <-gctx.Done():
				return fmt.Errorf("scenario orchestrator: %w", gctx.Err())
			}
		})
	}

	collected := make([]indexedResult, 0, len(scenarios))
	collectDone := make(chan struct{})
	go func() {
		defer close(collectDone)
		for sr := range resultsCh {
			collected = append(collected, sr)
		}
	}()

	waitErr := group.Wait()
	close(resultsCh)
	<-collectDone

	slices.SortFunc(collected, func(a, b indexedResult) int {
		return cmp.Compare(a.index, b.index)
	})
	results := make([]ScenarioResult, len(collected))
	for i := range collected {
		results[i] = collected[i].ScenarioResult
	}

	if waitErr != nil {
		return Report{Results: results}, fmt.Errorf("scenario orchestrator: %w", waitErr)
	}
	return Report{Results: results}, nil
}
