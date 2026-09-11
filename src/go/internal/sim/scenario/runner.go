package scenario

import (
	"context"

	"github.com/teamvoltimor/vtitan/src/go/internal/sim/corpus"
)

// Runner runs one scenario and reports how it went. Orchestrator depends on
// this interface, not on any concrete implementation — defined here at the
// point of use (go-architect §4) so the orchestrator's concurrency and
// aggregation logic is testable with a fake Runner, and so today's
// subprocess-based implementation (SubprocessRunner, shelling out to the
// existing Python simulator) can later be swapped for a native Go simulator
// without touching the orchestrator at all. That swap is gated on profiling
// the Python sim's hot path first — see the "Simulation" section of
// docs/internal/plans/go-migration-plan.md — and isn't implemented here.
type Runner interface {
	Run(ctx context.Context, sc corpus.Scenario) (Result, error)
}
