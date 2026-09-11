package simulation

import "context"

// Store is the storage port for the Simulation bounded context.
type Store interface {
	ListScenarios(ctx context.Context, challenge Challenge, limit int) ([]Scenario, error)
	GenerateScenario(ctx context.Context, req GenerateScenarioRequest) (Scenario, error)
	GetScenario(ctx context.Context, id string) (Scenario, error)
	DeleteScenario(ctx context.Context, id string) error

	ListRuns(ctx context.Context) ([]Run, error)
	StartRun(ctx context.Context, req StartRunRequest) (Run, error)
	GetRun(ctx context.Context, id string) (Run, error)
	ControlRun(ctx context.Context, id string, action RunAction) (Run, error)

	ListEnvironments(ctx context.Context) ([]Environment, error)
}
