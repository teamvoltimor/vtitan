package simulation

import "context"

// Service is the Simulation bounded context's port.
type Service interface {
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

// service is the concrete implementation of Service.
type service struct {
	store Store
}

// NewService constructs a Service backed by the given Store.
func NewService(store Store) Service {
	return &service{store: store}
}

func (s *service) ListScenarios(ctx context.Context, challenge Challenge, limit int) ([]Scenario, error) {
	return s.store.ListScenarios(ctx, challenge, limit)
}

func (s *service) GenerateScenario(ctx context.Context, req GenerateScenarioRequest) (Scenario, error) {
	return s.store.GenerateScenario(ctx, req)
}

func (s *service) GetScenario(ctx context.Context, id string) (Scenario, error) {
	return s.store.GetScenario(ctx, id)
}

func (s *service) DeleteScenario(ctx context.Context, id string) error {
	return s.store.DeleteScenario(ctx, id)
}

func (s *service) ListRuns(ctx context.Context) ([]Run, error) {
	return s.store.ListRuns(ctx)
}

func (s *service) StartRun(ctx context.Context, req StartRunRequest) (Run, error) {
	return s.store.StartRun(ctx, req)
}

func (s *service) GetRun(ctx context.Context, id string) (Run, error) {
	return s.store.GetRun(ctx, id)
}

func (s *service) ControlRun(ctx context.Context, id string, action RunAction) (Run, error) {
	return s.store.ControlRun(ctx, id, action)
}

func (s *service) ListEnvironments(ctx context.Context) ([]Environment, error) {
	return s.store.ListEnvironments(ctx)
}
