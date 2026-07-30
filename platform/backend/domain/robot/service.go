package robot

import "context"

// Service is the Robot bounded context's port.
type Service interface {
	List(ctx context.Context, fleetID string, state State) ([]Robot, error)
	Create(ctx context.Context, req CreateRequest) (Robot, error)
	Get(ctx context.Context, id string) (Robot, error)
	Update(ctx context.Context, id string, req UpdateRequest) (Robot, error)
	Delete(ctx context.Context, id string) error

	Status(ctx context.Context, id string) (Status, error)

	Config(ctx context.Context, id string) (Config, error)
	UpdateConfig(ctx context.Context, id string, req UpdateConfigRequest) (Config, error)

	Command(ctx context.Context, id string, cmd Command) (CommandResult, error)
}

// service is the concrete implementation of Service.
type service struct {
	store      Store
	dispatcher Dispatcher
}

// NewService constructs a Service backed by the given Store. dispatcher may
// be nil, in which case Command falls back to Store.Command.
func NewService(store Store, dispatcher Dispatcher) Service {
	return &service{store: store, dispatcher: dispatcher}
}

func (s *service) List(ctx context.Context, fleetID string, state State) ([]Robot, error) {
	return s.store.List(ctx, fleetID, state)
}

func (s *service) Create(ctx context.Context, req CreateRequest) (Robot, error) {
	return s.store.Create(ctx, req)
}

func (s *service) Get(ctx context.Context, id string) (Robot, error) {
	return s.store.Get(ctx, id)
}

func (s *service) Update(ctx context.Context, id string, req UpdateRequest) (Robot, error) {
	return s.store.Update(ctx, id, req)
}

func (s *service) Delete(ctx context.Context, id string) error {
	return s.store.Delete(ctx, id)
}

func (s *service) Status(ctx context.Context, id string) (Status, error) {
	return s.store.Status(ctx, id)
}

func (s *service) Config(ctx context.Context, id string) (Config, error) {
	return s.store.Config(ctx, id)
}

func (s *service) UpdateConfig(ctx context.Context, id string, req UpdateConfigRequest) (Config, error) {
	return s.store.UpdateConfig(ctx, id, req)
}

func (s *service) Command(ctx context.Context, id string, cmd Command) (CommandResult, error) {
	if s.dispatcher == nil {
		return s.store.Command(ctx, id, cmd)
	}
	if _, err := s.store.Get(ctx, id); err != nil {
		return CommandResult{}, err
	}
	return s.dispatcher.Dispatch(ctx, id, cmd)
}
