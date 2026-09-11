package navigation

import "context"

// Service is the Navigation bounded context's port.
type Service interface {
	ListWaypoints(ctx context.Context) ([]Waypoint, error)
	CreateWaypoint(ctx context.Context, req CreateWaypointRequest) (Waypoint, error)
	DeleteWaypoint(ctx context.Context, id string) error

	Route(ctx context.Context) (Route, error)
	PlanRoute(ctx context.Context, req PlanRouteRequest) (Route, error)

	Status(ctx context.Context) (Status, error)
	Clearance(ctx context.Context) (Clearance, error)

	Tuning(ctx context.Context) (Tuning, error)
	UpdateTuning(ctx context.Context, t Tuning) (Tuning, error)
}

// service is the concrete implementation of Service.
type service struct {
	store Store
}

// NewService constructs a Service backed by the given Store.
func NewService(store Store) Service {
	return &service{store: store}
}

func (s *service) ListWaypoints(ctx context.Context) ([]Waypoint, error) {
	return s.store.ListWaypoints(ctx)
}

func (s *service) CreateWaypoint(ctx context.Context, req CreateWaypointRequest) (Waypoint, error) {
	return s.store.CreateWaypoint(ctx, req)
}

func (s *service) DeleteWaypoint(ctx context.Context, id string) error {
	return s.store.DeleteWaypoint(ctx, id)
}

func (s *service) Route(ctx context.Context) (Route, error) {
	return s.store.Route(ctx)
}

func (s *service) PlanRoute(ctx context.Context, req PlanRouteRequest) (Route, error) {
	return s.store.PlanRoute(ctx, req)
}

func (s *service) Status(ctx context.Context) (Status, error) {
	return s.store.Status(ctx)
}

func (s *service) Clearance(ctx context.Context) (Clearance, error) {
	return s.store.Clearance(ctx)
}

func (s *service) Tuning(ctx context.Context) (Tuning, error) {
	return s.store.Tuning(ctx)
}

func (s *service) UpdateTuning(ctx context.Context, t Tuning) (Tuning, error) {
	return s.store.UpdateTuning(ctx, t)
}
