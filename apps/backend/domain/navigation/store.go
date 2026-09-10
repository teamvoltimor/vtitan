package navigation

import "context"

// Store is the storage port for the Navigation bounded context.
type Store interface {
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
