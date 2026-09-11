package robot

import (
	"context"
	"errors"
)

// ErrNotFound is returned when a robot, or a resource scoped to one, does not exist.
var ErrNotFound = errors.New("robot not found")

// Store is the storage port for the Robot bounded context.
type Store interface {
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
