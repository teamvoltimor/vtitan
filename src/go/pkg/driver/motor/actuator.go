package motor

import "context"

// Actuator is the actuator-shaped counterpart to driver.Driver[T]
// (src/go/pkg/driver): commanded via SetSpeed rather than
// sampled via Read. Driver (driver.go) is its only implementation today.
type Actuator interface {
	Connect(ctx context.Context) error
	SetSpeed(ctx context.Context, normalizedSpeed float64) error
	Close() error
}
