// Package driver defines the small, hardware-agnostic contract every
// sensor/actuator driver in this module implements — see
// platform/robot/docs/internal/plans/go-migration-plan.md ("Design
// patterns"). It exists as its own package, separate from internal/driver/
// {imu,lidar,motor}, because the contract is genuinely shared by all three
// planned implementations today, not a speculative abstraction.
package driver

import "context"

// Driver reads successive values of T from a hardware source. Connect must
// be called before Read; Close releases the underlying resource (serial
// port, I2C bus, etc.) and makes the Driver unusable.
type Driver[T any] interface {
	Connect(ctx context.Context) error
	Read(ctx context.Context) (T, error)
	Close() error
}
