package robot

import "context"

// Dispatcher pushes a Command down the live gRPC command channel to a
// connected robot and reports back the real dispatch outcome (queued for
// delivery, delivered, or rejected). It is optional: when Service is
// constructed with a nil Dispatcher, Command falls back to Store.Command,
// which only records that a command was received without forwarding it
// anywhere.
type Dispatcher interface {
	Dispatch(ctx context.Context, robotID string, cmd Command) (CommandResult, error)
}
