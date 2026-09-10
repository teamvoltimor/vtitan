// Package robotcmd is the gRPC client half of the backend command channel:
// it dials platform/backend's already-running RobotCommandService
// (platform/proto/telemetry/v1/commands.proto), decodes each streamed
// RobotCommand into an internal/statemachine/command.Command, dispatches
// it through a command.Dispatcher, and acks the outcome back --
// mirroring platform/robot/ros2_ws/src/vtitan_state_machine's
// command_channel.py's CommandChannel exactly, on this transport.
//
// The migration plan's NATS choice (docs/internal/plans/go-migration-plan.md)
// is scoped to intra-fleet messaging between this robot's own boards, not
// backend<->robot traffic -- that stays on the backend's existing gRPC
// channel, which is already deployed and already used by the Python stack
// this ports. internal/statemachine/command's Dispatcher/Command types
// were already transport-independent before this package existed
// (command/doc.go), so nothing there changes -- this is the first, and so
// far only, thing that decodes a real wire command into that shape.
package robotcmd
