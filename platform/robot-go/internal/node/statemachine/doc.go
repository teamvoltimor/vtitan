// Package statemachine assembles internal/statemachine/command's Dispatcher
// with real sinks for cmd/state-machine (and, later, cmd/pi5): a
// NATSButtonSink publishing synthetic button events on the same
// vtitan.ui.v1.button_event subject a physical button press would, and an
// UnimplementedChannelSink for the backend-channel administrative commands
// (vision debug stream, telemetry channel toggle, command-channel disable)
// this Go port doesn't have a real implementation for yet -- see
// UnimplementedChannelSink's doc comment for why that's an explicit,
// correctly-failing stand-in rather than a silently-succeeding stub.
package statemachine
