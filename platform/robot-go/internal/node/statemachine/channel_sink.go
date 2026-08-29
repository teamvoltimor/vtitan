package statemachine

import (
	"fmt"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/statemachine/command"
)

// UnimplementedChannelSink implements command.ChannelSink by returning a
// clear "not implemented" error from every method, so
// command.Dispatcher.Dispatch reports StatusFailed with an honest message
// rather than silently succeeding.
//
// The real implementations have no Go-side counterpart yet: vision debug
// streaming has no NATS subject or wire schema in this tree (vision stays
// Python-only, see internal/telemetry/diag.Detection's doc comment), and
// "the telemetry channel" is this same backend gRPC connection's sibling
// ingest stream, which in the Go architecture runs as a separate process
// (cmd/telemetry-node) rather than an in-process toggle -- disabling it
// from here would mean signalling across a process boundary this tree
// doesn't have a mechanism for yet. Both are real, scoped follow-ups, not
// oversights -- this sink exists so Dispatch's behavior for them is
// correct (fails loudly) in the meantime, instead of half-built.
type UnimplementedChannelSink struct{}

// SetVisionDebug implements command.ChannelSink.
func (UnimplementedChannelSink) SetVisionDebug(_ command.VisionDebugParams) error {
	return fmt.Errorf("node/statemachine: SetVisionDebug is not implemented on the Go robot stack yet")
}

// ToggleTelemetryChannel implements command.ChannelSink.
func (UnimplementedChannelSink) ToggleTelemetryChannel(_ bool) error {
	return fmt.Errorf("node/statemachine: ToggleTelemetryChannel is not implemented on the Go robot stack yet")
}

// DisableCommandChannel implements command.ChannelSink.
func (UnimplementedChannelSink) DisableCommandChannel() error {
	return fmt.Errorf("node/statemachine: DisableCommandChannel is not implemented on the Go robot stack yet")
}
