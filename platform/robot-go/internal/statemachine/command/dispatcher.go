package command

import (
	"fmt"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/button"
)

// ButtonSink is the one action every start/stop/e-stop Command maps to --
// the transport-independent replacement for command_channel.py's
// `_button_pub` rclpy Publisher field. Kept separate from ChannelSink
// (interfacebloat caps interfaces at 3 methods, and these two are
// genuinely different concerns: this one reaches the state machine
// itself, ChannelSink only administers the backend channels).
type ButtonSink interface {
	// PublishButtonEvent publishes a synthetic button event, the Go
	// analog of `_dispatch_button_event` publishing a String onto
	// /button/event. kind reuses internal/driver/button's Kind rather
	// than a duplicate enum, since the wire vocabulary
	// ("short_press"/"long_press") is exactly the physical button
	// driver's own.
	PublishButtonEvent(kind button.Kind) error
}

// ChannelSink is the set of backend-channel administrative actions a
// Command can request -- the transport-independent replacement for
// command_channel.py's `_vision_params_client` field and its
// `_telemetry_channel` reference. Defined here, at the point of use
// (go-architect §4, matching internal/telemetry/diag.Source and
// internal/sim/scenario.Runner), so Dispatch is fully testable against a
// fake Sink today; a real implementation backed by NATS publish/request-
// reply drops in later without this package changing.
type ChannelSink interface {
	// SetVisionDebug forwards vision_detector's debug-stream parameters,
	// the Go analog of `_dispatch_set_vision_debug`'s
	// vision_params_client.call_async(SetParameters.Request(...)).
	SetVisionDebug(params VisionDebugParams) error

	// ToggleTelemetryChannel starts or stops the telemetry ingest
	// channel, the Go analog of `_dispatch_set_telemetry_channel`
	// calling `_telemetry_channel.restart()`/`.stop()`.
	ToggleTelemetryChannel(enabled bool) error

	// DisableCommandChannel closes the command stream, the Go analog of
	// `_dispatch_disable_command_channel` calling `self.stop()`.
	DisableCommandChannel() error
}

// Dispatcher decides what a Command maps to and reports the outcome,
// mirroring command_channel.py's `_dispatch_command` and its `_dispatch_*`
// helpers. All the actual side effects happen through ButtonSink/
// ChannelSink; Dispatcher itself does no I/O. A single concrete type
// satisfying both interfaces is the expected real shape (one backend
// connection administers both), but Dispatcher only depends on the
// narrow slice of behavior each Command kind actually needs.
type Dispatcher struct {
	buttonSink  ButtonSink
	channelSink ChannelSink
}

// NewDispatcher builds a Dispatcher over buttonSink and channelSink.
func NewDispatcher(buttonSink ButtonSink, channelSink ChannelSink) *Dispatcher {
	return &Dispatcher{buttonSink: buttonSink, channelSink: channelSink}
}

// Dispatch applies cmd through the Dispatcher's sinks and returns the ack
// status and message to report back to the backend, matching
// command_channel.py's `_dispatch_command` return shape
// `tuple[int, str]`.
func (d *Dispatcher) Dispatch(cmd Command) (status Status, message string) {
	switch cmd.Kind {
	case KindStartRace:
		return d.dispatchButtonEvent(button.KindShortPress, "start race")
	case KindStopRace:
		return d.dispatchButtonEvent(button.KindLongPress, "stop race")
	case KindEmergencyStop:
		return d.dispatchButtonEvent(button.KindLongPress, "emergency stop")
	case KindSetVisionDebug:
		return d.dispatchSetVisionDebug(cmd.VisionDebug)
	case KindDisableCommandChannel:
		return d.dispatchDisableCommandChannel()
	case KindSetTelemetryChannel:
		return d.dispatchSetTelemetryChannel(cmd.TelemetryChannel)
	case KindUnimplemented:
		return StatusFailed, fmt.Sprintf("command type %q is not implemented on this robot", cmd.UnimplementedLabel)
	}
	return StatusFailed, fmt.Sprintf("command type %q is not implemented on this robot", cmd.Kind)
}

// dispatchButtonEvent publishes a synthetic button event, matching
// `_dispatch_button_event`. Unlike the Python original (fire-and-forget,
// always ACCEPTED because an rclpy Publisher can't report failure
// synchronously), a ButtonSink implementation backed by a real transport
// can fail to publish -- that failure is reported as StatusFailed rather
// than papered over, which is a deliberate improvement on the port, not
// an unfaithful one: the ack contract already has a Failed status for
// exactly this.
func (d *Dispatcher) dispatchButtonEvent(kind button.Kind, label string) (status Status, message string) {
	if err := d.buttonSink.PublishButtonEvent(kind); err != nil {
		return StatusFailed, fmt.Sprintf("failed to publish synthetic button event for %s: %v", label, err)
	}
	return StatusAccepted, fmt.Sprintf("published synthetic button event %q for %s", kind, label)
}

// dispatchSetVisionDebug matches `_dispatch_set_vision_debug`. The
// service-unavailable/timeout/rejected-parameters distinctions the Python
// original makes are the ChannelSink implementation's concern (they're
// specific to the ROS2 SetParameters service call, which has no Go
// equivalent in this tree yet) -- this layer only distinguishes success
// from failure.
func (d *Dispatcher) dispatchSetVisionDebug(params VisionDebugParams) (status Status, message string) {
	if err := d.channelSink.SetVisionDebug(params); err != nil {
		return StatusFailed, fmt.Sprintf("vision_detector rejected parameters: %v", err)
	}
	return StatusCompleted, "vision debug stream updated"
}

// dispatchDisableCommandChannel matches
// `_dispatch_disable_command_channel`, including its message wording --
// this is deliberately one-way, see commands.proto's docstring referenced
// there.
func (d *Dispatcher) dispatchDisableCommandChannel() (status Status, message string) {
	if err := d.channelSink.DisableCommandChannel(); err != nil {
		return StatusFailed, fmt.Sprintf("failed to disable command channel: %v", err)
	}
	return StatusCompleted, "command channel disabled; robot-side param access required to re-enable"
}

// dispatchSetTelemetryChannel matches `_dispatch_set_telemetry_channel`.
// The Python original also special-cases `_telemetry_channel is None`
// ("telemetry channel is not configured on this robot") for a robot built
// without one; here that's simply whatever error a ChannelSink
// implementation without a configured telemetry channel chooses to
// return from ToggleTelemetryChannel, rather than a distinct code path --
// ChannelSink is always present in this design (never an optional
// pointer), so "not configured" is the implementation's concern, not the
// Dispatcher's.
func (d *Dispatcher) dispatchSetTelemetryChannel(params TelemetryChannelParams) (status Status, message string) {
	if err := d.channelSink.ToggleTelemetryChannel(params.Enabled); err != nil {
		return StatusFailed, fmt.Sprintf("failed to toggle telemetry channel: %v", err)
	}
	verb := "disabled"
	if params.Enabled {
		verb = "enabled"
	}
	return StatusCompleted, "telemetry channel " + verb
}
