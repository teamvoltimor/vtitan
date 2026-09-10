package command

// Kind identifies which backend RobotCommand payload a Command carries --
// the Go analog of command_channel.py's `cmd.WhichOneof("payload")`
// dispatch on commands.proto's RobotCommand oneof.
type Kind int

// VisionDebugParams mirrors commands.proto's SetVisionDebugParams as
// consumed by `_dispatch_set_vision_debug`: whether the annotated debug
// stream is enabled, and an optional target FPS. StreamFPS is nil when
// the backend didn't set it, matching the Python `params.HasField
// ("stream_fps")` check (proto3 optional, explicit presence).
type VisionDebugParams struct {
	StreamFPS *float64
	Enabled   bool
}

// TelemetryChannelParams mirrors commands.proto's SetTelemetryChannelParams
// as consumed by `_dispatch_set_telemetry_channel`.
type TelemetryChannelParams struct {
	Enabled bool
}

// Command is one decoded backend RobotCommand, already stripped of its
// wire/oneof shape -- the input to Dispatcher.Dispatch. Exactly one of
// VisionDebug/TelemetryChannel/UnimplementedLabel is meaningful, selected
// by Kind, mirroring the proto oneof's own exclusivity.
type Command struct {
	UnimplementedLabel string
	VisionDebug        VisionDebugParams
	TelemetryChannel   TelemetryChannelParams
	Kind               Kind
}

const (
	// KindStartRace maps to `_dispatch_button_event("short_press", ...)`.
	KindStartRace Kind = iota
	// KindStopRace maps to `_dispatch_button_event("long_press", ...)`,
	// same as KindEmergencyStop -- state_machine_node draws no distinction
	// between a graceful stop and an e-stop today, see
	// command_channel.py's `_dispatch_command` comment.
	KindStopRace
	// KindEmergencyStop maps to `_dispatch_button_event("long_press", ...)`.
	KindEmergencyStop
	// KindSetVisionDebug maps to `_dispatch_set_vision_debug`.
	KindSetVisionDebug
	// KindDisableCommandChannel maps to `_dispatch_disable_command_channel`.
	KindDisableCommandChannel
	// KindSetTelemetryChannel maps to `_dispatch_set_telemetry_channel`.
	KindSetTelemetryChannel
	// KindUnimplemented is any command payload command_channel.py doesn't
	// recognize -- the Python `_dispatch_command`'s fallthrough branch
	// (`f"command type '{kind}' is not implemented on this robot"`).
	// UnimplementedLabel on the Command carries the raw wire label for
	// that message.
	KindUnimplemented
)

// unknownKindLabel is Kind.String's fallback for a value outside its
// defined range.
const unknownKindLabel = "unknown"

// String returns a short label for logging/error messages. Not a wire
// value -- Kind never crosses the wire itself, only the Command it's
// embedded in does, once a real commands.proto client exists in this
// tree.
func (k Kind) String() string {
	switch k {
	case KindStartRace:
		return "start_race"
	case KindStopRace:
		return "stop_race"
	case KindEmergencyStop:
		return "emergency_stop"
	case KindSetVisionDebug:
		return "set_vision_debug"
	case KindDisableCommandChannel:
		return "disable_command_channel"
	case KindSetTelemetryChannel:
		return "set_telemetry_channel"
	case KindUnimplemented:
		return "unimplemented"
	default:
		return unknownKindLabel
	}
}
