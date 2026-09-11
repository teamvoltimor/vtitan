package robotcmd

import (
	"github.com/teamvoltimor/vtitan/src/go/internal/statemachine/command"

	telemetryv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/telemetry/v1"
)

// toCommand decodes rc's oneof payload into a command.Command, mirroring
// command_channel.py's `cmd.WhichOneof("payload")` dispatch. start_race/
// stop_race/emergency_stop map to the same synthetic-button-event Kinds
// command.Dispatcher already handles. pause/resume/return_to_start/reboot/
// shutdown decode to a real, correctly-labeled KindUnimplemented --
// state_machine_node draws no distinction for them either (see
// command.Dispatcher's doc comment) -- rather than falling through to the
// same generic label a genuinely-unset payload would get.
func toCommand(rc *telemetryv1.RobotCommand) command.Command {
	switch payload := rc.GetPayload().(type) {
	case *telemetryv1.RobotCommand_StartRace:
		return command.Command{Kind: command.KindStartRace}
	case *telemetryv1.RobotCommand_StopRace:
		return command.Command{Kind: command.KindStopRace}
	case *telemetryv1.RobotCommand_EmergencyStop:
		return command.Command{Kind: command.KindEmergencyStop}
	case *telemetryv1.RobotCommand_SetVisionDebug:
		return command.Command{
			Kind:        command.KindSetVisionDebug,
			VisionDebug: visionDebugParams(payload.SetVisionDebug),
		}
	case *telemetryv1.RobotCommand_DisableCommandChannel:
		return command.Command{Kind: command.KindDisableCommandChannel}
	case *telemetryv1.RobotCommand_SetTelemetryChannel:
		return command.Command{
			Kind:             command.KindSetTelemetryChannel,
			TelemetryChannel: command.TelemetryChannelParams{Enabled: payload.SetTelemetryChannel.GetEnabled()},
		}
	case *telemetryv1.RobotCommand_Pause:
		return unimplemented("pause")
	case *telemetryv1.RobotCommand_Resume:
		return unimplemented("resume")
	case *telemetryv1.RobotCommand_ReturnToStart:
		return unimplemented("return_to_start")
	case *telemetryv1.RobotCommand_Reboot:
		return unimplemented("reboot")
	case *telemetryv1.RobotCommand_Shutdown:
		return unimplemented("shutdown")
	default:
		// A malformed message with no payload at all -- the backend's own
		// buf.validate "required" rule on RobotCommand.payload should
		// already reject this before it reaches here.
		return unimplemented("unset")
	}
}

// unimplemented builds the Command Dispatcher.Dispatch reports StatusFailed
// for, matching command_channel.py's fallthrough message
// ("command type '{kind}' is not implemented on this robot").
func unimplemented(label string) command.Command {
	return command.Command{Kind: command.KindUnimplemented, UnimplementedLabel: label}
}

// visionDebugParams converts SetVisionDebugParams' *uint32 StreamFps into
// command.VisionDebugParams' *float64 StreamFPS -- the params type
// predates this client and picked float64 (no Go equivalent existed to
// match units against yet); the wire value is always a whole FPS count, so
// the conversion is exact in both directions.
func visionDebugParams(params *telemetryv1.SetVisionDebugParams) command.VisionDebugParams {
	out := command.VisionDebugParams{Enabled: params.GetEnabled()}
	if params.StreamFps != nil {
		fps := float64(*params.StreamFps)
		out.StreamFPS = &fps
	}
	return out
}

// toProtoStatus maps a command.Status to its commands.proto wire value.
// command.StatusUnspecified is never a real Dispatch outcome (see its doc
// comment) but is mapped defensively rather than panicking, since acking
// UNSPECIFIED would itself be rejected by the backend's buf.validate rule
// -- toProtoStatus's caller logs this case rather than sending the ack.
func toProtoStatus(s command.Status) telemetryv1.CommandExecutionStatus {
	switch s {
	case command.StatusAccepted:
		return telemetryv1.CommandExecutionStatus_COMMAND_EXECUTION_STATUS_ACCEPTED
	case command.StatusCompleted:
		return telemetryv1.CommandExecutionStatus_COMMAND_EXECUTION_STATUS_COMPLETED
	case command.StatusFailed:
		return telemetryv1.CommandExecutionStatus_COMMAND_EXECUTION_STATUS_FAILED
	case command.StatusUnspecified:
		return telemetryv1.CommandExecutionStatus_COMMAND_EXECUTION_STATUS_UNSPECIFIED
	default:
		return telemetryv1.CommandExecutionStatus_COMMAND_EXECUTION_STATUS_UNSPECIFIED
	}
}
