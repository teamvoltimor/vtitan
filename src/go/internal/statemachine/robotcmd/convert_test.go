package robotcmd

import (
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/statemachine/command"

	telemetryv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/telemetry/v1"
)

func TestToCommand(t *testing.T) {
	t.Parallel()

	streamFPS := uint32(15)

	tests := []struct {
		name    string
		payload *telemetryv1.RobotCommand
		want    command.Command
	}{
		{
			name: "start race",
			payload: &telemetryv1.RobotCommand{
				Payload: &telemetryv1.RobotCommand_StartRace{StartRace: &telemetryv1.StartRaceParams{}},
			},
			want: command.Command{Kind: command.KindStartRace},
		},
		{
			name: "stop race",
			payload: &telemetryv1.RobotCommand{
				Payload: &telemetryv1.RobotCommand_StopRace{StopRace: &telemetryv1.StopRaceParams{}},
			},
			want: command.Command{Kind: command.KindStopRace},
		},
		{
			name: "emergency stop",
			payload: &telemetryv1.RobotCommand{
				Payload: &telemetryv1.RobotCommand_EmergencyStop{EmergencyStop: &telemetryv1.EmergencyStopParams{}},
			},
			want: command.Command{Kind: command.KindEmergencyStop},
		},
		{
			name: "set vision debug with stream fps",
			payload: &telemetryv1.RobotCommand{Payload: &telemetryv1.RobotCommand_SetVisionDebug{
				SetVisionDebug: &telemetryv1.SetVisionDebugParams{Enabled: true, StreamFps: &streamFPS},
			}},
			want: command.Command{
				Kind:        command.KindSetVisionDebug,
				VisionDebug: command.VisionDebugParams{Enabled: true, StreamFPS: floatPtr(15)},
			},
		},
		{
			name: "set vision debug without stream fps",
			payload: &telemetryv1.RobotCommand{Payload: &telemetryv1.RobotCommand_SetVisionDebug{
				SetVisionDebug: &telemetryv1.SetVisionDebugParams{Enabled: false},
			}},
			want: command.Command{
				Kind:        command.KindSetVisionDebug,
				VisionDebug: command.VisionDebugParams{Enabled: false},
			},
		},
		{
			name: "disable command channel",
			payload: &telemetryv1.RobotCommand{Payload: &telemetryv1.RobotCommand_DisableCommandChannel{
				DisableCommandChannel: &telemetryv1.DisableCommandChannelParams{},
			}},
			want: command.Command{Kind: command.KindDisableCommandChannel},
		},
		{
			name: "set telemetry channel",
			payload: &telemetryv1.RobotCommand{Payload: &telemetryv1.RobotCommand_SetTelemetryChannel{
				SetTelemetryChannel: &telemetryv1.SetTelemetryChannelParams{Enabled: true},
			}},
			want: command.Command{
				Kind:             command.KindSetTelemetryChannel,
				TelemetryChannel: command.TelemetryChannelParams{Enabled: true},
			},
		},
		{
			name: "pause is unimplemented, not unset",
			payload: &telemetryv1.RobotCommand{
				Payload: &telemetryv1.RobotCommand_Pause{Pause: &telemetryv1.PauseParams{}},
			},
			want: command.Command{Kind: command.KindUnimplemented, UnimplementedLabel: "pause"},
		},
		{
			name: "reboot is unimplemented, not unset",
			payload: &telemetryv1.RobotCommand{
				Payload: &telemetryv1.RobotCommand_Reboot{Reboot: &telemetryv1.RebootParams{}},
			},
			want: command.Command{Kind: command.KindUnimplemented, UnimplementedLabel: "reboot"},
		},
		{
			name:    "no payload at all",
			payload: &telemetryv1.RobotCommand{},
			want:    command.Command{Kind: command.KindUnimplemented, UnimplementedLabel: "unset"},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			got := toCommand(tt.payload)
			if got.Kind != tt.want.Kind {
				t.Errorf("Kind = %v, want %v", got.Kind, tt.want.Kind)
			}
			if got.UnimplementedLabel != tt.want.UnimplementedLabel {
				t.Errorf("UnimplementedLabel = %q, want %q", got.UnimplementedLabel, tt.want.UnimplementedLabel)
			}
			if got.VisionDebug.Enabled != tt.want.VisionDebug.Enabled {
				t.Errorf("VisionDebug.Enabled = %v, want %v", got.VisionDebug.Enabled, tt.want.VisionDebug.Enabled)
			}
			if (got.VisionDebug.StreamFPS == nil) != (tt.want.VisionDebug.StreamFPS == nil) {
				t.Errorf("VisionDebug.StreamFPS presence = %v, want %v",
					got.VisionDebug.StreamFPS != nil, tt.want.VisionDebug.StreamFPS != nil)
			} else if got.VisionDebug.StreamFPS != nil && *got.VisionDebug.StreamFPS != *tt.want.VisionDebug.StreamFPS {
				t.Errorf("VisionDebug.StreamFPS = %v, want %v",
					*got.VisionDebug.StreamFPS, *tt.want.VisionDebug.StreamFPS)
			}
			if got.TelemetryChannel != tt.want.TelemetryChannel {
				t.Errorf("TelemetryChannel = %+v, want %+v", got.TelemetryChannel, tt.want.TelemetryChannel)
			}
		})
	}
}

func TestToProtoStatus(t *testing.T) {
	t.Parallel()

	tests := []struct {
		status command.Status
		want   telemetryv1.CommandExecutionStatus
	}{
		{command.StatusAccepted, telemetryv1.CommandExecutionStatus_COMMAND_EXECUTION_STATUS_ACCEPTED},
		{command.StatusCompleted, telemetryv1.CommandExecutionStatus_COMMAND_EXECUTION_STATUS_COMPLETED},
		{command.StatusFailed, telemetryv1.CommandExecutionStatus_COMMAND_EXECUTION_STATUS_FAILED},
		{command.StatusUnspecified, telemetryv1.CommandExecutionStatus_COMMAND_EXECUTION_STATUS_UNSPECIFIED},
	}

	for _, tt := range tests {
		t.Run(tt.status.String(), func(t *testing.T) {
			t.Parallel()

			if got := toProtoStatus(tt.status); got != tt.want {
				t.Errorf("toProtoStatus(%v) = %v, want %v", tt.status, got, tt.want)
			}
		})
	}
}

//nolint:modernize // gopls' new(expr) suggestion here isn't valid Go (new takes a type, not a value)
func floatPtr(f float64) *float64 {
	return &f
}
