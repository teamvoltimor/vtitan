package command_test

import (
	"errors"
	"strings"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/driver/button"
	"github.com/teamvoltimor/vtitan/src/go/internal/statemachine/command"
)

// fakeSink records every call it receives and returns whichever
// canned error was configured for that method -- the same "fake, not
// mock framework" pattern internal/telemetry/diag's tests use for Source.
type fakeSink struct {
	publishButtonEventErr     error
	setVisionDebugErr         error
	toggleTelemetryChannelErr error
	disableCommandChannelErr  error

	publishedButtonEvent button.Kind
	visionDebugParams    command.VisionDebugParams
	telemetryEnabled     bool

	publishButtonEventCalled     bool
	setVisionDebugCalled         bool
	toggleTelemetryChannelCalled bool
	disableCommandChannelCalled  bool
}

func (f *fakeSink) PublishButtonEvent(kind button.Kind) error {
	f.publishButtonEventCalled = true
	f.publishedButtonEvent = kind
	return f.publishButtonEventErr
}

func (f *fakeSink) SetVisionDebug(params command.VisionDebugParams) error {
	f.setVisionDebugCalled = true
	f.visionDebugParams = params
	return f.setVisionDebugErr
}

func (f *fakeSink) ToggleTelemetryChannel(enabled bool) error {
	f.toggleTelemetryChannelCalled = true
	f.telemetryEnabled = enabled
	return f.toggleTelemetryChannelErr
}

func (f *fakeSink) DisableCommandChannel() error {
	f.disableCommandChannelCalled = true
	return f.disableCommandChannelErr
}

// TestDispatcher_Dispatch_Success covers every Kind's happy path,
// asserting both the ack Status and which Sink method (and with what
// argument) actually ran.
func TestDispatcher_Dispatch_Success(t *testing.T) {
	t.Parallel()

	wantStreamFPS := 15.0

	tests := []struct {
		name       string
		cmd        command.Command
		wantStatus command.Status
		assertSink func(t *testing.T, sink *fakeSink)
	}{
		{
			name:       "start race publishes a short press",
			cmd:        command.Command{Kind: command.KindStartRace},
			wantStatus: command.StatusAccepted,
			assertSink: func(t *testing.T, sink *fakeSink) {
				t.Helper()
				if !sink.publishButtonEventCalled {
					t.Fatal("PublishButtonEvent was not called")
				}
				if sink.publishedButtonEvent != button.KindShortPress {
					t.Fatalf("published %v, want KindShortPress", sink.publishedButtonEvent)
				}
			},
		},
		{
			name:       "stop race publishes a long press",
			cmd:        command.Command{Kind: command.KindStopRace},
			wantStatus: command.StatusAccepted,
			assertSink: func(t *testing.T, sink *fakeSink) {
				t.Helper()
				if sink.publishedButtonEvent != button.KindLongPress {
					t.Fatalf("published %v, want KindLongPress", sink.publishedButtonEvent)
				}
			},
		},
		{
			name:       "emergency stop publishes a long press",
			cmd:        command.Command{Kind: command.KindEmergencyStop},
			wantStatus: command.StatusAccepted,
			assertSink: func(t *testing.T, sink *fakeSink) {
				t.Helper()
				if sink.publishedButtonEvent != button.KindLongPress {
					t.Fatalf("published %v, want KindLongPress", sink.publishedButtonEvent)
				}
			},
		},
		{
			name: "set vision debug forwards params",
			cmd: command.Command{
				Kind:        command.KindSetVisionDebug,
				VisionDebug: command.VisionDebugParams{Enabled: true, StreamFPS: &wantStreamFPS},
			},
			wantStatus: command.StatusCompleted,
			assertSink: func(t *testing.T, sink *fakeSink) {
				t.Helper()
				if !sink.setVisionDebugCalled {
					t.Fatal("SetVisionDebug was not called")
				}
				if !sink.visionDebugParams.Enabled {
					t.Fatal("VisionDebugParams.Enabled = false, want true")
				}
				if sink.visionDebugParams.StreamFPS == nil || *sink.visionDebugParams.StreamFPS != 15 {
					t.Fatalf("VisionDebugParams.StreamFPS = %v, want 15", sink.visionDebugParams.StreamFPS)
				}
			},
		},
		{
			name:       "disable command channel",
			cmd:        command.Command{Kind: command.KindDisableCommandChannel},
			wantStatus: command.StatusCompleted,
			assertSink: func(t *testing.T, sink *fakeSink) {
				t.Helper()
				if !sink.disableCommandChannelCalled {
					t.Fatal("DisableCommandChannel was not called")
				}
			},
		},
		{
			name: "set telemetry channel enabled",
			cmd: command.Command{
				Kind:             command.KindSetTelemetryChannel,
				TelemetryChannel: command.TelemetryChannelParams{Enabled: true},
			},
			wantStatus: command.StatusCompleted,
			assertSink: func(t *testing.T, sink *fakeSink) {
				t.Helper()
				if !sink.toggleTelemetryChannelCalled {
					t.Fatal("ToggleTelemetryChannel was not called")
				}
				if !sink.telemetryEnabled {
					t.Fatal("telemetryEnabled = false, want true")
				}
			},
		},
		{
			name: "set telemetry channel disabled",
			cmd: command.Command{
				Kind:             command.KindSetTelemetryChannel,
				TelemetryChannel: command.TelemetryChannelParams{Enabled: false},
			},
			wantStatus: command.StatusCompleted,
			assertSink: func(t *testing.T, sink *fakeSink) {
				t.Helper()
				if sink.telemetryEnabled {
					t.Fatal("telemetryEnabled = true, want false")
				}
			},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			sink := &fakeSink{}
			dispatcher := command.NewDispatcher(sink, sink)

			status, message := dispatcher.Dispatch(tt.cmd)

			if status != tt.wantStatus {
				t.Fatalf("Dispatch() status = %v, want %v (message: %q)", status, tt.wantStatus, message)
			}
			if message == "" {
				t.Fatal("Dispatch() message is empty, want a human-readable ack message")
			}
			tt.assertSink(t, sink)
		})
	}
}

// TestDispatcher_Dispatch_SinkFailures covers every Kind's failure path:
// a Sink error must surface as StatusFailed with the underlying error
// folded into the message, never silently swallowed or reported as
// success.
func TestDispatcher_Dispatch_SinkFailures(t *testing.T) {
	t.Parallel()

	wantErr := errors.New("transport unavailable")

	tests := []struct {
		name string
		cmd  command.Command
		sink *fakeSink
	}{
		{
			name: "start race publish failure",
			cmd:  command.Command{Kind: command.KindStartRace},
			sink: &fakeSink{publishButtonEventErr: wantErr},
		},
		{
			name: "set vision debug failure",
			cmd:  command.Command{Kind: command.KindSetVisionDebug},
			sink: &fakeSink{setVisionDebugErr: wantErr},
		},
		{
			name: "disable command channel failure",
			cmd:  command.Command{Kind: command.KindDisableCommandChannel},
			sink: &fakeSink{disableCommandChannelErr: wantErr},
		},
		{
			name: "set telemetry channel failure",
			cmd:  command.Command{Kind: command.KindSetTelemetryChannel},
			sink: &fakeSink{toggleTelemetryChannelErr: wantErr},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			dispatcher := command.NewDispatcher(tt.sink, tt.sink)

			status, message := dispatcher.Dispatch(tt.cmd)

			if status != command.StatusFailed {
				t.Fatalf("Dispatch() status = %v, want StatusFailed", status)
			}
			if !strings.Contains(message, wantErr.Error()) {
				t.Fatalf("Dispatch() message = %q, want it to contain %q", message, wantErr.Error())
			}
		})
	}
}

// TestDispatcher_Dispatch_Unimplemented mirrors _dispatch_command's
// fallthrough branch for a command kind the robot doesn't recognize.
func TestDispatcher_Dispatch_Unimplemented(t *testing.T) {
	t.Parallel()

	sink := &fakeSink{}
	dispatcher := command.NewDispatcher(sink, sink)

	status, message := dispatcher.Dispatch(
		command.Command{Kind: command.KindUnimplemented, UnimplementedLabel: "reboot"},
	)

	if status != command.StatusFailed {
		t.Fatalf("Dispatch() status = %v, want StatusFailed", status)
	}
	if !strings.Contains(message, "reboot") {
		t.Fatalf("Dispatch() message = %q, want it to mention the unimplemented label", message)
	}
	if sink.publishButtonEventCalled || sink.setVisionDebugCalled || sink.toggleTelemetryChannelCalled ||
		sink.disableCommandChannelCalled {
		t.Fatal("an unimplemented command still reached the Sink")
	}
}
