package robotcmd

import (
	"context"
	"testing"
	"time"

	"google.golang.org/grpc"

	robotdomain "github.com/teamvoltimor/vtitan/platform/backend/domain/robot"
	telemetryv1 "github.com/teamvoltimor/vtitan/platform/backend/gen/telemetry/v1"
)

// fakeStream is a minimal telemetryv1.RobotCommandService_StreamCommandsServer
// backed by a channel, so tests can drive StreamCommands without a real
// network connection.
type fakeStream struct {
	grpc.ServerStream
	ctx context.Context
	out chan *telemetryv1.RobotCommand
}

func (s *fakeStream) Send(cmd *telemetryv1.RobotCommand) error {
	s.out <- cmd
	return nil
}

func (s *fakeStream) Context() context.Context { return s.ctx }

func newFakeStream(ctx context.Context) *fakeStream {
	return &fakeStream{ctx: ctx, out: make(chan *telemetryv1.RobotCommand, maxBacklog+1)}
}

func TestDispatch_NoConnectedRobot_QueuesAndReplaysOnConnect(t *testing.T) {
	s := New()
	ctx := context.Background()

	res, err := s.Dispatch(ctx, "robot-1", robotdomain.Command{Type: robotdomain.CommandStartRace})
	if err != nil {
		t.Fatalf("Dispatch: %v", err)
	}
	if res.Status != robotdomain.CommandQueued {
		t.Fatalf("Status = %q, want queued", res.Status)
	}

	streamCtx, cancel := context.WithCancel(ctx)
	defer cancel()
	fs := newFakeStream(streamCtx)
	go func() {
		_ = s.StreamCommands(&telemetryv1.StreamCommandsRequest{RobotId: "robot-1"}, fs)
	}()

	select {
	case cmd := <-fs.out:
		if cmd.CommandId != res.CommandID {
			t.Fatalf("replayed command id = %q, want %q", cmd.CommandId, res.CommandID)
		}
		if cmd.GetStartRace() == nil {
			t.Fatalf("expected StartRace payload, got %#v", cmd.Payload)
		}
	case <-time.After(2 * time.Second):
		t.Fatal("timed out waiting for replayed command")
	}
}

func TestDispatch_ConnectedRobot_DeliversLive(t *testing.T) {
	s := New()
	ctx := context.Background()

	streamCtx, cancel := context.WithCancel(ctx)
	defer cancel()
	fs := newFakeStream(streamCtx)
	go func() {
		_ = s.StreamCommands(&telemetryv1.StreamCommandsRequest{RobotId: "robot-1"}, fs)
	}()

	// Give StreamCommands a moment to register its subscriber channel.
	time.Sleep(50 * time.Millisecond)

	res, err := s.Dispatch(ctx, "robot-1", robotdomain.Command{Type: robotdomain.CommandEmergencyStop})
	if err != nil {
		t.Fatalf("Dispatch: %v", err)
	}

	select {
	case cmd := <-fs.out:
		if cmd.CommandId != res.CommandID {
			t.Fatalf("delivered command id = %q, want %q", cmd.CommandId, res.CommandID)
		}
		if cmd.GetEmergencyStop() == nil {
			t.Fatalf("expected EmergencyStop payload, got %#v", cmd.Payload)
		}
	case <-time.After(2 * time.Second):
		t.Fatal("timed out waiting for live command")
	}
}

func TestAckCommand_TrimsBacklog(t *testing.T) {
	s := New()
	ctx := context.Background()

	first, err := s.Dispatch(ctx, "robot-1", robotdomain.Command{Type: robotdomain.CommandStartRace})
	if err != nil {
		t.Fatalf("Dispatch: %v", err)
	}
	second, err := s.Dispatch(ctx, "robot-1", robotdomain.Command{Type: robotdomain.CommandStopRace})
	if err != nil {
		t.Fatalf("Dispatch: %v", err)
	}

	if _, err := s.AckCommand(ctx, &telemetryv1.AckCommandRequest{
		RobotId:   "robot-1",
		CommandId: first.CommandID,
		Status:    telemetryv1.CommandExecutionStatus_COMMAND_EXECUTION_STATUS_ACCEPTED,
	}); err != nil {
		t.Fatalf("AckCommand: %v", err)
	}

	s.mu.Lock()
	backlog := s.backlog["robot-1"]
	s.mu.Unlock()

	if len(backlog) != 1 || backlog[0].CommandId != second.CommandID {
		t.Fatalf("backlog after ack = %+v, want only %q", backlog, second.CommandID)
	}
}

func TestDispatch_DisableCommandChannel(t *testing.T) {
	s := New()
	ctx := context.Background()

	streamCtx, cancel := context.WithCancel(ctx)
	defer cancel()
	fs := newFakeStream(streamCtx)
	go func() {
		_ = s.StreamCommands(&telemetryv1.StreamCommandsRequest{RobotId: "robot-1"}, fs)
	}()

	// Give StreamCommands a moment to register its subscriber channel.
	time.Sleep(50 * time.Millisecond)

	res, err := s.Dispatch(ctx, "robot-1", robotdomain.Command{Type: robotdomain.CommandDisableCommandChannel})
	if err != nil {
		t.Fatalf("Dispatch: %v", err)
	}

	select {
	case cmd := <-fs.out:
		if cmd.CommandId != res.CommandID {
			t.Fatalf("delivered command id = %q, want %q", cmd.CommandId, res.CommandID)
		}
		if cmd.GetDisableCommandChannel() == nil {
			t.Fatalf("expected DisableCommandChannel payload, got %#v", cmd.Payload)
		}
	case <-time.After(2 * time.Second):
		t.Fatal("timed out waiting for live command")
	}
}

func TestDispatch_SetTelemetryChannel(t *testing.T) {
	s := New()
	ctx := context.Background()

	streamCtx, cancel := context.WithCancel(ctx)
	defer cancel()
	fs := newFakeStream(streamCtx)
	go func() {
		_ = s.StreamCommands(&telemetryv1.StreamCommandsRequest{RobotId: "robot-1"}, fs)
	}()

	// Give StreamCommands a moment to register its subscriber channel.
	time.Sleep(50 * time.Millisecond)

	res, err := s.Dispatch(ctx, "robot-1", robotdomain.Command{
		Type:       robotdomain.CommandSetTelemetryChannel,
		Parameters: map[string]any{"enabled": true},
	})
	if err != nil {
		t.Fatalf("Dispatch: %v", err)
	}

	select {
	case cmd := <-fs.out:
		if cmd.CommandId != res.CommandID {
			t.Fatalf("delivered command id = %q, want %q", cmd.CommandId, res.CommandID)
		}
		if got := cmd.GetSetTelemetryChannel().GetEnabled(); !got {
			t.Fatalf("SetTelemetryChannel.Enabled = %v, want true", got)
		}
	case <-time.After(2 * time.Second):
		t.Fatal("timed out waiting for live command")
	}
}

func TestDispatch_UnsupportedCommandType_Errors(t *testing.T) {
	s := New()
	if _, err := s.Dispatch(context.Background(), "robot-1", robotdomain.Command{Type: "NOT_A_REAL_COMMAND"}); err == nil {
		t.Fatal("expected error for unsupported command type")
	}
}
