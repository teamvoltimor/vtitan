package robotcmd_test

import (
	"context"
	"io"
	"log/slog"
	"net"
	"sync"
	"testing"
	"time"

	"google.golang.org/grpc"
	"google.golang.org/grpc/credentials/insecure"
	"google.golang.org/grpc/test/bufconn"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/button"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/statemachine/command"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/statemachine/robotcmd"

	telemetryv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/telemetry/v1"
)

// fakeServer is a scripted RobotCommandService: it streams a fixed sequence
// of commands to the first StreamCommands caller, then blocks until ctx is
// done, and records every AckCommand it receives.
type fakeServer struct {
	telemetryv1.UnimplementedRobotCommandServiceServer

	commands []*telemetryv1.RobotCommand

	mu   sync.Mutex
	acks []*telemetryv1.AckCommandRequest
}

func (s *fakeServer) StreamCommands(
	req *telemetryv1.StreamCommandsRequest,
	stream telemetryv1.RobotCommandService_StreamCommandsServer,
) error {
	for _, cmd := range s.commands {
		if err := stream.Send(cmd); err != nil {
			return err //nolint:wrapcheck // test fake, error path only exercised by test failures
		}
	}
	<-stream.Context().Done()
	return stream.Context().Err()
}

func (s *fakeServer) AckCommand(
	_ context.Context, req *telemetryv1.AckCommandRequest,
) (*telemetryv1.AckCommandResponse, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.acks = append(s.acks, req)
	return &telemetryv1.AckCommandResponse{}, nil
}

func (s *fakeServer) recordedAcks() []*telemetryv1.AckCommandRequest {
	s.mu.Lock()
	defer s.mu.Unlock()
	return append([]*telemetryv1.AckCommandRequest(nil), s.acks...)
}

// fakeButtonSink records every synthetic button event PublishButtonEvent
// receives.
type fakeButtonSink struct {
	mu     sync.Mutex
	events []string
}

func (s *fakeButtonSink) PublishButtonEvent(kind button.Kind) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.events = append(s.events, kind.String())
	return nil
}

func (s *fakeButtonSink) recorded() []string {
	s.mu.Lock()
	defer s.mu.Unlock()
	return append([]string(nil), s.events...)
}

func dialBufconn(t *testing.T, srv *fakeServer) (*grpc.ClientConn, func()) {
	t.Helper()

	lis := bufconn.Listen(1024 * 1024)
	grpcServer := grpc.NewServer()
	telemetryv1.RegisterRobotCommandServiceServer(grpcServer, srv)
	go func() { _ = grpcServer.Serve(lis) }()

	conn, err := grpc.NewClient("passthrough:///bufconn",
		grpc.WithContextDialer(func(ctx context.Context, _ string) (net.Conn, error) {
			return lis.DialContext(ctx)
		}),
		grpc.WithTransportCredentials(insecure.NewCredentials()),
	)
	if err != nil {
		t.Fatalf("dialing bufconn: %v", err)
	}

	return conn, func() {
		_ = conn.Close()
		grpcServer.Stop()
	}
}

func TestClient_Run_DispatchesAndAcks(t *testing.T) {
	t.Parallel()

	srv := &fakeServer{commands: []*telemetryv1.RobotCommand{
		{CommandId: "cmd-1", Payload: &telemetryv1.RobotCommand_StartRace{StartRace: &telemetryv1.StartRaceParams{}}},
		{CommandId: "cmd-2", Payload: &telemetryv1.RobotCommand_StopRace{StopRace: &telemetryv1.StopRaceParams{}}},
	}}
	conn, cleanup := dialBufconn(t, srv)
	defer cleanup()

	logger := slog.New(slog.NewTextHandler(io.Discard, nil))
	client := robotcmd.NewFromConn(conn, robotcmd.Config{RobotID: "robot-1"}, logger)

	buttonSink := &fakeButtonSink{}
	dispatcher := command.NewDispatcher(buttonSink, fakeChannelSink{})

	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()

	runErr := make(chan error, 1)
	go func() { runErr <- client.Run(ctx, dispatcher) }()

	// Give the stream time to deliver both commands and their acks.
	time.Sleep(300 * time.Millisecond)
	cancel()
	if err := <-runErr; err != nil {
		t.Fatalf("Run: %v", err)
	}

	acks := srv.recordedAcks()
	if len(acks) != 2 {
		t.Fatalf("got %d acks, want 2: %+v", len(acks), acks)
	}
	if acks[0].GetCommandId() != "cmd-1" || acks[1].GetCommandId() != "cmd-2" {
		t.Errorf("ack command IDs = %q, %q, want cmd-1, cmd-2", acks[0].GetCommandId(), acks[1].GetCommandId())
	}
	for _, ack := range acks {
		if ack.GetStatus() == telemetryv1.CommandExecutionStatus_COMMAND_EXECUTION_STATUS_UNSPECIFIED {
			t.Errorf("ack for %s has UNSPECIFIED status", ack.GetCommandId())
		}
	}

	wantEvents := []string{button.KindShortPress.String(), button.KindLongPress.String()}
	if events := buttonSink.recorded(); len(events) != 2 || events[0] != wantEvents[0] || events[1] != wantEvents[1] {
		t.Errorf("button events = %v, want %v", events, wantEvents)
	}
}

// fakeChannelSink satisfies command.ChannelSink without touching any real
// backend administrative channel.
type fakeChannelSink struct{}

func (fakeChannelSink) SetVisionDebug(_ command.VisionDebugParams) error { return nil }
func (fakeChannelSink) ToggleTelemetryChannel(_ bool) error              { return nil }
func (fakeChannelSink) DisableCommandChannel() error                     { return nil }
