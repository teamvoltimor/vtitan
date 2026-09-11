package robotcmd_test

import (
	"context"
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

// fakeButtonSink records every synthetic button event PublishButtonEvent
// receives.
type fakeButtonSink struct {
	mu     sync.Mutex
	events []string
}

// fakeChannelSink satisfies command.ChannelSink without touching any real
// backend administrative channel.
type fakeChannelSink struct{}

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
	if err := stream.Context().Err(); err != nil {
		return err //nolint:wrapcheck // test fake, just relaying ctx cancellation to the RPC caller
	}
	return nil
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

func (fakeChannelSink) SetVisionDebug(_ command.VisionDebugParams) error { return nil }
func (fakeChannelSink) ToggleTelemetryChannel(_ bool) error              { return nil }
func (fakeChannelSink) DisableCommandChannel() error                     { return nil }

// dialBufconn starts srv on an in-memory bufconn listener and returns a
// client connection to it, plus a cleanup func the caller must defer.
func dialBufconn(t *testing.T, srv *fakeServer) (conn *grpc.ClientConn, cleanup func()) {
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

	logger := slog.New(slog.DiscardHandler)
	client := robotcmd.NewFromConn(conn, robotcmd.Config{RobotID: "robot-1"}, logger)

	buttonSink := &fakeButtonSink{}
	dispatcher := command.NewDispatcher(buttonSink, fakeChannelSink{})

	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()

	runErr := make(chan error, 1)
	go func() { runErr <- client.Run(ctx, dispatcher) }()

	// Wait for the actual final condition -- both commands dispatched, acked,
	// AND their button events published -- instead of assuming a fixed delay.
	// The dispatch pipeline is synchronous (Dispatch publishes the button
	// event, then Run acks), so observing 2 acks also guarantees 2 button
	// events; polling both closes the race that the fixed 300ms sleep left
	// open under CI load.
	deadline := time.Now().Add(2 * time.Second)
	for {
		if len(srv.recordedAcks()) == 2 && len(buttonSink.recorded()) == 2 {
			break
		}
		if time.Now().After(deadline) {
			t.Fatalf("timed out waiting for dispatch+ack: acks=%v events=%v",
				srv.recordedAcks(), buttonSink.recorded())
		}
		time.Sleep(10 * time.Millisecond)
	}

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
