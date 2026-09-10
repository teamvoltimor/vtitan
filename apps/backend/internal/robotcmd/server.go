// Package robotcmd is the gRPC adapter for RobotCommandService. It bridges
// domain/robot.Service.Command to the robot's live StreamCommands connection:
// Dispatch enqueues a command for the robot identified by robotID, and
// StreamCommands is the long-lived call the robot opens to receive them.
//
// The channel is best-effort: if no robot is currently connected, Dispatch
// still succeeds (the command is queued in a bounded per-robot backlog and
// replayed on the robot's next connect/reconnect via last_command_id).
package robotcmd

import (
	"context"
	"fmt"
	"log/slog"
	"sync"

	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
	"google.golang.org/protobuf/types/known/timestamppb"

	"github.com/google/uuid"

	robotdomain "github.com/teamvoltimor/vtitan/platform/backend/domain/robot"
	telemetryv1 "github.com/teamvoltimor/vtitan/platform/backend/gen/telemetry/v1"
)

const (
	// streamBufferSize bounds how many commands can be in-flight to a
	// connected robot before Dispatch falls back to backlog-only delivery.
	streamBufferSize = 16
	// maxBacklog bounds how many undelivered/unacked commands are retained
	// per robot while it is disconnected. Oldest entries are dropped first.
	maxBacklog = 64
)

// Server implements telemetryv1.RobotCommandServiceServer and
// domain/robot.Dispatcher over the same in-memory state.
type Server struct {
	telemetryv1.UnimplementedRobotCommandServiceServer

	mu      sync.Mutex
	subs    map[string]chan *telemetryv1.RobotCommand // robotID -> active stream's outbound channel
	backlog map[string][]*telemetryv1.RobotCommand    // robotID -> undelivered/unacked commands, oldest first
}

// New returns a Server with no robots connected.
func New() *Server {
	return &Server{
		subs:    make(map[string]chan *telemetryv1.RobotCommand),
		backlog: make(map[string][]*telemetryv1.RobotCommand),
	}
}

// Dispatch implements domain/robot.Dispatcher. It always succeeds for a
// known robot ID (existence is checked by the caller, domain/robot.Service);
// delivery itself is best-effort.
func (s *Server) Dispatch(_ context.Context, robotID string, cmd robotdomain.Command) (robotdomain.CommandResult, error) {
	pb, err := toProto(cmd)
	if err != nil {
		return robotdomain.CommandResult{}, err
	}
	pb.CommandId = uuid.NewString()
	pb.IssuedAt = timestamppb.Now()

	s.mu.Lock()
	s.backlog[robotID] = appendBacklog(s.backlog[robotID], pb)
	ch, connected := s.subs[robotID]
	s.mu.Unlock()

	if !connected {
		return robotdomain.CommandResult{
			CommandID: pb.CommandId,
			Status:    robotdomain.CommandQueued,
			Message:   "robot not connected; queued for delivery on reconnect",
		}, nil
	}

	select {
	case ch <- pb:
		return robotdomain.CommandResult{
			CommandID: pb.CommandId,
			Status:    robotdomain.CommandQueued,
			Message:   "delivered to robot's command stream",
		}, nil
	default:
		return robotdomain.CommandResult{
			CommandID: pb.CommandId,
			Status:    robotdomain.CommandQueued,
			Message:   "robot's command channel is full; queued for retry on reconnect",
		}, nil
	}
}

// StreamCommands is opened once by a robot and kept open for its lifetime.
// On connect it replays anything queued after LastCommandId, then blocks,
// pushing newly dispatched commands as they arrive.
func (s *Server) StreamCommands(req *telemetryv1.StreamCommandsRequest, stream telemetryv1.RobotCommandService_StreamCommandsServer) error {
	robotID := req.GetRobotId()
	ch := make(chan *telemetryv1.RobotCommand, streamBufferSize)

	s.mu.Lock()
	if old, ok := s.subs[robotID]; ok {
		close(old) // a newer connection for this robot supersedes the old one
	}
	s.subs[robotID] = ch
	replay := replayFrom(s.backlog[robotID], req.LastCommandId)
	s.mu.Unlock()

	slog.Info("robot command stream opened", "robot_id", robotID, "replay_count", len(replay))

	defer func() {
		s.mu.Lock()
		if s.subs[robotID] == ch {
			delete(s.subs, robotID)
		}
		s.mu.Unlock()
	}()

	for _, cmd := range replay {
		if err := stream.Send(cmd); err != nil {
			return status.Errorf(codes.Internal, "send replay: %v", err)
		}
	}

	for {
		select {
		case <-stream.Context().Done():
			return nil
		case cmd, ok := <-ch:
			if !ok {
				return nil // superseded by a newer stream for this robot
			}
			if err := stream.Send(cmd); err != nil {
				return status.Errorf(codes.Internal, "send: %v", err)
			}
		}
	}
}

// AckCommand records the robot's execution outcome for a previously streamed
// command and trims it (and anything older) from the retry backlog.
func (s *Server) AckCommand(_ context.Context, req *telemetryv1.AckCommandRequest) (*telemetryv1.AckCommandResponse, error) {
	slog.Info("robot command ack",
		"robot_id", req.GetRobotId(),
		"command_id", req.GetCommandId(),
		"status", req.GetStatus().String(),
		"message", req.GetMessage())

	s.mu.Lock()
	s.backlog[req.GetRobotId()] = trimBacklog(s.backlog[req.GetRobotId()], req.GetCommandId())
	s.mu.Unlock()

	return &telemetryv1.AckCommandResponse{}, nil
}

// appendBacklog appends cmd, evicting the oldest entry if over maxBacklog.
func appendBacklog(backlog []*telemetryv1.RobotCommand, cmd *telemetryv1.RobotCommand) []*telemetryv1.RobotCommand {
	backlog = append(backlog, cmd)
	if len(backlog) > maxBacklog {
		backlog = backlog[len(backlog)-maxBacklog:]
	}
	return backlog
}

// trimBacklog drops commandID and everything queued before it, since the
// robot has now fully processed them (accepted or rejected).
func trimBacklog(backlog []*telemetryv1.RobotCommand, commandID string) []*telemetryv1.RobotCommand {
	for i, cmd := range backlog {
		if cmd.GetCommandId() == commandID {
			return backlog[i+1:]
		}
	}
	return backlog
}

// replayFrom returns the commands queued strictly after lastCommandID. A nil
// or unmatched cursor (never connected before, or the entry aged out of the
// backlog) replays everything currently queued.
func replayFrom(backlog []*telemetryv1.RobotCommand, lastCommandID *string) []*telemetryv1.RobotCommand {
	if lastCommandID == nil || *lastCommandID == "" {
		return backlog
	}
	for i, cmd := range backlog {
		if cmd.GetCommandId() == *lastCommandID {
			return backlog[i+1:]
		}
	}
	return backlog
}

// toProto converts a domain Command into its wire RobotCommand, resolving
// the correct oneof payload for cmd.Type.
func toProto(cmd robotdomain.Command) (*telemetryv1.RobotCommand, error) {
	pb := &telemetryv1.RobotCommand{}
	switch cmd.Type {
	case robotdomain.CommandStartRace:
		params := &telemetryv1.StartRaceParams{}
		if v, ok := cmd.Parameters["mission_name"].(string); ok && v != "" {
			params.MissionName = &v
		}
		pb.Payload = &telemetryv1.RobotCommand_StartRace{StartRace: params}
	case robotdomain.CommandStopRace:
		pb.Payload = &telemetryv1.RobotCommand_StopRace{StopRace: &telemetryv1.StopRaceParams{}}
	case robotdomain.CommandPause:
		pb.Payload = &telemetryv1.RobotCommand_Pause{Pause: &telemetryv1.PauseParams{}}
	case robotdomain.CommandResume:
		pb.Payload = &telemetryv1.RobotCommand_Resume{Resume: &telemetryv1.ResumeParams{}}
	case robotdomain.CommandEmergencyStop:
		pb.Payload = &telemetryv1.RobotCommand_EmergencyStop{EmergencyStop: &telemetryv1.EmergencyStopParams{}}
	case robotdomain.CommandReturnToStart:
		pb.Payload = &telemetryv1.RobotCommand_ReturnToStart{ReturnToStart: &telemetryv1.ReturnToStartParams{}}
	case robotdomain.CommandReboot:
		pb.Payload = &telemetryv1.RobotCommand_Reboot{Reboot: &telemetryv1.RebootParams{}}
	case robotdomain.CommandShutdown:
		pb.Payload = &telemetryv1.RobotCommand_Shutdown{Shutdown: &telemetryv1.ShutdownParams{}}
	case robotdomain.CommandSetVisionDebug:
		params := &telemetryv1.SetVisionDebugParams{}
		if v, ok := cmd.Parameters["enabled"].(bool); ok {
			params.Enabled = v
		}
		if v, ok := cmd.Parameters["stream_fps"].(uint32); ok {
			params.StreamFps = &v
		}
		pb.Payload = &telemetryv1.RobotCommand_SetVisionDebug{SetVisionDebug: params}
	case robotdomain.CommandDisableCommandChannel:
		pb.Payload = &telemetryv1.RobotCommand_DisableCommandChannel{DisableCommandChannel: &telemetryv1.DisableCommandChannelParams{}}
	case robotdomain.CommandSetTelemetryChannel:
		params := &telemetryv1.SetTelemetryChannelParams{}
		if v, ok := cmd.Parameters["enabled"].(bool); ok {
			params.Enabled = v
		}
		pb.Payload = &telemetryv1.RobotCommand_SetTelemetryChannel{SetTelemetryChannel: params}
	default:
		return nil, fmt.Errorf("robotcmd: unsupported command type %q", cmd.Type)
	}
	return pb, nil
}
