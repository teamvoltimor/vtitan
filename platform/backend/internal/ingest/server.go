package ingest

import (
	"errors"
	"io"

	"go.uber.org/zap"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"

	telemetryv1 "github.com/teamvoltimor/vtitan/platform/backend/gen/telemetry/v1"
	"github.com/teamvoltimor/vtitan/platform/backend/domain/session"
	"github.com/teamvoltimor/vtitan/platform/backend/domain/telemetry"
)

// Server is the gRPC ingest adapter. It translates incoming streams into
// service calls; all business logic lives in the domain services.
type Server struct {
	telemetryv1.UnimplementedTelemetryIngestServiceServer
	telSvc  telemetry.TelemetryService
	sessSvc session.SessionService
	log     *zap.Logger
}

// New returns a Server that writes every received frame to the telemetry
// service and records it via the session service.
func New(telSvc telemetry.TelemetryService, sessSvc session.SessionService, log *zap.Logger) *Server {
	return &Server{telSvc: telSvc, sessSvc: sessSvc, log: log}
}

// StreamSnapshots receives a client-stream of robot snapshots and writes each
// to the store. The summary response is sent on clean stream close.
func (s *Server) StreamSnapshots(stream telemetryv1.TelemetryIngestService_StreamSnapshotsServer) error {
	var count uint64
	for {
		req, err := stream.Recv()
		if errors.Is(err, io.EOF) {
			s.log.Info("snapshot stream closed", zap.Uint64("received", count))
			return stream.SendAndClose(&telemetryv1.StreamSnapshotsResponse{SnapshotsReceived: count})
		}
		if err != nil {
			s.log.Warn("snapshot stream error", zap.Error(err))
			return status.Errorf(codes.Internal, "recv: %v", err)
		}
		if req.Snapshot == nil {
			return status.Error(codes.InvalidArgument, "snapshot must not be nil")
		}
		s.telSvc.Write(req.Snapshot)
		if err := s.sessSvc.Record(stream.Context(), req.Snapshot); err != nil {
			s.log.Warn("record snapshot", zap.Error(err))
		}
		count++
	}
}

// StreamTopics receives a client-stream of topic snapshots and updates the
// store's latest topic state on each frame.
func (s *Server) StreamTopics(stream telemetryv1.TelemetryIngestService_StreamTopicsServer) error {
	var count uint64
	for {
		req, err := stream.Recv()
		if errors.Is(err, io.EOF) {
			s.log.Info("topics stream closed", zap.Uint64("received", count))
			return stream.SendAndClose(&telemetryv1.StreamTopicsResponse{UpdatesReceived: count})
		}
		if err != nil {
			s.log.Warn("topics stream error", zap.Error(err))
			return status.Errorf(codes.Internal, "recv: %v", err)
		}
		if req.Topics == nil {
			return status.Error(codes.InvalidArgument, "topics must not be nil")
		}
		s.telSvc.WriteTopics(req.Topics)
		count++
	}
}

