package ingest

import (
	"context"
	"errors"
	"io"

	"go.uber.org/zap"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"

	telemetryv1 "github.com/klevor/telemetry-backend/gen/telemetry/v1"
)

type (
	// Store is the write side of the memory store.
	Store interface {
		Write(snap *telemetryv1.RobotSnapshot)
		WriteTopics(topics *telemetryv1.TopicsSnapshot)
	}

	// Recorder persists snapshot frames to durable storage.
	Recorder interface {
		Record(ctx context.Context, snap *telemetryv1.RobotSnapshot) error
	}
)

// Server is the gRPC ingest adapter. It translates incoming streams into
// store writes; all business logic lives in the store.
type Server struct {
	telemetryv1.UnimplementedTelemetryIngestServiceServer
	store    Store
	recorder Recorder
	log      *zap.Logger
}

func New(store Store, recorder Recorder, log *zap.Logger) *Server {
	return &Server{store: store, recorder: recorder, log: log}
}

// StreamSnapshots receives a client-stream of robot snapshots and writes each
// to the store. The summary response is sent on clean stream close.
func (s *Server) StreamSnapshots(stream telemetryv1.TelemetryIngestService_StreamSnapshotsServer) error {
	var count uint64
	for {
		req, err := stream.Recv()
		if errors.Is(err, io.EOF) {
			s.log.Info("snapshot stream closed", zap.Uint64("received", count))
			return stream.SendAndClose(&telemetryv1.IngestSnapshotResponse{SnapshotsReceived: count})
		}
		if err != nil {
			s.log.Warn("snapshot stream error", zap.Error(err))
			return status.Errorf(codes.Internal, "recv: %v", err)
		}
		if req.Snapshot == nil {
			return status.Error(codes.InvalidArgument, "snapshot must not be nil")
		}
		s.store.Write(req.Snapshot)
		if err := s.recorder.Record(stream.Context(), req.Snapshot); err != nil {
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
			return stream.SendAndClose(&telemetryv1.IngestTopicsResponse{UpdatesReceived: count})
		}
		if err != nil {
			s.log.Warn("topics stream error", zap.Error(err))
			return status.Errorf(codes.Internal, "recv: %v", err)
		}
		if req.Topics == nil {
			return status.Error(codes.InvalidArgument, "topics must not be nil")
		}
		s.store.WriteTopics(req.Topics)
		count++
	}
}
