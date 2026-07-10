package vision

import (
	"context"
	"time"

	telemetryv1 "github.com/teamvoldemor/voldemorbot/platform/backend/gen/telemetry/v1"
)

// TelemetryReader is the minimal read port into the Telemetry context that
// vision detections are sourced from: this context does not own detection
// state itself, since the robot's YOLO pipeline reports through the
// telemetry ingest path, not directly to Vision.
type TelemetryReader interface {
	Latest() *telemetryv1.RobotSnapshot
	History(limit int) []*telemetryv1.RobotSnapshot
}

// Service is the Vision bounded context's port.
type Service interface {
	ListDetections(ctx context.Context, limit int, confidenceMin *float32) ([]Detection, error)
	CurrentDetections(ctx context.Context) (*DetectionFrame, error)

	ListAnnotations(ctx context.Context) ([]Annotation, error)
	CreateAnnotation(ctx context.Context, req CreateAnnotationRequest) (Annotation, error)
	UpdateAnnotation(ctx context.Context, id string, req UpdateAnnotationRequest) (Annotation, error)
	DeleteAnnotation(ctx context.Context, id string) error

	ActiveModel(ctx context.Context) (ModelInfo, error)
	SetActiveModel(ctx context.Context, req SetModelRequest) (ModelInfo, error)

	PipelineStatus(ctx context.Context) (PipelineStatus, error)
}

// service is the concrete implementation of Service.
type service struct {
	store Store
	tel   TelemetryReader
}

// NewService constructs a Service backed by the given Store and TelemetryReader.
func NewService(store Store, tel TelemetryReader) Service {
	return &service{store: store, tel: tel}
}

func (s *service) ListDetections(_ context.Context, limit int, confidenceMin *float32) ([]Detection, error) {
	if limit <= 0 {
		limit = 1
	}
	snaps := s.tel.History(limit)

	out := make([]Detection, 0, limit)
	for _, snap := range snaps {
		ts := snap.GetTimestamp().AsTime()
		for _, d := range snap.GetVisionDetections() {
			det := fromProtoDetection(d, &ts)
			if confidenceMin != nil && det.Confidence < *confidenceMin {
				continue
			}
			out = append(out, det)
		}
	}
	return out, nil
}

func (s *service) CurrentDetections(_ context.Context) (*DetectionFrame, error) {
	snap := s.tel.Latest()
	if snap == nil {
		return nil, nil //nolint:nilnil // "no snapshot yet" is a valid, distinct outcome from an error
	}
	ts := snap.GetTimestamp().AsTime()
	dets := make([]Detection, 0, len(snap.GetVisionDetections()))
	for _, d := range snap.GetVisionDetections() {
		dets = append(dets, fromProtoDetection(d, &ts))
	}
	return &DetectionFrame{
		Timestamp:  ts,
		Detections: dets,
	}, nil
}

func fromProtoDetection(d *telemetryv1.Detection, ts *time.Time) Detection {
	return Detection{
		ClassName:  d.GetClassName(),
		Confidence: float32(d.GetConfidence()),
		Bbox: BoundingBox{
			X:      d.GetBboxX(),
			Y:      d.GetBboxY(),
			Width:  d.GetBboxW(),
			Height: d.GetBboxH(),
		},
		Timestamp: ts,
	}
}

func (s *service) ListAnnotations(ctx context.Context) ([]Annotation, error) {
	return s.store.ListAnnotations(ctx)
}

func (s *service) CreateAnnotation(ctx context.Context, req CreateAnnotationRequest) (Annotation, error) {
	return s.store.CreateAnnotation(ctx, req)
}

func (s *service) UpdateAnnotation(ctx context.Context, id string, req UpdateAnnotationRequest) (Annotation, error) {
	return s.store.UpdateAnnotation(ctx, id, req)
}

func (s *service) DeleteAnnotation(ctx context.Context, id string) error {
	return s.store.DeleteAnnotation(ctx, id)
}

func (s *service) ActiveModel(ctx context.Context) (ModelInfo, error) {
	return s.store.ActiveModel(ctx)
}

func (s *service) SetActiveModel(ctx context.Context, req SetModelRequest) (ModelInfo, error) {
	return s.store.SetActiveModel(ctx, req)
}

func (s *service) PipelineStatus(ctx context.Context) (PipelineStatus, error) {
	return s.store.PipelineStatus(ctx)
}
