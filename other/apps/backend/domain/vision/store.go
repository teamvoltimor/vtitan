package vision

import "context"

// Store is the storage port for the Vision bounded context's own state
// (annotations, active model, pipeline status). Detections are read from
// TelemetryReader instead, since this context doesn't own that data.
type Store interface {
	ListAnnotations(ctx context.Context) ([]Annotation, error)
	CreateAnnotation(ctx context.Context, req CreateAnnotationRequest) (Annotation, error)
	UpdateAnnotation(ctx context.Context, id string, req UpdateAnnotationRequest) (Annotation, error)
	DeleteAnnotation(ctx context.Context, id string) error

	ActiveModel(ctx context.Context) (ModelInfo, error)
	SetActiveModel(ctx context.Context, req SetModelRequest) (ModelInfo, error)

	PipelineStatus(ctx context.Context) (PipelineStatus, error)
}
