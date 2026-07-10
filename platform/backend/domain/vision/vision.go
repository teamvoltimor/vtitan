// Package vision is the Vision bounded context: traffic-sign detections
// (sourced from the live telemetry feed, since the robot's YOLO pipeline
// reports through the telemetry ingest path, not directly to this context),
// ground-truth annotation management, and inference model lifecycle.
package vision

import (
	"errors"
	"time"
)

// ErrNotFound is returned when an annotation does not exist.
var ErrNotFound = errors.New("vision resource not found")

// BoundingBox is a normalized [0,1] detection/annotation bounding box.
type BoundingBox struct {
	X      float64
	Y      float64
	Width  float64
	Height float64
}

// Detection is a single traffic-sign detection.
type Detection struct {
	ID         *string
	ClassName  string
	Confidence float32
	Bbox       BoundingBox
	Timestamp  *time.Time
}

// DetectionFrame is one frame's worth of detections.
type DetectionFrame struct {
	Timestamp       time.Time
	Detections      []Detection
	InferenceTimeMs float32
	ImageWidth      *int
	ImageHeight     *int
	NPUFps          *float32
}

// Annotation is a ground-truth bounding-box label for a stored image.
type Annotation struct {
	ID        string
	ClassName string
	Bbox      BoundingBox
	ImageID   string
	Metadata  map[string]any
	CreatedAt time.Time
	UpdatedAt *time.Time
}

// CreateAnnotationRequest is the payload for creating an annotation.
type CreateAnnotationRequest struct {
	ClassName string
	Bbox      BoundingBox
	ImageID   string
	Metadata  map[string]any
}

// UpdateAnnotationRequest is the payload for updating an annotation. Nil
// fields are left unchanged.
type UpdateAnnotationRequest struct {
	ClassName *string
	Bbox      *BoundingBox
}

// ModelInfo describes the active inference model.
type ModelInfo struct {
	Name                string
	Version             string
	InputShape          []int
	Classes             []string
	ConfidenceThreshold *float32
	IOUThreshold        *float32
}

// SetModelRequest is the payload for switching the active inference model.
type SetModelRequest struct {
	Name    string
	Version *string
}

// PipelineStatus is the vision inference pipeline's current health.
type PipelineStatus struct {
	Active          bool
	ModelLoaded     bool
	ModelName       *string
	FPS             float32
	FramesProcessed *int
	LastInference   *time.Time
	Error           *string
}
