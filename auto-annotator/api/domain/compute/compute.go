// Package compute is the Go client boundary to the Python ML workers.
package compute

import (
	"context"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/job"
)

type (
	// Point is a normalized 2-D coordinate.
	Point struct {
		X float64
		Y float64
	}

	// ClickPoint is a SAM prompt point.
	ClickPoint struct {
		PointType string
		ClassName string
		X         float64
		Y         float64
	}

	// Shape is an inference result polygon/bbox.
	Shape struct {
		ID        string
		ClassName string
		Points    []Point
	}

	// SegmentInput is a stateless SAM segmentation request passed to Clients.
	SegmentInput struct {
		ImagePath  string
		Points     []ClickPoint
		ClassNames []string
		ImageID    int64
	}

	// SegmentResult mirrors the SegmentResponse envelope.
	SegmentResult struct {
		State   string
		Message string
		Shapes  []Shape
	}

	// AugmentSource is one image to augment.
	AugmentSource struct {
		Path       string
		FormatUsed string
		ImageID    int64
	}

	// AugmentInput is an augmentation job request passed to Clients.
	AugmentInput struct {
		Sources          []AugmentSource
		NumAugmentations int32
	}

	// TrainInput is a training job request passed to Clients.
	TrainInput struct {
		ModelName    string
		DataYamlPath string
		Epochs       int32
		Batch        int32
		Imgsz        int32
	}

	// AugmentedImage is a file the worker produced for the API to persist.
	AugmentedImage struct {
		Path       string
		FormatUsed string
		ParentID   int64
	}

	// Progress is one streamed job event from the worker.
	Progress struct {
		Augmented *AugmentedImage
		Status    string
		Message   string
		Stage     string
		Details   string
		Error     string
		Progress  float64
		Finished  bool
	}

	// Model is a configured ML model entry.
	Model struct {
		ID           string
		Label        string
		ModelType    string
		Active       bool
		Available    bool
		SupportsText bool
	}

	// SegmentReq is the service-level segmentation request (image resolved, coords float64).
	SegmentReq struct {
		Points  []ClickPoint
		ImageID int64
	}

	// AugmentJobReq is the service-level augmentation job request.
	AugmentJobReq struct {
		ImageIDs         []int64
		NumAugmentations int32
	}

	// TrainJobReq is the service-level training job request.
	TrainJobReq struct {
		ModelName string
		Epochs    int32
		Batch     int32
		Imgsz     int32
	}
)

// State enumerates the compute ready/error states.
type State string

const (
	StateReady State = "ready"
	StateError State = "error"
)

func (s State) String() string { return string(s) }

// Clients is the low-level compute boundary for gRPC workers.
type Clients interface {
	Segment(ctx context.Context, in SegmentInput) (SegmentResult, error)
	RunAugmentation(ctx context.Context, in AugmentInput, onProgress func(Progress)) error
	RunTraining(ctx context.Context, in TrainInput, onProgress func(Progress)) error
	Close() error
}

// Store is the data boundary for compute operations.
type Store interface {
	GetImage(ctx context.Context, id int64) (ImageRef, error)
	ListClassNames(ctx context.Context) ([]string, error)
	InsertAugmented(ctx context.Context, img AugmentedImage) error
}

// ImageRef is the minimal image info needed by the compute service.
type ImageRef struct {
	Path       string
	FormatUsed string
	ID         int64
}

// Service is the compute domain service interface.
type Service interface {
	ListModels() ([]Model, error)
	Segment(ctx context.Context, req SegmentReq) (SegmentResult, error)
	StartAugment(ctx context.Context, req AugmentJobReq) error
	StartTrain(ctx context.Context, req TrainJobReq) error
	JobRunning() bool
	Subscribe() (string, <-chan job.Event, bool)
}
