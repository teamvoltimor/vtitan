// Package compute is the Go client boundary to the Python ML workers. Handlers
// depend on the Clients interface (framework- and protobuf-agnostic) so they can
// be tested with fakes; the gRPC implementation lives in grpc.go.
package compute

import "context"

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

	// SegmentInput is a stateless SAM segmentation request.
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

	// AugmentSource is one image to augment (resolved from the DB by the API).
	AugmentSource struct {
		Path       string
		FormatUsed string
		ImageID    int64
	}

	// AugmentInput is an augmentation job request.
	AugmentInput struct {
		Sources          []AugmentSource
		NumAugmentations int32
	}

	// TrainInput is a training job request.
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

	// Progress is one streamed job event.
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
)

// Clients is the compute boundary. Streaming RPCs deliver each Progress via the
// onProgress callback and return a non-nil error on stream failure.
type Clients interface {
	Segment(ctx context.Context, in SegmentInput) (SegmentResult, error)
	RunAugmentation(ctx context.Context, in AugmentInput, onProgress func(Progress)) error
	RunTraining(ctx context.Context, in TrainInput, onProgress func(Progress)) error
	Close() error
}
