package vision

import (
	"github.com/google/uuid"

	domain "github.com/teamvoltimor/vtitan/apps/backend/domain/vision"
)

func parseUUID(s string) uuid.UUID {
	id, err := uuid.Parse(s)
	if err != nil {
		return uuid.Nil
	}
	return id
}

func toWireBbox(b domain.BoundingBox) BoundingBox {
	return BoundingBox{X: b.X, Y: b.Y, Width: b.Width, Height: b.Height}
}

func fromWireBbox(b BoundingBox) domain.BoundingBox {
	return domain.BoundingBox{X: b.X, Y: b.Y, Width: b.Width, Height: b.Height}
}

func toWireDetection(d domain.Detection) Detection {
	out := Detection{
		ClassName:  DetectionClassName(d.ClassName),
		Confidence: d.Confidence,
		Bbox:       toWireBbox(d.Bbox),
		Timestamp:  d.Timestamp,
	}
	if d.ID != nil {
		id := parseUUID(*d.ID)
		out.Id = &id
	}
	return out
}

func toWireDetections(ds []domain.Detection) []Detection {
	out := make([]Detection, len(ds))
	for i, d := range ds {
		out[i] = toWireDetection(d)
	}
	return out
}

func toWireDetectionFrame(f *domain.DetectionFrame) DetectionFrame {
	return DetectionFrame{
		Timestamp:       f.Timestamp,
		Detections:      toWireDetections(f.Detections),
		InferenceTimeMs: f.InferenceTimeMs,
		ImageWidth:      f.ImageWidth,
		ImageHeight:     f.ImageHeight,
		NpuFps:          f.NPUFps,
	}
}

func toWireAnnotation(a domain.Annotation) Annotation {
	out := Annotation{
		Id:        parseUUID(a.ID),
		ClassName: a.ClassName,
		Bbox:      toWireBbox(a.Bbox),
		ImageId:   parseUUID(a.ImageID),
		CreatedAt: a.CreatedAt,
		UpdatedAt: a.UpdatedAt,
	}
	if a.Metadata != nil {
		out.Metadata = &a.Metadata
	}
	return out
}

func toWireAnnotations(as []domain.Annotation) []Annotation {
	out := make([]Annotation, len(as))
	for i, a := range as {
		out[i] = toWireAnnotation(a)
	}
	return out
}

func toWireModelInfo(m domain.ModelInfo) ModelInfo {
	return ModelInfo{
		Name:                m.Name,
		Version:             m.Version,
		InputShape:          m.InputShape,
		Classes:             m.Classes,
		ConfidenceThreshold: m.ConfidenceThreshold,
		IouThreshold:        m.IOUThreshold,
	}
}

func toWirePipelineStatus(p domain.PipelineStatus) PipelineStatus {
	return PipelineStatus{
		Active:          p.Active,
		ModelLoaded:     p.ModelLoaded,
		ModelName:       p.ModelName,
		Fps:             p.FPS,
		FramesProcessed: p.FramesProcessed,
		LastInference:   p.LastInference,
		Error:           p.Error,
	}
}

func fromCreateAnnotationRequest(req CreateAnnotationRequest) domain.CreateAnnotationRequest {
	out := domain.CreateAnnotationRequest{
		ClassName: req.ClassName,
		Bbox:      fromWireBbox(req.Bbox),
		ImageID:   req.ImageId.String(),
	}
	if req.Metadata != nil {
		out.Metadata = *req.Metadata
	}
	return out
}

func fromUpdateAnnotationRequest(req UpdateAnnotationRequest) domain.UpdateAnnotationRequest {
	out := domain.UpdateAnnotationRequest{ClassName: req.ClassName}
	if req.Bbox != nil {
		bb := fromWireBbox(*req.Bbox)
		out.Bbox = &bb
	}
	return out
}

func fromSetModelRequest(req SetModelRequest) domain.SetModelRequest {
	return domain.SetModelRequest{Name: req.Name, Version: req.Version}
}
