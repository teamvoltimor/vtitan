package vision_test

import (
	"context"
	"errors"
	"testing"

	"github.com/teamvoldemor/voldemorbot/platform/backend/domain/vision"
	"github.com/teamvoldemor/voldemorbot/platform/backend/domain/vision/memory"
	telemetryv1 "github.com/teamvoldemor/voldemorbot/platform/backend/gen/telemetry/v1"
	"google.golang.org/protobuf/types/known/timestamppb"
)

// fakeTelemetry is a minimal vision.TelemetryReader stub so the Vision domain
// service can be tested without a real telemetry ingest pipeline.
type fakeTelemetry struct {
	latest  *telemetryv1.RobotSnapshot
	history []*telemetryv1.RobotSnapshot
}

func (f *fakeTelemetry) Latest() *telemetryv1.RobotSnapshot { return f.latest }
func (f *fakeTelemetry) History(limit int) []*telemetryv1.RobotSnapshot {
	if limit > 0 && limit < len(f.history) {
		return f.history[len(f.history)-limit:]
	}
	return f.history
}

func snapshotWithDetections(dets ...*telemetryv1.Detection) *telemetryv1.RobotSnapshot {
	return &telemetryv1.RobotSnapshot{
		Timestamp:        timestamppb.Now(),
		VisionDetections: dets,
	}
}

func TestCurrentDetectionsNilWhenNoSnapshot(t *testing.T) {
	svc := vision.NewService(memory.NewMemory(), &fakeTelemetry{})
	frame, err := svc.CurrentDetections(context.Background())
	if err != nil {
		t.Fatalf("CurrentDetections: %v", err)
	}
	if frame != nil {
		t.Fatalf("frame = %+v, want nil (no snapshot received yet)", frame)
	}
}

func TestCurrentDetectionsReflectsLatestSnapshot(t *testing.T) {
	snap := snapshotWithDetections(&telemetryv1.Detection{ClassName: "red_sign", Confidence: 0.9, BboxX: 0.1, BboxY: 0.2, BboxW: 0.3, BboxH: 0.4})
	svc := vision.NewService(memory.NewMemory(), &fakeTelemetry{latest: snap})

	frame, err := svc.CurrentDetections(context.Background())
	if err != nil {
		t.Fatalf("CurrentDetections: %v", err)
	}
	if frame == nil {
		t.Fatal("frame is nil, want a real frame")
	}
	if len(frame.Detections) != 1 {
		t.Fatalf("len(Detections) = %d, want 1", len(frame.Detections))
	}
	got := frame.Detections[0]
	if got.ClassName != "red_sign" || got.Confidence != 0.9 {
		t.Fatalf("Detections[0] = %+v, want class_name=red_sign confidence=0.9", got)
	}
	if got.Bbox.X != 0.1 || got.Bbox.Y != 0.2 || got.Bbox.Width != 0.3 || got.Bbox.Height != 0.4 {
		t.Fatalf("Detections[0].Bbox = %+v, want {0.1 0.2 0.3 0.4}", got.Bbox)
	}
}

func TestListDetectionsAppliesConfidenceFilter(t *testing.T) {
	snap := snapshotWithDetections(
		&telemetryv1.Detection{ClassName: "red_sign", Confidence: 0.3},
		&telemetryv1.Detection{ClassName: "green_sign", Confidence: 0.95},
	)
	svc := vision.NewService(memory.NewMemory(), &fakeTelemetry{history: []*telemetryv1.RobotSnapshot{snap}})

	min := float32(0.5)
	dets, err := svc.ListDetections(context.Background(), 10, &min)
	if err != nil {
		t.Fatalf("ListDetections: %v", err)
	}
	if len(dets) != 1 || dets[0].ClassName != "green_sign" {
		t.Fatalf("ListDetections(min=0.5) = %+v, want only green_sign", dets)
	}
}

func TestAnnotationCRUD(t *testing.T) {
	svc := vision.NewService(memory.NewMemory(), &fakeTelemetry{})
	ctx := context.Background()

	created, err := svc.CreateAnnotation(ctx, vision.CreateAnnotationRequest{
		ClassName: "red_sign",
		Bbox:      vision.BoundingBox{X: 0.1, Y: 0.1, Width: 0.2, Height: 0.2},
		ImageID:   "img-1",
	})
	if err != nil {
		t.Fatalf("CreateAnnotation: %v", err)
	}

	newClass := "green_sign"
	updated, err := svc.UpdateAnnotation(ctx, created.ID, vision.UpdateAnnotationRequest{ClassName: &newClass})
	if err != nil {
		t.Fatalf("UpdateAnnotation: %v", err)
	}
	if updated.ClassName != newClass {
		t.Fatalf("ClassName = %q, want %q", updated.ClassName, newClass)
	}
	if updated.UpdatedAt == nil {
		t.Fatal("UpdatedAt not set after update")
	}

	if err := svc.DeleteAnnotation(ctx, created.ID); err != nil {
		t.Fatalf("DeleteAnnotation: %v", err)
	}
	if _, err := svc.UpdateAnnotation(ctx, created.ID, vision.UpdateAnnotationRequest{}); !errors.Is(err, vision.ErrNotFound) {
		t.Fatalf("UpdateAnnotation after delete: want ErrNotFound, got %v", err)
	}
}

func TestSetActiveModelReflectedInPipelineStatus(t *testing.T) {
	svc := vision.NewService(memory.NewMemory(), &fakeTelemetry{})
	ctx := context.Background()

	if _, err := svc.SetActiveModel(ctx, vision.SetModelRequest{Name: "yolo11n"}); err != nil {
		t.Fatalf("SetActiveModel: %v", err)
	}

	st, err := svc.PipelineStatus(ctx)
	if err != nil {
		t.Fatalf("PipelineStatus: %v", err)
	}
	if !st.ModelLoaded {
		t.Fatal("ModelLoaded = false, want true after SetActiveModel")
	}
	if st.ModelName == nil || *st.ModelName != "yolo11n" {
		t.Fatalf("ModelName = %v, want yolo11n", st.ModelName)
	}
}
