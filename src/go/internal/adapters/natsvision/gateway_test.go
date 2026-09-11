package natsvision

import (
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	visionv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/vision/v1"
)

// fakePose is a PoseSource test double.
type fakePose struct {
	pose trackmodel.Pose
	ok   bool
}

func (f fakePose) GetCurrentPose() (trackmodel.Pose, bool) { return f.pose, f.ok }

func redDetection(centerX, heightPX float64) *visionv1.Detection {
	return &visionv1.Detection{
		ClassName:  visionv1.SignColor_SIGN_COLOR_RED,
		Confidence: 0.9,
		Bbox: &visionv1.BBox{
			XMin: centerX - 5, YMin: 100, XMax: centerX + 5, YMax: 100 + heightPX,
		},
		X:      centerX,
		Height: heightPX,
	}
}

func TestConvert_RejectsNonSignClasses(t *testing.T) {
	t.Parallel()

	for _, color := range []visionv1.SignColor{
		visionv1.SignColor_SIGN_COLOR_MAGENTA,
		visionv1.SignColor_SIGN_COLOR_UNSPECIFIED,
	} {
		det := &visionv1.Detection{ClassName: color, Bbox: &visionv1.BBox{}}
		if _, ok := convert(det); ok {
			t.Errorf("convert(%v) ok = true, want false", color)
		}
	}
}

func TestConvert_AcceptsRedAndGreen(t *testing.T) {
	t.Parallel()

	red := redDetection(320, 40)
	bbox, ok := convert(red)
	if !ok {
		t.Fatal("convert(red) ok = false, want true")
	}
	if bbox.Color != signrouter.SignColorRed {
		t.Errorf("Color = %v, want SignColorRed", bbox.Color)
	}
	if bbox.CenterX != 320 || bbox.Height != 40 {
		t.Errorf("CenterX/Height = %v/%v, want 320/40", bbox.CenterX, bbox.Height)
	}
}

func TestGateway_GetVisionDetections_NoMessageYet(t *testing.T) {
	t.Parallel()

	gw, err := New(signrouter.DefaultConfig(), 5.0, 0.05, fakePose{ok: true})
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	if _, ok := gw.GetVisionDetections(); ok {
		t.Error("GetVisionDetections() ok = true before any message, want false")
	}
}

func TestGateway_Observe_DropsWithoutPose(t *testing.T) {
	t.Parallel()

	gw, err := New(signrouter.DefaultConfig(), 5.0, 0.05, fakePose{ok: false})
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	gw.observe(&visionv1.Detections{Detections: []*visionv1.Detection{redDetection(320, 40)}})
	if _, ok := gw.GetVisionDetections(); ok {
		t.Error("GetVisionDetections() ok = true after a message arrived with no pose, want false")
	}
}

func TestGateway_Observe_ProducesWorldFrameObservation(t *testing.T) {
	t.Parallel()

	pose := fakePose{pose: trackmodel.Pose{X: 1.0, Y: 1.0, Yaw: 0.0}, ok: true}
	gw, err := New(signrouter.DefaultConfig(), 5.0, 0.05, pose)
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	// A tall bbox centred in frame projects to a short, roughly-ahead range --
	// the exact number is DetectionToObservation's own concern (already
	// tested in the signrouter package); this only checks the plumbing
	// reaches it and caches a result.
	gw.observe(&visionv1.Detections{Detections: []*visionv1.Detection{redDetection(
		signrouter.DefaultConfig().CameraWidthPX/2, 200,
	)}})

	obs, ok := gw.GetVisionDetections()
	if !ok {
		t.Fatal("GetVisionDetections() ok = false after a valid observation, want true")
	}
	if len(obs) != 1 {
		t.Fatalf("len(obs) = %d, want 1", len(obs))
	}
	if obs[0].Color != signrouter.SignColorRed {
		t.Errorf("Color = %v, want SignColorRed", obs[0].Color)
	}
}

func TestGateway_Observe_EmptyBatchIsNotADropout(t *testing.T) {
	t.Parallel()

	gw, err := New(signrouter.DefaultConfig(), 5.0, 0.05, fakePose{ok: true})
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	gw.observe(&visionv1.Detections{})

	obs, ok := gw.GetVisionDetections()
	if !ok {
		t.Fatal("GetVisionDetections() ok = false after an empty (but received) batch, want true")
	}
	if len(obs) != 0 {
		t.Errorf("len(obs) = %d, want 0", len(obs))
	}
}

func TestNew_RejectsNilPoseSource(t *testing.T) {
	t.Parallel()

	if _, err := New(signrouter.DefaultConfig(), 5.0, 0.05, nil); err == nil {
		t.Error("New(nil pose source) err = nil, want an error")
	}
}
