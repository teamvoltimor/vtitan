package signrouter

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// NewSignRouterForTest builds an empty SignRouter for discovery wiring, since
// discover mode starts with no known signs.
func NewSignRouterForTest() *SignRouter {
	r, err := NewSignRouter(nil, DefaultConfig(), trackmodel.Clockwise)
	if err != nil {
		panic(err)
	}
	return r
}

// TestDetectionToWorld_Pinhole verifies the bearing is accurate and range
// comes from the pinhole model, mirroring sign_discovery.py's
// _detection_to_world: a sign dead ahead (cx at image centre) projects to the
// robot's heading at the pinhole-estimated distance.
func TestDetectionToWorld_Pinhole(t *testing.T) {
	t.Parallel()
	// Sign 0.10 m tall, 42 px tall at 1.5 m (the doc's worked example).
	// distance = f * real_h / pixel_h.
	cfg := DefaultConfig()
	f := cfg.CameraFocalPX()
	wantDist := (f * cfg.SignHeightM) / 42.0

	det := BoundingBox{
		XMin: 0, YMin: 0, XMax: 100, YMax: 42,
		CenterX: cfg.CameraWidthPX / 2.0, Height: 42,
		Color: SignColorRed, Confidence: 0.9,
	}
	pose := trackmodel.Waypoint{X: 1.0, Y: 1.0}
	got := cfg.DetectionToWorld(
		det,
		pose,
		0,
		DefaultMinReliableBBoxHeightPX,
		DefaultMinValidLidarRangeM,
		nil,
		nil,
	)
	if got == nil {
		t.Fatal("DetectionToWorld = nil, want a point")
	}
	// Bearing 0 (forward) => +x, projected from the sensor origin (mount
	// offset forward of the body centre).
	wantX := pose.X + cfg.SensorMountXOffsetM + wantDist
	if math.Abs(got.X-wantX) > 1e-6 {
		t.Errorf("x = %v, want %v", got.X, wantX)
	}
	if math.Abs(got.Y-pose.Y) > 1e-6 {
		t.Errorf("y = %v, want %v", got.Y, pose.Y)
	}
}

// TestDetectionToWorld_Bearing mirrors the horizontal-angle term: a bbox right
// of centre yields a positive (left-of-forward? theta_h sign) offset matching
// theta_h = (cx/W - 0.5)*HFOV.
func TestDetectionToWorld_Bearing(t *testing.T) {
	t.Parallel()
	cfg := DefaultConfig()
	// Half-width off centre => theta_h = +HFOV/4 (to the left in image = the
	// sign is to the robot's left when cx > centre? cx is measured from left,
	// so cx > centre means the object is on the robot's right in yaw terms
	// per the Python model theta_h = (cx/W - 0.5)*HFOV, bearing = yaw+theta_h).
	cx := cfg.CameraWidthPX * 0.75
	det := BoundingBox{
		XMin: 0, YMin: 0, XMax: 20, YMax: 42,
		CenterX: cx, Height: 42,
		Color: SignColorGreen, Confidence: 0.9,
	}
	pose := trackmodel.Waypoint{X: 0, Y: 0}
	got := cfg.DetectionToWorld(
		det,
		pose,
		0,
		DefaultMinReliableBBoxHeightPX,
		DefaultMinValidLidarRangeM,
		nil,
		nil,
	)
	if got == nil {
		t.Fatal("DetectionToWorld = nil")
	}
	wantTheta := (cx/cfg.CameraWidthPX - 0.5) * cfg.CameraHFOVRad
	wantDist := (cfg.CameraFocalPX() * cfg.SignHeightM) / 42.0
	// At yaw 0 the mount offset is entirely along +x (forward), so the
	// sensor origin is (cfg.SensorMountXOffsetM, 0); only x carries the offset.
	wantX := cfg.SensorMountXOffsetM + wantDist*math.Cos(wantTheta)
	wantY := wantDist * math.Sin(wantTheta)
	if math.Abs(got.X-wantX) > 1e-6 {
		t.Errorf("x = %v, want %v", got.X, wantX)
	}
	if math.Abs(got.Y-wantY) > 1e-6 {
		t.Errorf("y = %v, want %v", got.Y, wantY)
	}
}

// TestObservedSignMap_StableSpec mirrors test_sign_discovery.py: repeated
// observations at slightly different positions collapse into ONE stable
// SignSpec (closest observation wins) once MinHits is reached, and a far
// observation beyond MaxIngestRange is ignored.
func TestObservedSignMap_StableSpec(t *testing.T) {
	t.Parallel()
	sd := NewSignRouterForTest()
	m := NewObservedSignMap(DefaultDiscoveryConfig(), sd)

	// A red sign near (1.0, 0.4) seen from a robot at (1.0, 1.0) facing -y.
	robotPos := trackmodel.Waypoint{X: 1.0, Y: 1.0}
	// Project: bearing must point toward (1.0, 0.4) from (1.0,1.0) => -y.
	for i := 0; i < DefaultMinHits; i++ {
		obs := TrafficSignObservation{
			WorldXM: 1.0 + float64(i)*0.005, WorldYM: 0.4,
			Color: SignColorRed, Confidence: 0.9,
		}
		m.Observe([]TrafficSignObservation{obs}, robotPos)
	}
	m.Publish()

	specs := sd.Signs()
	if len(specs) != 1 {
		t.Fatalf("published signs = %d, want 1", len(specs))
	}
	// Closest observation is the first (i=0): (1.0, 0.4).
	if math.Abs(specs[0].X-1.0) > 1e-9 || math.Abs(specs[0].Y-0.4) > 1e-9 {
		t.Errorf("spec = (%v,%v), want (1.0,0.4) (closest observation)", specs[0].X, specs[0].Y)
	}
	if specs[0].Color != SignColorRed {
		t.Errorf("color = %v, want red", specs[0].Color)
	}
	if !m.IsDiscovering() {
		t.Error("IsDiscovering = false, want true (router attached)")
	}
}

// TestObservedSignMap_IgnoresFar mirrors the MAX_INGEST_RANGE gate: a sign
// beyond MaxIngestRangeM never produces a published spec.
func TestObservedSignMap_IgnoresFar(t *testing.T) {
	t.Parallel()
	sd := NewSignRouterForTest()
	m := NewObservedSignMap(DefaultDiscoveryConfig(), sd)
	robotPos := trackmodel.Waypoint{X: 1.0, Y: 1.0}
	far := DefaultMaxIngestRangeM + 0.5
	obs := TrafficSignObservation{
		WorldXM:    1.0,
		WorldYM:    1.0 - far,
		Color:      SignColorGreen,
		Confidence: 0.9,
	}
	for i := 0; i < DefaultMinHits*2; i++ {
		m.Observe([]TrafficSignObservation{obs}, robotPos)
	}
	m.Publish()
	if len(sd.Signs()) != 0 {
		t.Errorf("published signs = %d, want 0 (beyond ingest range)", len(sd.Signs()))
	}
}

// TestObservedSignMap_StandaloneNoPublish mirrors discover mode off: without a
// router the map accumulates tracks but publishes nothing.
func TestObservedSignMap_StandaloneNoPublish(t *testing.T) {
	t.Parallel()
	m := NewObservedSignMap(DefaultDiscoveryConfig(), nil)
	robotPos := trackmodel.Waypoint{X: 1.0, Y: 1.0}
	obs := TrafficSignObservation{WorldXM: 1.0, WorldYM: 0.4, Color: SignColorRed, Confidence: 0.9}
	for i := 0; i < DefaultMinHits; i++ {
		m.Observe([]TrafficSignObservation{obs}, robotPos)
	}
	m.Publish()
	if m.IsDiscovering() {
		t.Error("IsDiscovering = true, want false (no router)")
	}
	if len(m.Specs()) != 1 {
		t.Errorf("tracks = %d, want 1", len(m.Specs()))
	}
}
