// Package natsvision implements navigator.VisionGateway over the
// vtitan.vision.v1.detections NATS subject, published by the Python
// sidecar (src/python/src/vision/nats_sidecar.py) -- camera capture and
// Hailo NPU inference stay in Python (no Go HailoRT bindings exist; see
// the hailort_go_bindings_feasibility research), while the bbox-to-world
// geometry (signrouter.Config.DetectionToObservation, already a 1:1 port of
// sign_discovery.py's pinhole math) runs here in Go, against Go's own
// already-local pose estimate rather than a cross-process one.
//
// The pull-cache shape (subscribe once, store latest under a sync.RWMutex,
// serve GetVisionDetections from the cache) matches natsgw.Gateway's own
// pattern for the same reason: a hardware gateway's Get* methods must never
// block the navigator's tick on a network read.
package natsvision

import (
	"context"
	"errors"
	"sync"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	visionv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/vision/v1"
	natsx "github.com/teamvoltimor/vtitan/src/go/internal/transport/nats"
)

// PoseSource is the subset of controllers.HardwareGateway this package
// reads: the pose each detection is projected from. Declared narrowly
// (rather than importing the whole port) so a caller with a narrower pose
// source -- a test double, or a future gateway that separates pose from
// drive -- can satisfy it without implementing anything this package never
// touches.
type PoseSource interface {
	GetCurrentPose() (trackmodel.Pose, bool)
}

// Gateway implements navigator.VisionGateway, matching this package's own
// doc comment above.
type Gateway struct {
	cfg                     signrouter.Config
	minReliableBBoxHeightPX float64
	minValidLidarRangeM     float64
	pose                    PoseSource

	mu         sync.RWMutex
	latest     []signrouter.TrafficSignObservation
	haveLatest bool
}

// New builds a Gateway. cfg supplies the pinhole/mount constants
// DetectionToObservation projects with -- pass the SAME signrouter.Config
// the navigator itself was constructed with (Params.SignRouterConfig), so
// the vision geometry and the sign router agree about the camera it is
// reading. pose supplies the robot pose each detection batch is attributed
// to; minReliableBBoxHeightPX/minValidLidarRangeM match
// signrouter.DiscoveryConfig's fields of the same name (LIDAR range fusion
// is never exercised here -- see convert's own comment -- so
// minValidLidarRangeM only guards a code path that is currently always
// skipped, kept for signature symmetry with DetectionToObservation).
func New(
	cfg signrouter.Config,
	minReliableBBoxHeightPX, minValidLidarRangeM float64,
	pose PoseSource,
) (*Gateway, error) {
	if pose == nil {
		return nil, errors.New("natsvision: nil pose source")
	}
	return &Gateway{
		cfg:                     cfg,
		minReliableBBoxHeightPX: minReliableBBoxHeightPX,
		minValidLidarRangeM:     minValidLidarRangeM,
		pose:                    pose,
	}, nil
}

// GetVisionDetections implements navigator.VisionGateway. ok=false until the
// first Detections message has arrived; an empty, non-nil slice is a
// genuine "nothing in frame this tick", not a dropout -- callers must not
// conflate the two, matching Detections' own proto doc comment.
func (g *Gateway) GetVisionDetections() ([]signrouter.TrafficSignObservation, bool) {
	g.mu.RLock()
	defer g.mu.RUnlock()
	return g.latest, g.haveLatest
}

// Run subscribes to sub and converts each message into this tick's
// observations until ctx is done or Read returns a non-cancellation error.
func (g *Gateway) Run(ctx context.Context, sub *natsx.Subscriber[*visionv1.Detections]) error {
	for {
		msg, err := sub.Read(ctx)
		if err != nil {
			if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
				return nil
			}
			return err //nolint:wrapcheck // Read already wraps with "nats: ..." context
		}
		g.observe(msg)
	}
}

// observe converts one Detections message into world-frame observations
// against the CURRENT pose and caches them. A message that arrives before
// any pose is available is dropped rather than projected against the zero
// pose -- a wrong-but-plausible-looking observation is worse than none.
func (g *Gateway) observe(msg *visionv1.Detections) {
	pose, havePose := g.pose.GetCurrentPose()
	if !havePose {
		return
	}
	robotPos := trackmodel.Waypoint{X: pose.X, Y: pose.Y}

	observations := make([]signrouter.TrafficSignObservation, 0, len(msg.GetDetections()))
	for _, det := range msg.GetDetections() {
		bbox, ok := convert(det)
		if !ok {
			continue
		}
		obs := g.cfg.DetectionToObservation(
			bbox, robotPos, pose.Yaw,
			g.minReliableBBoxHeightPX, g.minValidLidarRangeM,
			nil, nil, // LIDAR range fusion: never exercised here, see convert's doc.
		)
		if obs != nil {
			observations = append(observations, *obs)
		}
	}

	g.mu.Lock()
	g.latest = observations
	g.haveLatest = true
	g.mu.Unlock()
}

// convert maps one wire Detection onto signrouter.BoundingBox, ok=false for
// a MAGENTA/UNSPECIFIED class DetectionToObservation would reject anyway
// (checked here too so a caller need not construct a BoundingBox for a
// detection that can never become an observation).
//
// LIDAR range fusion is deliberately never wired in from this package: it
// ships OFF in the Python original (SIGN_LIDAR_PROPOSE defaults false, and
// the per-detection fusion flag is documented there as measured WORSE, not
// merely unused -- see sign_discovery.py's LIDAR_RANGE_FUSION comment), so
// the pinhole-only estimate DetectionToObservation falls back to with nil
// ranges/angles already matches shipped behavior.
func convert(det *visionv1.Detection) (signrouter.BoundingBox, bool) {
	var color signrouter.SignColor
	switch det.GetClassName() {
	case visionv1.SignColor_SIGN_COLOR_RED:
		color = signrouter.SignColorRed
	case visionv1.SignColor_SIGN_COLOR_GREEN:
		color = signrouter.SignColorGreen
	default:
		return signrouter.BoundingBox{}, false
	}
	return signrouter.BoundingBox{
		XMin:       det.GetBbox().GetXMin(),
		YMin:       det.GetBbox().GetYMin(),
		XMax:       det.GetBbox().GetXMax(),
		YMax:       det.GetBbox().GetYMax(),
		CenterX:    det.GetX(),
		Height:     det.GetHeight(),
		Color:      color,
		Confidence: det.GetConfidence(),
	}, true
}
