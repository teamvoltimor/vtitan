package signrouter

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/waypoints"
)

// CameraFocalPX is the pinhole focal length in pixels, derived from the
// configured HFOV and image width, matching sign_discovery.py's
// _CAMERA_FOCAL_PX.
//
// Derived rather than stored so it cannot fall out of step with the HFOV it
// is computed from: these values used to be package constants carrying a
// 60 deg placeholder while the shipped robot.toml said 102 deg (a Raspberry
// Pi Camera Module 3 Wide), and nothing raised because a constant has no
// loader to disagree with.
func (c Config) CameraFocalPX() float64 {
	return (c.CameraWidthPX / 2.0) / math.Tan(c.CameraHFOVRad/2.0)
}

// DetectionToWorld projects a pixel bounding box to an approximate world
// position, matching sign_discovery.py's _detection_to_world. Bearing is
// accurate (horizontal position in frame is depth-independent); range is
// estimated from the bbox height via the pinhole model (error grows with
// range), optionally overridden by a coincident LIDAR ray. Returns nil when
// the bbox is too small to trust.
func (c Config) DetectionToWorld(
	det BoundingBox,
	robotPos trackmodel.Waypoint,
	robotYaw,
	minReliableBBoxHeightPX,
	minValidLidarRangeM float64,
	lidarRangesM, lidarAnglesRad []float64,
) *trackmodel.Waypoint {
	pixelHeight := det.Height
	if pixelHeight < minReliableBBoxHeightPX {
		return nil
	}

	// Range from the pinhole model: d = (f * real_h) / pixel_h.
	distance := (c.CameraFocalPX() * c.SignHeightM) / pixelHeight

	// Horizontal angle from image centre.
	cx := det.CenterX
	thetaH := (cx/c.CameraWidthPX - 0.5) * c.CameraHFOVRad

	if len(lidarRangesM) > 0 && len(lidarAnglesRad) > 0 {
		lidarRange := navutil.NearestRay(lidarRangesM, lidarAnglesRad, thetaH)
		if minValidLidarRangeM < lidarRange && lidarRange < c.CameraFarClipM {
			distance = lidarRange
		}
	}

	// Project from the sensor, not the body centre.
	sensorX := robotPos.X + c.SensorMountXOffsetM*math.Cos(robotYaw)
	sensorY := robotPos.Y + c.SensorMountXOffsetM*math.Sin(robotYaw)
	bearing := robotYaw + thetaH
	wx := sensorX + distance*math.Cos(bearing)
	wy := sensorY + distance*math.Sin(bearing)
	return &trackmodel.Waypoint{X: wx, Y: wy}
}

// DetectionToObservation converts a pixel bounding box into a world-frame
// TrafficSignObservation, matching sign_discovery.py's
// detection_to_observation. Returns nil for a non-sign class or an
// unprojectable bbox.
func (c Config) DetectionToObservation(
	det BoundingBox,
	robotPos trackmodel.Waypoint,
	robotYaw,
	minReliableBBoxHeightPX,
	minValidLidarRangeM float64,
	lidarRangesM, lidarAnglesRad []float64,
) *TrafficSignObservation {
	if det.Color != SignColorRed && det.Color != SignColorGreen {
		return nil
	}
	world := c.DetectionToWorld(
		det, robotPos, robotYaw,
		minReliableBBoxHeightPX, minValidLidarRangeM,
		lidarRangesM, lidarAnglesRad,
	)
	if world == nil {
		return nil
	}
	return &TrafficSignObservation{
		WorldXM:    world.X,
		WorldYM:    world.Y,
		Color:      det.Color,
		Confidence: det.Confidence,
	}
}

// BoundingBox is a pixel detection's geometry, matching the fields
// DetectionToWorld reads (the subset of sign_discovery.py's Detection.as_bbox
// plus its class_name/confidence). Keep it in this package so callers do not
// have to import the ROS wire type.
type BoundingBox struct {
	XMin, YMin, XMax, YMax float64
	// CenterX is the horizontal centre in pixels (cx in the Python source).
	CenterX float64
	// Height is the bbox pixel height (YMax - YMin).
	Height     float64
	Color      SignColor
	Confidence float64
}

// DiscoveryConfig tunes ObservedSignMap, matching the constructor parameters of
// sign_discovery.py's ObservedSignMap (sourced from NavigationTuning
// .sign_discovery in Python). See the Default* block for the shipped values.
type DiscoveryConfig struct {
	MinConfidence           float64
	MaxIngestRangeM         float64
	AssociationDistM        float64
	MinHits                 int
	RobotCorridorFlipTicks  int
	MinReliableBBoxHeightPX float64
	// MinValidLidarRangeM is lidar_sectors.MIN_VALID_RANGE_M, the floor for
	// the LIDAR range-fusion gate.
	MinValidLidarRangeM float64
	// CornerMinM/CornerMaxM are TrackDimensions CORNER_MIN/CORNER_MAX, the
	// track bounds corridor_for_position uses to settle the robot's corridor
	// (the association gate). Mirrors SignRouter's TrackCornerMin/MaxM.
	CornerMinM float64
	CornerMaxM float64
}

// Default* mirror the shipped sign_discovery.toml / lidar_sectors values.
// DefaultMinConfidence is reused from this package's sighted router config.
//
// RobotCorridorFlipTicks and MinReliableBBoxHeightPX carried unconfirmed
// placeholders (1 and 8.0) against the shipped 5 and 5 until 2026-09-06;
// DiscoveryConfigFor now reads the file, so these are the no-config-root
// fallback rather than a second source of truth.
const (
	DefaultMaxIngestRangeM         = 2.0
	DefaultAssociationDistM        = 0.25
	DefaultMinHits                 = 3
	DefaultRobotCorridorFlipTicks  = 5
	DefaultMinReliableBBoxHeightPX = 5.0
	DefaultMinValidLidarRangeM     = 0.05
	DefaultCornerMinM              = 1.0
	DefaultCornerMaxM              = 2.0
)

// DefaultDiscoveryConfig returns the DiscoveryConfig matching the shipped
// Python tuning defaults.
func DefaultDiscoveryConfig() DiscoveryConfig {
	return DiscoveryConfig{
		MinConfidence:           DefaultMinConfidence,
		MaxIngestRangeM:         DefaultMaxIngestRangeM,
		AssociationDistM:        DefaultAssociationDistM,
		MinHits:                 DefaultMinHits,
		RobotCorridorFlipTicks:  DefaultRobotCorridorFlipTicks,
		MinReliableBBoxHeightPX: DefaultMinReliableBBoxHeightPX,
		MinValidLidarRangeM:     DefaultMinValidLidarRangeM,
		CornerMinM:              DefaultCornerMinM,
		CornerMaxM:              DefaultCornerMaxM,
	}
}

// signTrack is one candidate sign, accumulated across frames, matching
// sign_discovery.py's _SignTrack.
type signTrack struct {
	x, y           float64
	bestRange      float64
	corridor       trackmodel.Section
	hits           int
	votes          map[SignColor]float64
	publishedIndex *int
}

// color returns the confidence-weighted majority colour vote.
func (t *signTrack) color() SignColor {
	best := SignColorRed
	bestV := -1.0
	for c, v := range t.votes {
		if v > bestV {
			bestV = v
			best = c
		}
	}
	return best
}

// asSpec materialises the track as a SignSpec.
func (t *signTrack) asSpec() SignSpec {
	return SignSpec{X: t.x, Y: t.y, Color: t.color()}
}

// ObservedSignMap is a persistent world-frame sign map accumulated from camera
// detections, matching sign_discovery.py's ObservedSignMap. Tracks are
// append-only once published so SignRouter's index-keyed bookkeeping stays
// valid; positions are refined in place from the closest observation.
type ObservedSignMap struct {
	cfg    DiscoveryConfig
	sd     *SignRouter
	tracks []*signTrack

	robotCorridor           *trackmodel.Section
	robotCorridorFlipStreak *struct {
		corridor trackmodel.Section
		streak   int
	}
}

// NewObservedSignMap builds an empty map, matching ObservedSignMap.__init__.
// sd, when non-nil, is the SignRouter the confirmed tracks are published into
// (the discover wiring); nil keeps the map standalone.
func NewObservedSignMap(cfg DiscoveryConfig, sd *SignRouter) *ObservedSignMap {
	if sd != nil {
		cfg.CornerMinM = sd.cornerMinM()
		cfg.CornerMaxM = sd.cornerMaxM()
	}
	return &ObservedSignMap{
		cfg:    cfg,
		sd:     sd,
		tracks: nil,
	}
}

// IsDiscovering reports whether this map is actively feeding a SignRouter
// (discover mode), matching SignRouter.is_discovering.
func (m *ObservedSignMap) IsDiscovering() bool { return m != nil && m.sd != nil }

func (m *ObservedSignMap) settleRobotCorridor(raw trackmodel.Section) trackmodel.Section {
	if m.robotCorridor == nil || raw == *m.robotCorridor {
		m.robotCorridorFlipStreak = nil
		m.robotCorridor = &raw
		return raw
	}
	var candidate trackmodel.Section
	streak := 0
	if s := m.robotCorridorFlipStreak; s != nil {
		candidate, streak = s.corridor, s.streak
	}
	if candidate != raw {
		streak = 1
	} else {
		streak++
	}
	if streak < m.cfg.RobotCorridorFlipTicks {
		m.robotCorridorFlipStreak = &struct {
			corridor trackmodel.Section
			streak   int
		}{corridor: raw, streak: streak}
		return *m.robotCorridor
	}
	m.robotCorridorFlipStreak = nil
	m.robotCorridor = &raw
	return raw
}

// Observe folds one frame of world-coordinate observations into the map,
// matching ObservedSignMap.observe.
func (m *ObservedSignMap) Observe(
	observations []TrafficSignObservation,
	robotPos trackmodel.Waypoint,
) {
	if len(observations) == 0 {
		return
	}
	robotCorridor := m.settleRobotCorridor(
		waypoints.CorridorForPosition(robotPos.X, robotPos.Y, m.cfg.CornerMinM, m.cfg.CornerMaxM),
	)
	for i := range observations {
		obs := observations[i]
		if obs.Confidence < m.cfg.MinConfidence {
			continue
		}
		if obs.Color != SignColorRed && obs.Color != SignColorGreen {
			continue
		}
		world := trackmodel.Waypoint{X: obs.WorldXM, Y: obs.WorldYM}
		observedRange := world.DistanceTo(robotPos)
		if observedRange > m.cfg.MaxIngestRangeM {
			continue
		}
		m.fold(world, observedRange, obs, robotCorridor)
	}
}

func (m *ObservedSignMap) fold(
	world trackmodel.Waypoint,
	observedRange float64,
	obs TrafficSignObservation,
	robotCorridor trackmodel.Section,
) {
	track := m.nearestTrack(world, robotCorridor)
	if track == nil {
		track = &signTrack{
			x: world.X, y: world.Y, bestRange: observedRange,
			corridor: robotCorridor, hits: 0,
			votes: map[SignColor]float64{},
		}
		m.tracks = append(m.tracks, track)
	}
	track.hits++
	track.votes[obs.Color] += obs.Confidence

	// Closest observation wins outright: pinhole range error is monotone in
	// range, so a nearer reading is strictly better evidence.
	if observedRange <= track.bestRange {
		track.bestRange = observedRange
		track.x, track.y = world.X, world.Y
	}
}

func (m *ObservedSignMap) nearestTrack(
	world trackmodel.Waypoint,
	robotCorridor trackmodel.Section,
) *signTrack {
	best := (*signTrack)(nil)
	bestDist := m.cfg.AssociationDistM
	for _, t := range m.tracks {
		if t.corridor != robotCorridor {
			continue
		}
		d := math.Hypot(t.x-world.X, t.y-world.Y)
		if d < bestDist {
			bestDist = d
			best = t
		}
	}
	return best
}

// NewlyConfirmed returns tracks that crossed MinHits and are not yet
// published, matching ObservedSignMap.newly_confirmed. The caller assigns
// published_index once the returned specs are appended to the router.
func (m *ObservedSignMap) NewlyConfirmed() []*signTrack {
	var out []*signTrack
	for _, t := range m.tracks {
		if t.publishedIndex == nil && t.hits >= m.cfg.MinHits {
			out = append(out, t)
		}
	}
	return out
}

// Publish folds every newly-confirmed track into the SignRouter (discover
// mode), assigning it the next index. No-op without a router.
func (m *ObservedSignMap) Publish() {
	if m.sd == nil {
		return
	}
	for _, t := range m.NewlyConfirmed() {
		idx := m.sd.AppendSign(t.asSpec())
		i := idx
		t.publishedIndex = &i
	}
}

// Specs returns the world-frame SignSpec for every track, matching the Python
// _SignTrack.as_spec over all tracks (published and pending).
func (m *ObservedSignMap) Specs() []SignSpec {
	out := make([]SignSpec, 0, len(m.tracks))
	for _, t := range m.tracks {
		out = append(out, t.asSpec())
	}
	return out
}
