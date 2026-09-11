package widthbelief

import (
	"log/slog"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/corridorestimator"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navigator"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/waypoints"
)

// Layout is the per-tick blind layout-belief loop: fold the latest scan into
// the corridor-width estimate, decide (via Gate) whether the change may move
// the path yet, and replan when it may. It ports
// track_navigator_node.py's _update_layout_belief.
//
// It lives here rather than in internal/nav/navigator because it is a HOST
// concern in the Python original too: CoreNavigator drives a path it is
// given, and the node owns the belief that produces one. Both Go hosts (the
// simulator's NativeRunner and cmd/track-navigator) need the identical loop,
// so it is written once here instead of twice at the two composition roots.
type Layout struct {
	logger    *slog.Logger
	estimator *corridorestimator.WidthEstimator
	gate      *Gate

	base          waypoints.PlannerInput
	cfg           waypoints.Config
	centerBiasM   *float64
	maxCoordM     float64
	lastReplanned map[trackmodel.Section]float64
	// creepReplayed records that the navigator's creep-phase width buffer
	// has been drained. Once only: the readings are votes, and replaying
	// them a second time would count each one twice.
	creepReplayed bool
}

// Params is everything Layout needs to replan a believed path.
type Params struct {
	Logger *slog.Logger
	// Estimator is the corridor-width estimator to fold scans into. A nil
	// estimator disables the loop entirely (Update becomes a no-op), which is
	// the sighted case: a told width has nothing to estimate.
	Estimator *corridorestimator.WidthEstimator
	// Defer enables holding a change back until the robot has left the
	// corridor it describes (waypoints.Config.DeferCurrentCorridorReplan).
	Defer bool
	// Base is the planner input the believed path is rebuilt from; its
	// Geometry is replaced with the current belief on every replan.
	Base waypoints.PlannerInput
	// Config is the waypoint tuning to plan with.
	Config waypoints.Config
	// CenterBiasM is the Obstacles uniform-bias override, nil for Open.
	CenterBiasM *float64
	// MaxCoordM is the track's outer boundary, used to rebuild the wall model
	// the localizer matches against.
	MaxCoordM float64
}

// NewLayout builds a Layout. A nil Params.Estimator returns nil, so a
// sighted caller can construct unconditionally and check for nil once.
func NewLayout(p Params) *Layout {
	if p.Estimator == nil {
		return nil
	}
	return &Layout{
		logger:      p.Logger,
		estimator:   p.Estimator,
		gate:        New(p.Defer),
		base:        p.Base,
		cfg:         p.Config,
		centerBiasM: p.CenterBiasM,
		maxCoordM:   p.MaxCoordM,
	}
}

// Sensors is the slice of controllers.HardwareGateway the belief loop reads:
// the scan it measures a corridor from, the pose it attributes that
// measurement by, and the wall model it must re-seed so the NEXT pose is
// computed against the layout just adopted.
//
// Declared here rather than taking the whole gateway so a caller with a
// narrower simulation gateway (internal/sim/scenario's simGateway) can pass
// it without implementing the parts of the port a belief loop never touches.
type Sensors interface {
	GetLidarScan() (controllers.LidarScan, bool)
	GetCurrentPose() (trackmodel.Pose, bool)
	SetBelievedWalls(walls *trackmodel.TrackWalls)
}

// Update folds one tick's scan into the belief and replans nav's path if the
// gate released a change. Returns true when the path was rebuilt.
//
// direction is the round's travel direction; a blind round that has not yet
// settled one passes nil and Update does nothing, because attribution needs
// it (see the section comment below).
//
// A nil Layout is a no-op, so a sighted caller need not branch.
func (l *Layout) Update(
	nav *navigator.Navigator,
	gateway Sensors,
	direction *trackmodel.Direction,
) bool {
	if l == nil || direction == nil {
		return false
	}
	scan, haveScan := gateway.GetLidarScan()
	pose, havePose := gateway.GetCurrentPose()
	if !haveScan || !havePose {
		return false
	}

	// Attribute the reading by HEADING, not position. Position would be
	// circular -- it comes from matching against a wall model built from the
	// very widths being estimated, so a wrong belief mis-attributes the
	// reading that would have corrected it and the error locks in. Heading
	// comes from the IMU and owes nothing to the map.
	section := corridorestimator.SectionFromHeading(pose.Yaw, *direction)

	// Fold in the readings taken during BLIND_CREEP, now that there is a
	// direction to attribute them by. They were taken driving straight down
	// a corridor and are the cleanest of the round; without them the first
	// readings the estimator ever sees are whatever this tick happens to
	// offer, which on a round that settled its direction at a CORNER is a
	// side ray running off down the NEXT corridor, filed against this one.
	//
	// Attributed per reading by its OWN heading, not this tick's: the buffer
	// can span a turn, and one section label for all of them would file the
	// pre-turn readings against the post-turn corridor.
	if !l.creepReplayed {
		l.creepReplayed = true
		for _, cw := range nav.TakeCreepWidths() {
			l.estimator.ObserveMeasurement(
				corridorestimator.SectionFromHeading(cw.Yaw, *direction), cw.WidthM,
			)
		}
	}

	l.estimator.Observe(section, scan.RangesM, scan.AnglesRad, pose.Yaw)

	// Gated every tick rather than only when Observe reports a change: a
	// belief held back is released by the robot LEAVING the corridor, not by
	// a new reading, so the tick that finally applies it is usually one the
	// estimator had nothing to say about.
	believed, unconfirmed, changed := l.gate.Update(
		l.estimator.Widths(), l.unconfirmedFromEstimator(), section,
	)
	if !changed {
		return false
	}

	geometry := trackmodel.CorridorGeometryFromWidths(believed, l.maxCoordM)
	path, err := waypoints.PlanBelievedPath(
		l.base,
		geometry,
		direction,
		section,
		trackmodel.Waypoint{X: pose.X, Y: pose.Y},
		pose.Yaw,
		l.cfg,
		l.centerBiasM,
		unconfirmed,
	)
	if err != nil {
		// A belief that cannot be planned is a belief to ignore, not a reason
		// to stop driving: the robot keeps the last feasible path and the
		// next reading may well produce a planable one. The gate has already
		// recorded the change, so this is not retried against the same
		// belief -- which is correct, since it would fail identically.
		l.logger.Warn("widthbelief: planning the updated layout failed, keeping the current path",
			"error", err)
		return false
	}

	// The localizer matches scans against a wall model, so it has to be told
	// the new layout too -- otherwise the pose feeding the next reading is
	// still computed against the widths that were just superseded.
	gateway.SetBelievedWalls(trackmodel.NewTrackWalls(geometry, -l.maxCoordM, l.maxCoordM))
	nav.ReplacePath(path, trackmodel.Waypoint{X: pose.X, Y: pose.Y}, &pose.Yaw)
	l.lastReplanned = believed
	l.logger.Info("widthbelief: layout belief updated",
		"north_m", believed[trackmodel.North],
		"south_m", believed[trackmodel.South],
		"east_m", believed[trackmodel.East],
		"west_m", believed[trackmodel.West])
	return true
}

// Believed is the layout the PATH is currently planned from, or nil before
// the first replan. Exposed for diagnostics and tests, not for control.
func (l *Layout) Believed() map[trackmodel.Section]float64 {
	if l == nil {
		return nil
	}
	return l.lastReplanned
}

// unconfirmedFromEstimator reads the estimator's per-section observed flags
// into the planner's set. Built fresh each tick rather than cached: the
// estimator owns the truth, and a cached copy would be one more thing that
// can disagree with it.
func (l *Layout) unconfirmedFromEstimator() waypoints.UnconfirmedSections {
	var set waypoints.UnconfirmedSections
	for _, section := range []trackmodel.Section{
		trackmodel.North, trackmodel.South, trackmodel.East, trackmodel.West,
	} {
		set.Set(section, !l.estimator.IsObserved(section))
	}
	return set
}
