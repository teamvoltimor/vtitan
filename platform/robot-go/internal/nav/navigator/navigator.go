package navigator

import (
	"errors"
	"fmt"
	"log/slog"
	"math"
	"slices"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/waypoints"
)

// VisionGateway is the port supplying traffic-sign observations for
// SignRouter's per-tick deformation, matching
// ports.HardwareGateway.get_vision_detections.
//
// Kept separate from controllers.HardwareGateway so that package retains no
// dependency on signrouter -- the observation type is signrouter's, and
// controllers sits below it in the layering. A nil VisionGateway, or one
// returning ok=false, is "no observations this tick", exactly as Python's
// gateway returns an empty sequence when nothing was detected.
type VisionGateway interface {
	// GetVisionDetections returns this tick's traffic-sign detections, and
	// ok=false when none are available.
	GetVisionDetections() ([]signrouter.TrafficSignObservation, bool)
}

// Params bundles Navigator's construction inputs, matching
// CoreNavigator.__init__'s parameter list plus the three config structs Go
// threads explicitly where Python reads module-level singletons
// (NavigationTuning, RobotSpecs, SignRouterConfig).
//
// LapDetector and ParkController have no Go equivalent and are absent by
// design, not omitted by accident -- see doc.go for what that removes from
// Step's behavior. Direction is a plain value rather than a pointer for
// the same reason: this port is sighted-only, so the travel direction is
// known at construction and never passes through the blind creep phase
// that made it optional in Python.
type Params struct {
	// Gateway is the hardware port every branch of Step reads and
	// publishes through. Required.
	Gateway controllers.HardwareGateway
	// Vision supplies traffic-sign observations; nil means none, which is
	// every Open Challenge run.
	Vision VisionGateway
	// Waypoints is the planned path: one canonical lap of a closed loop.
	Waypoints []trackmodel.Waypoint
	// Direction is the travel direction around the loop, consumed by the
	// escape maneuver as the fallback side when a LIDAR-only clearance
	// comparison cannot decide one.
	Direction trackmodel.Direction
	// NumLaps is the laps to complete before holding position; 0 means
	// DefaultOpenChallengeLaps.
	NumLaps int
	// Config is this package's own tuning. The zero value is NOT usable --
	// pass DefaultConfig() or ConfigFor(...).
	Config Config
	// ControllersConfig builds the waypoint/collision/stuck controllers.
	ControllersConfig controllers.Config
	// SignRouterConfig parameterizes the lane transform and supplies the
	// chassis half-diagonal the lane offset is derived from. Ignored when
	// SignRouter is nil.
	SignRouterConfig signrouter.Config
	// SignRouter routes past traffic signs; nil outside the Obstacles
	// Challenge.
	SignRouter *signrouter.SignRouter
	// Logger receives the lap/escape/refusal messages CoreNavigator logs;
	// nil falls back to slog.Default().
	Logger *slog.Logger
}

// Navigator orchestrates navigation using a controllers.HardwareGateway,
// matching core_navigator.navigator.CoreNavigator (with
// escape_recovery.EscapeRecovery's methods flattened onto it, as they are
// at runtime in Python -- see escape_recovery.go).
type Navigator struct {
	logger  *slog.Logger
	gateway controllers.HardwareGateway
	vision  VisionGateway

	cfg           Config
	signRouterCfg signrouter.Config

	// waypoints is the path currently being driven; laneBaseWaypoints is
	// the path as PLANNED, before any sign-lane transform. Kept separately
	// so each lane rebuild starts from the centerline instead of stacking
	// onto the previous lane -- see refreshSignLanes.
	waypoints         []trackmodel.Waypoint
	laneBaseWaypoints []trackmodel.Waypoint
	// laneFingerprint is the sign layout the current lane path was built
	// for; laneFingerprintSet distinguishes "built for an empty layout"
	// from "never built", which Python spells `None`.
	laneFingerprint    []laneFingerprintEntry
	laneFingerprintSet bool

	numLaps    int
	signRouter *signrouter.SignRouter
	direction  trackmodel.Direction

	waypointIndex     int
	lapsCompleted     int
	suppressNextWrap  bool
	waypointThreshold float64
	currentCorridor   *trackmodel.Section

	// Escape-maneuver latching: an escape runs for its full
	// DurationFrames instead of a single 50 ms tick, and repeated escapes
	// escalate (reverse longer, switch side) rather than repeating an
	// identical failed pulse.
	activeManeuver     *controllers.EscapeManeuver
	maneuverFramesLeft int
	// escapeCount is escapes begun since the last normal drive tick with
	// real progress.
	escapeCount int
	// escapeSteerSign is the BASE side for escapes, not a running toggle:
	// which side a given attempt uses is derived in
	// escapeSteerSignForAttempt.
	escapeSteerSign       float64
	escapeSequenceStartXY *trackmodel.Waypoint
	// poseTrail is where the chassis has physically been, newest last. The
	// basis for a retrace-reverse: ground the robot occupied a moment ago
	// is known free without any rear-facing sensor. See retraceSteer.
	poseTrail []trackmodel.Pose
	retracing bool

	waypointController  *controllers.WaypointController
	collisionController *controllers.CollisionAvoidanceController
	stuckDetector       *controllers.StuckDetector

	debug DebugSnapshot
}

// laneFingerprintEntry is one sign of the layout a lane path was built
// for. Python reads SignRouter.lane_fingerprint, a discovery-only property
// with no Go counterpart (see signrouter's doc.go), so the fingerprint is
// derived here from LaneSpecs instead -- the same (x, y, corridor) triple,
// from the same source.
type laneFingerprintEntry struct {
	X, Y     float64
	Corridor trackmodel.Section
}

// perception is the LIDAR-derived picture one tick of Step works from.
//
// The two risk readings are deliberate: the RAW scan governs how fast the
// robot may go, the MAPPED-OBSTACLE-MASKED scan governs whether the escape
// maneuver fires. A sign the router is routing around is passed at the
// lateral offset by design, which is inside the contact distance -- so on
// the raw scan the escape maneuver would fire at every sign pass and
// reverse the robot out of the very gap the planner aimed for. Walls and
// unmapped returns are untouched in both readings.
type perception struct {
	scan             controllers.LidarScan
	haveScan         bool
	escapeRanges     []float64
	forwardClearance float64
	risk             controllers.RiskLevel
	escapeRisk       controllers.RiskLevel
	minRange         *float64
}

// ErrNoGateway is returned by New when no hardware gateway is supplied:
// every branch of Step both reads from and publishes to it, so there is no
// degraded mode to fall back on.
var ErrNoGateway = errors.New("navigator: gateway is required")

// ErrNoWaypoints is returned by New when the planned path is empty. Python
// tolerates it only because _apply_path_wall_budget early-returns; every
// subsequent index into the list would panic here.
var ErrNoWaypoints = errors.New("navigator: waypoints must not be empty")

// New builds a Navigator, matching CoreNavigator.__init__.
func New(p Params) (*Navigator, error) {
	if p.Gateway == nil {
		return nil, ErrNoGateway
	}
	if len(p.Waypoints) == 0 {
		return nil, ErrNoWaypoints
	}

	logger := p.Logger
	if logger == nil {
		logger = slog.Default()
	}
	numLaps := p.NumLaps
	if numLaps == 0 {
		numLaps = DefaultOpenChallengeLaps
	}

	stuckDetector, err := p.ControllersConfig.NewStuckDetector(logger)
	if err != nil {
		return nil, fmt.Errorf("navigator: building stuck detector: %w", err)
	}

	n := &Navigator{
		logger:              logger,
		gateway:             p.Gateway,
		vision:              p.Vision,
		cfg:                 p.Config,
		signRouterCfg:       p.SignRouterConfig,
		waypoints:           slices.Clone(p.Waypoints),
		laneBaseWaypoints:   slices.Clone(p.Waypoints),
		numLaps:             numLaps,
		signRouter:          p.SignRouter,
		direction:           p.Direction,
		waypointThreshold:   p.Config.MainLoopReachedDistanceM,
		escapeSteerSign:     1.0,
		waypointController:  p.ControllersConfig.NewWaypointController(),
		collisionController: p.ControllersConfig.NewCollisionAvoidanceController(),
		stuckDetector:       stuckDetector,
	}
	n.applyPathWallBudget()
	return n, nil
}

// DebugSnapshot is the full internal state of the most recent Step call,
// matching the debug_snapshot property.
func (n *Navigator) DebugSnapshot() DebugSnapshot { return n.debug }

// CurrentCorridor is the track corridor derived from the robot's position,
// matching the current_corridor property. nil before the first Step.
func (n *Navigator) CurrentCorridor() *trackmodel.Section { return n.currentCorridor }

// SignRouter is the traffic-sign router, or nil outside the Obstacles
// Challenge, matching the sign_router property.
func (n *Navigator) SignRouter() *signrouter.SignRouter { return n.signRouter }

// LapsCompleted is the number of laps confirmed completed so far, matching
// the laps_completed property.
func (n *Navigator) LapsCompleted() int { return n.lapsCompleted }

// WaypointIndex is the index of the waypoint currently being driven
// toward, matching _waypoint_index. Exposed because ReplacePath's re-seek
// and the wrap-detection branch are otherwise unobservable from outside.
func (n *Navigator) WaypointIndex() int { return n.waypointIndex }

// Waypoints is the path currently being driven -- the planned centerline,
// or its sign-lane transform once one has been laid over it.
func (n *Navigator) Waypoints() []trackmodel.Waypoint { return slices.Clone(n.waypoints) }

// outgoingBearing is the direction the path points at index, toward its
// next waypoint, matching navigator.py's _outgoing_bearing.
//
// Wraps to waypoint 0 past the end -- the planned path is one canonical lap
// of a closed loop (see ReplacePath), not an open segment.
func outgoingBearing(path []trackmodel.Waypoint, index int) float64 {
	return path[index].BearingTo(path[(index+1)%len(path)])
}

// Step executes one control step, matching CoreNavigator.step: pulls
// current state from the gateway, calculates commands, and pushes them
// back to the gateway.
func (n *Navigator) Step() {
	pose, ok := n.gateway.GetCurrentPose()
	if !ok {
		// No localization available (startup or sensor dropout): stop
		// rather than coast on the last published command.
		n.gateway.PublishDrive(controllers.DriveCommand{})
		n.debug = DebugSnapshot{
			Phase:              PhaseNoPose,
			CommandedSpeedMPS:  new(0.0),
			CommandedSteerNorm: new(0.0),
		}
		return
	}

	robotX, robotY, robotYaw := pose.X, pose.Y, pose.Yaw
	corridor := waypoints.CorridorForPosition(robotX, robotY, n.cfg.CornerMinM, n.cfg.CornerMaxM)
	n.currentCorridor = &corridor

	n.recordPoseTrail(pose)

	// Continue an in-progress escape maneuver until its latched duration
	// elapses, so escapes are real motions rather than single-tick pulses
	// that never clear the wall.
	if n.activeManeuver != nil {
		n.driveActiveManeuver(robotX, robotY, robotYaw, PhaseActiveManeuver)
		return
	}

	// Update the stuck detector -- it runs while actively driving, but not
	// once the robot has reached its final deliberate stop (the
	// open-challenge hold): otherwise a robot correctly holding position at
	// zero velocity would eventually read as "stuck" and reverse itself
	// back out of a completed race.
	//
	// Python also suspends and resets the detector while ParkController is
	// mid reposition; that branch is unreachable here (no park controller,
	// see doc.go) and is omitted rather than stubbed.
	if !n.isHolding() {
		n.stuckDetector.Update(trackmodel.Waypoint{X: robotX, Y: robotY})
		if n.stuckDetector.GetDiagnostics().IsStuck {
			n.handleStuckEscape(robotX, robotY, robotYaw)
			return
		}
	}

	// Lap completion. With no ParkController there is no handoff to defer:
	// the race is over, so hold position from here on.
	if n.lapsCompleted >= n.numLaps {
		n.handleFinish(robotX, robotY, robotYaw)
		return
	}

	if n.handleWaypointWrap(robotX, robotY, robotYaw) {
		return
	}

	// Re-plan the path onto its pass-side lanes if the routed sign layout
	// has changed since the last rebuild. Deliberately at the top of the
	// driving tick rather than inside the router: the rebuilt path has to
	// be in place before crosstrack, the lookahead gate and the target
	// search all read it, and the router only runs after those.
	n.refreshSignLanes()

	rawWP := n.advancePastPassedWaypoints(robotX, robotY, robotYaw)

	percept := n.assessPerception(pose)

	// Check waypoint reached -- against the RAW planned point, not the
	// sign-deformed one: deformation only biases steering near a sign, it
	// must never stall path progression. A sign can pull the steering
	// target sideways by up to the lateral offset, so the robot's real
	// trajectory may never pass within the threshold of the deformed point
	// -- checking that point would freeze the index indefinitely while the
	// sign stays engaged, corrupting every later tick's lookahead search.
	if rawWP.DistanceTo(trackmodel.Waypoint{X: robotX, Y: robotY}) < n.waypointThreshold {
		n.waypointIndex++
		debug := n.baseDebug(robotX, robotY, robotYaw)
		debug.Phase = PhaseWaypointReached
		debug.ForwardClearanceM = new(percept.forwardClearance)
		debug.MinLidarRangeM = percept.minRange
		debug.Risk = new(percept.risk)
		debug.EscapeRisk = new(percept.escapeRisk)
		n.debug = debug
		return
	}

	n.driveNormally(pose, percept)
}

// recordPoseTrail appends a breadcrumb for a retrace-reverse, matching the
// pose-trail block at the top of step().
//
// Recorded on every tick including mid-maneuver, so the trail is a true
// record of where the chassis has physically been -- which is the entire
// basis for reversing along it without rear sensing. See retraceSteer.
// applyPathWallBudget tells the pursuit controller how much drift this path
// can absorb, matching _apply_path_wall_budget.
//
// Measured off the mat's outer walls rather than the corridor-width belief,
// deliberately. WRO moves the inner walls between rounds but the mat's own
// edges are fixed, so the distance from a waypoint to the nearest edge is
// knowable without believing anything -- and it is the outer wall the robot
// keeps hitting, because an under-estimated corridor width biases the
// planned path toward it. Deriving the budget from the belief instead would
// make it wrong in exactly the rounds it matters.
//
// Subtracts the chassis half-width, since the budget is how far the BODY
// may stray, not its centerline.
func (n *Navigator) applyPathWallBudget() {
	if len(n.waypoints) == 0 {
		return
	}
	clearance := math.Inf(1)
	for _, wp := range n.waypoints {
		clearance = math.Min(clearance, math.Min(
			math.Min(wp.X, n.cfg.TrackMaxCoordM-wp.X),
			math.Min(wp.Y, n.cfg.TrackMaxCoordM-wp.Y),
		))
	}
	budget := clearance - n.cfg.ChassisWidthM/2 - n.cfg.WallMarginSafetyM
	budget = math.Max(budget, n.cfg.MinLookaheadTransitionM)
	n.waypointController.SetCrosstrackBudget(&budget)
}

// baseDebug is the fields available on every phase once pose is known --
// the common prefix every Step branch's snapshot builds on, matching
// _base_debug.
func (n *Navigator) baseDebug(robotX, robotY, robotYaw float64) DebugSnapshot {
	snapshot := DebugSnapshot{
		PoseX:         new(robotX),
		PoseY:         new(robotY),
		PoseYaw:       new(robotYaw),
		Direction:     new(n.direction),
		WaypointIndex: new(n.waypointIndex),
		LapsCompleted: n.lapsCompleted,
		NumLaps:       n.numLaps,
	}
	if n.currentCorridor != nil {
		c := *n.currentCorridor
		snapshot.CurrentCorridor = &c
	}
	return snapshot
}

func (n *Navigator) recordPoseTrail(pose trackmodel.Pose) {
	here := trackmodel.Waypoint{X: pose.X, Y: pose.Y}
	if len(n.poseTrail) > 0 {
		last := n.poseTrail[len(n.poseTrail)-1]
		if last.ToWaypoint().DistanceTo(here) < n.cfg.PoseTrailMinStepM {
			return
		}
	}
	n.poseTrail = append(n.poseTrail, pose)
	if n.cfg.PoseTrailLen > 0 && len(n.poseTrail) > n.cfg.PoseTrailLen {
		n.poseTrail = slices.Delete(n.poseTrail, 0, len(n.poseTrail)-n.cfg.PoseTrailLen)
	}
}

// isHolding reports whether the robot has reached a deliberate, terminal
// stop, matching _is_holding. With no ParkController the only terminal
// state is the open-challenge hold, and it is monotonic (never reverts once
// reached), so it is safe to permanently stop running stuck detection here.
func (n *Navigator) isHolding() bool { return n.lapsCompleted >= n.numLaps }
