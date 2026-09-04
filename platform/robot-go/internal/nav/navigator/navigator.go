package navigator

import (
	"errors"
	"fmt"
	"log/slog"
	"math"
	"slices"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/bayexit"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/corridorfollower"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/directionestimator"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/parking"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/startmeasurement"
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
// Step's behavior. Direction is Optional: when nil the navigator runs the
// BLIND_CREEP bootstrap (corridor follower + direction estimator) until the
// travel direction resolves, then hands off to the planned path. When set,
// the blind creep phase is skipped entirely and sighted behavior is preserved.
type Params struct {
	// Gateway is the hardware port every branch of Step reads and
	// publishes through. Required.
	Gateway controllers.HardwareGateway
	// Vision supplies traffic-sign observations; nil means none, which is
	// every Open Challenge run.
	Vision VisionGateway
	// Waypoints is the planned path: one canonical lap of a closed loop.
	Waypoints []trackmodel.Waypoint
	// Direction is the travel direction around the loop. Optional: nil puts
	// the navigator into blind bootstrap (BLIND_CREEP) where the direction
	// is inferred from LIDAR before the planned path is followed. Once set,
	// the escape maneuver consumes it as the fallback side when a LIDAR-only
	// clearance comparison cannot decide one.
	Direction *trackmodel.Direction
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
	// ParkController drives the post-final-lap parking maneuver; nil for
	// the Open Challenge (or any scenario with no parking lot), in which
	// case handleFinish holds position once NumLaps is reached, matching
	// _handle_finish's `pc is None` branch.
	ParkController *parking.ParkController
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
	direction  *trackmodel.Direction

	// parkController drives the post-final-lap parking maneuver; nil for
	// the Open Challenge. parkingEngaged latches once the robot is close
	// enough to the staging point to start it -- see shouldEngageParking.
	parkController *parking.ParkController
	parkingEngaged bool

	// Blind bootstrap state. Nil/empty until the navigator is in blind
	// mode (Direction == nil at construction). dirEstimator settles the
	// travel direction; discovery accumulates camera signs; believedYawOffset
	// is the belief->map yaw measured at start (start_measurement).
	dirEstimator      *directionestimator.Estimator
	discovery         *signrouter.ObservedSignMap
	believedYawOffset float64
	believedYawSet    bool

	// In-bay start. bayStartChecked is tested ONCE (the first blindCreep
	// tick with a scan), matching track_navigator_node.py's
	// _bay_start_checked: re-testing every tick lets it fire mid-creep at a
	// corner and settle the direction off geometry that is not a bay at
	// all. exitingBay latches while BayExit drives the pocket exit; the
	// direction is settled into dirEstimator immediately (so the router and
	// belief-offset math can commit once the exit clears) but n.direction
	// itself is not published until then, so blindCreep keeps running the
	// exit maneuver instead of handing off to normal driving mid-pocket.
	bayStartChecked bool
	exitingBay      bool
	bayExit         *bayexit.BayExit

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

	// The contact zone is per-challenge, and an attached sign router is what
	// identifies the Obstacles Challenge -- the same test CoreNavigator uses.
	// Resolved onto the COLLISION controller alone: it is that controller's
	// assess_risk that fires the escape, and widening the zone for the
	// waypoint controller or the stuck detector would change things the
	// measurement behind the override never covered.
	collisionCfg := p.ControllersConfig
	if p.SignRouter != nil {
		collisionCfg = collisionCfg.ForObstaclesChallenge()
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
		parkController:      p.ParkController,
		direction:           p.Direction,
		waypointThreshold:   p.Config.MainLoopReachedDistanceM,
		escapeSteerSign:     1.0,
		waypointController:  p.ControllersConfig.NewWaypointController(),
		collisionController: collisionCfg.NewCollisionAvoidanceController(),
		stuckDetector:       stuckDetector,
	}
	// Blind bootstrap: build the direction estimator and (when a router is
	// attached) the discovery map so camera signs accumulate while creeping.
	if n.direction == nil {
		n.dirEstimator = directionestimator.NewEstimator(
			directionestimator.DefaultConfig().MinVotes,
		)
		if n.signRouter != nil {
			n.discovery = signrouter.NewObservedSignMap(
				signrouter.DefaultDiscoveryConfig(),
				n.signRouter,
			)
		}
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

// ParkController is the post-final-lap parking maneuver, or nil for the
// Open Challenge (or any scenario with no parking lot), matching the
// park_controller property.
func (n *Navigator) ParkController() *parking.ParkController { return n.parkController }

// Direction is the round's travel direction, nil while a blind round's
// bootstrap has not settled one yet. Exposed because the host's layout-belief
// loop (internal/nav/widthbelief) cannot attribute a corridor reading without
// it, and must not guess: attributing to the wrong section folds a
// measurement of one corridor into another's estimate.
func (n *Navigator) Direction() *trackmodel.Direction { return n.direction }

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

	// Blind bootstrap: while the travel direction is unknown, creep along the
	// corridor (centred between visible walls) and infer the direction from
	// LIDAR, accumulating any camera sign detections. Once the direction
	// settles, hand off to the planned path. Sighted runs (Direction set at
	// construction) never reach this branch.
	if n.direction == nil {
		n.blindCreep(robotX, robotY, robotYaw)
		return
	}

	// Update the stuck detector -- it runs while actively driving, but not
	// once the robot has reached its final deliberate stop (the
	// open-challenge hold, or parking done): otherwise a robot correctly
	// holding position at zero velocity would eventually read as "stuck"
	// and reverse itself back out of a completed race.
	//
	// Also suspended and RESET while ParkController is mid
	// reverse-and-reorient recovery (IsRepositioning): that maneuver is
	// itself a deliberate, low-net-displacement reverse burst, and the
	// generic escape it would otherwise trigger is blind to the inner
	// keep-out block ParkController is navigating around. Reset, not just
	// skipped, so the history queue does not span across the gap and
	// reintroduce the same false "stuck" trigger one tick later.
	if n.parkController != nil && n.parkController.IsRepositioning() {
		n.stuckDetector.Reset()
	} else if !n.isHolding() {
		n.stuckDetector.Update(trackmodel.Waypoint{X: robotX, Y: robotY})
		if n.stuckDetector.GetDiagnostics().IsStuck {
			n.handleStuckEscape(robotX, robotY, robotYaw)
			return
		}
	}

	// Lap completion: defer the parking handoff until the robot is
	// actually in the parking corridor and within reach of the staging
	// point (see handleFinish/shouldEngageParking). Until then keep
	// navigating so the handoff never fires mid-corridor.
	if n.lapsCompleted >= n.numLaps && n.handleFinish(robotX, robotY, robotYaw) {
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
		Direction:     n.direction,
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
// stop, matching _is_holding. Both terminal states (the open-challenge
// hold, and a done ParkController) are monotonic -- never revert once
// reached -- so it is safe to permanently stop running stuck detection once
// this is true.
func (n *Navigator) isHolding() bool {
	if n.lapsCompleted < n.numLaps {
		return false
	}
	return n.parkController == nil || n.parkController.IsDone()
}

// BelievedYawOffset returns the belief->map yaw measured at start (the
// start_measurement believed-offset), or ok=false until ApplyBelievedStart
// has been called. Match for the MCAP belief-stream plan: the localizer's
// heading can be a rigid rotation off truth at start, and sign routing /
// direction inference must know it.
func (n *Navigator) BelievedYawOffset() (offset float64, ok bool) {
	return n.believedYawOffset, n.believedYawSet
}

// ApplyBelievedStart records the belief->map yaw offset from a measured start
// pose, matching start_measurement's believed-start offset: the difference
// between the robot's current heading estimate and the measured one. Corrects
// the estimator heading via the gateway so subsequent poses are in the map
// frame.
func (n *Navigator) ApplyBelievedStart(measured trackmodel.Pose, pose trackmodel.Pose) {
	offset := navutil.WrapAngle(pose.Yaw - measured.Yaw)
	n.believedYawOffset = offset
	n.believedYawSet = true
	if math.Abs(offset) > 1e-9 {
		n.gateway.CorrectHeadingForDirectionChange(offset)
	}
}

// blindCreep drives the BLIND_CREEP phase: creep along the corridor centred
// between visible walls, infer the travel direction from LIDAR, and accumulate
// camera sign detections, matching CoreNavigator.step's blind bootstrap. Once
// the direction settles (parking-bay read or enough agreeing scans), the
// navigator adopts it and the next tick follows the planned path.
func (n *Navigator) blindCreep(robotX, robotY, robotYaw float64) {
	debug := n.baseDebug(robotX, robotY, robotYaw)
	debug.Phase = PhaseBlindCreep

	scan, haveScan := n.gateway.GetLidarScan()
	var ranges, angles []float64
	if haveScan {
		ranges, angles = scan.RangesM, scan.AnglesRad
	}

	// Accumulate camera sign detections into the discovery map (discover
	// mode), so signs are published to the router as they confirm.
	if n.discovery != nil {
		n.discovery.Observe(n.visionDetections(), trackmodel.Waypoint{X: robotX, Y: robotY})
		n.discovery.Publish()
	}

	// Resolve the direction: a boxed-in parking bay names it outright;
	// otherwise vote on scans. The parking-bay check is tested ONCE (the
	// first tick a scan is available) -- see bayStartChecked's doc comment;
	// re-testing every tick lets it fire mid-creep at a corner and settle
	// the direction off geometry that is not a bay at all.
	if n.dirEstimator != nil {
		boxed := false
		if !n.bayStartChecked && haveScan {
			n.bayStartChecked = true
			if dir, ok := directionestimator.DirectionFromParkingBay(ranges, angles, directionestimator.DefaultConfig()); ok {
				n.dirEstimator.Settle(dir)
				n.exitingBay = true
				boxed = true
			}
		}
		if !boxed && !n.exitingBay && haveScan {
			n.dirEstimator.Observe(ranges, angles, robotYaw, directionestimator.DefaultConfig())
		}

		// Out of the pocket. Falls through to the settle block below rather
		// than returning, so the path is rebuilt for the committed
		// direction once the maneuver ends -- see bayexit.IsClear.
		bxCfg := bayexit.DefaultConfig()
		if n.exitingBay && haveScan && bayexit.IsClear(ranges, angles, bxCfg) {
			n.exitingBay = false
		}
		if n.exitingBay {
			odom, odomOK := n.gateway.GetWheelOdometry()
			if !odomOK {
				// No odometry means the reverse leg cannot be bounded, and
				// this maneuver reverses toward a fin. Hold rather than
				// guess.
				n.gateway.PublishDrive(controllers.DriveCommand{})
				debug.CommandedSpeedMPS = new(0.0)
				debug.CommandedSteerNorm = new(0.0)
				n.debug = debug
				return
			}
			if n.bayExit == nil {
				n.bayExit = bayexit.New()
			}
			cmd := n.bayExit.Command(ranges, angles, odom.DistanceM, n.cfg.CreepSpeedMPS(), bxCfg)
			n.gateway.PublishDrive(cmd)
			debug.CommandedSpeedMPS = new(cmd.SpeedMPS)
			debug.CommandedSteerNorm = new(cmd.SteeringNorm)
			n.debug = debug
			return
		}

		if dir, ok := n.dirEstimator.Direction(); ok {
			n.direction = &dir
			// With the direction known, measure the start so the map frame
			// is corrected before the planned path is followed.
			if haveScan {
				if measured, measuredOK := startmeasurement.MeasureStartPose(
					ranges, angles, dir, trackmodel.South, startmeasurement.DefaultConfig(),
				); measuredOK {
					n.ApplyBelievedStart(
						trackmodel.Pose{X: measured.X, Y: measured.Y, Yaw: robotYaw},
						trackmodel.Pose{X: robotX, Y: robotY, Yaw: robotYaw},
					)
				}
			}
			n.debug = debug
			return
		}
	}

	// No direction yet: creep along the corridor. Without a scan there is
	// nothing to react to, so hold still rather than guess.
	if !haveScan {
		n.gateway.PublishDrive(controllers.DriveCommand{})
		debug.CommandedSpeedMPS = new(0.0)
		debug.CommandedSteerNorm = new(0.0)
		n.debug = debug
		return
	}
	yaw := robotYaw
	cmd := corridorfollower.FollowCorridor(ranges, angles, corridorfollower.Params{
		SpeedMPS: n.cfg.CreepSpeedMPS(),
		Yaw:      &yaw,
	}, corridorfollower.DefaultConfig())
	n.gateway.PublishDrive(cmd)
	debug.CommandedSpeedMPS = new(cmd.SpeedMPS)
	debug.CommandedSteerNorm = new(cmd.SteeringNorm)
	n.debug = debug
}
