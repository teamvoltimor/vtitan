package navigator

import (
	"errors"
	"fmt"
	"log/slog"
	"math"
	"slices"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
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

// ReplacePath swaps in a new planned path mid-run, resuming at the nearest
// point, matching replace_path.
//
// The waypoint index cannot carry over: the new path has its own indexing
// and the old index would point somewhere arbitrary on it. Re-seeking to
// the nearest waypoint keeps progress instead of restarting the lap, and
// matters because the paths differ by centimeters, not corridors.
//
// Nearest-by-position alone can go wrong near a corner, where several
// waypoints sit at almost the same distance while pointing in very
// different directions, and the robot's heading at that instant does not
// always match the path's local direction there yet. Picking purely by
// position can then hand WaypointController a point past the turn,
// demanding a correction far larger than finishing the corner needs --
// measured on real hardware as a ~193 deg swing where ~90 deg would do.
// Passing a non-nil robotYaw re-ranks the near-tied-by-distance candidates
// (ReplanHeadingTieMarginM) by heading agreement instead. Pass nil where
// the robot has been tracking a path very similar to the new one, where
// nearest-by-position alone is already safe.
func (n *Navigator) ReplacePath(path []trackmodel.Waypoint, robotXY trackmodel.Waypoint, robotYaw *float64) {
	if len(path) == 0 {
		return
	}
	previousIndex := n.waypointIndex
	n.waypoints = slices.Clone(path)
	// A replanned path is a new centerline, so the lanes have to be laid
	// over it again -- and the fingerprint cleared, or the unchanged sign
	// layout would read as "already applied" and leave the new path bare.
	n.laneBaseWaypoints = slices.Clone(path)
	n.laneFingerprint, n.laneFingerprintSet = nil, false
	n.applyPathWallBudget()

	distances := make([]float64, len(path))
	nearestIndex := 0
	for i, wp := range path {
		distances[i] = wp.DistanceTo(robotXY)
		if distances[i] < distances[nearestIndex] {
			nearestIndex = i
		}
	}

	if robotYaw != nil {
		margin := distances[nearestIndex] + n.cfg.ReplanHeadingTieMarginM
		bestError := math.Inf(1)
		best := nearestIndex
		for i, d := range distances {
			if d > margin {
				continue
			}
			headingError := math.Abs(navutil.WrapAngle(outgoingBearing(path, i) - *robotYaw))
			if headingError < bestError {
				best, bestError = i, headingError
			}
		}
		nearestIndex = best
	}

	n.waypointIndex = nearestIndex

	// A large forward jump is never earned progress -- the robot cannot
	// skip most of a lap between two ticks. It means the re-seek landed on
	// the far side of the start/finish seam: the path was rebuilt running
	// the other way, leaving the robot just BEHIND the new waypoint 0,
	// which on a closed loop is also the tail of the list. The tail it is
	// about to drive belongs to a lap it never ran, so the wrap at the end
	// of it is the entry into lap 1, not the completion of it.
	if n.waypointIndex-previousIndex > len(path)/2 {
		n.suppressNextWrap = true
	}
}

// ReplaceSignRouter swaps in a sign router built for a new race, matching
// replace_sign_router.
//
// The state machine can cycle FINISHED -> BOOT_CHECK -> READY -> RACING
// purely from the button, so a process built once as Open (nil router) must
// be able to pick up a switch to Obstacles, and vice versa, without a
// restart. The previous router's own committed state does not carry over --
// the caller builds a fresh one from the current section/direction/tuning.
func (n *Navigator) ReplaceSignRouter(router *signrouter.SignRouter) {
	n.signRouter = router
	// Whatever lanes the previous router's layout produced belong to that
	// race. Drop back to the planned centerline and let the next tick
	// re-lane from the new router, if there is one.
	n.waypoints = slices.Clone(n.laneBaseWaypoints)
	n.laneFingerprint, n.laneFingerprintSet = nil, false
}

// SetTravelDirection adopts the (re-)inferred travel direction, matching
// set_travel_direction. Consumed by the collision-avoidance escape maneuver
// as the fallback side when a LIDAR-only clearance comparison cannot decide
// one.
func (n *Navigator) SetTravelDirection(direction trackmodel.Direction) {
	n.direction = direction
}

// Reset clears per-race state so a new race starts as if this were the
// first, matching reset.
//
// Needed because the state machine can cycle FINISHED -> BOOT_CHECK ->
// READY -> RACING purely from the physical button, with no process restart
// -- so nothing else re-creates this object between races. Without this,
// LapsCompleted alone would stay at its previous value and the very first
// tick of the new race would immediately read as already finished.
func (n *Navigator) Reset() {
	n.waypointIndex = 0
	n.lapsCompleted = 0
	n.suppressNextWrap = false
	n.activeManeuver = nil
	n.maneuverFramesLeft = 0
	n.escapeCount = 0
	n.escapeSteerSign = 1.0
	n.escapeSequenceStartXY = nil
	n.stuckDetector.Reset()
	n.waypointController.Reset()
	if n.signRouter != nil {
		n.signRouter.ResetForNewLap()
	}
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
			CommandedSpeedMPS:  ptr(0.0),
			CommandedSteerNorm: ptr(0.0),
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
		debug.ForwardClearanceM = ptr(percept.forwardClearance)
		debug.MinLidarRangeM = percept.minRange
		debug.Risk = ptr(percept.risk)
		debug.EscapeRisk = ptr(percept.escapeRisk)
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
		PoseX:         ptr(robotX),
		PoseY:         ptr(robotY),
		PoseYaw:       ptr(robotYaw),
		Direction:     ptr(n.direction),
		WaypointIndex: ptr(n.waypointIndex),
		LapsCompleted: n.lapsCompleted,
		NumLaps:       n.numLaps,
	}
	if n.currentCorridor != nil {
		snapshot.CurrentCorridor = ptr(*n.currentCorridor)
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

// handleFinish handles the post-final-lap phase, matching _handle_finish's
// `pc is None` branch: the Open Challenge has no parking maneuver, so the
// robot holds position. Python's version returns a bool because the parking
// branch can decline to act; that branch does not exist here, so this
// always issues a command and the caller always stops this tick.
func (n *Navigator) handleFinish(robotX, robotY, robotYaw float64) {
	n.gateway.PublishDrive(controllers.DriveCommand{})
	debug := n.baseDebug(robotX, robotY, robotYaw)
	debug.Phase = PhaseFinishedHold
	debug.CommandedSpeedMPS = ptr(0.0)
	debug.CommandedSteerNorm = ptr(0.0)
	n.debug = debug
}

// handleWaypointWrap detects the index running off the end of the lap and
// counts the lap, matching step()'s waypoint-wrap branch. Returns true when
// the tick is finished.
//
// This is the ONLY lap-counting mechanism in this port. Python prefers a
// LapDetector's geometric confirmation and falls back to counting wraps
// directly; LapDetector has no Go equivalent (see doc.go), so the fallback
// branch is always the one taken.
func (n *Navigator) handleWaypointWrap(robotX, robotY, robotYaw float64) bool {
	if n.waypointIndex < len(n.waypoints) {
		return false
	}
	n.waypointIndex = 0
	n.stuckDetector.Reset()
	if n.suppressNextWrap {
		// Seeded past the seam by ReplacePath, not driven -- see there.
		n.suppressNextWrap = false
		return false
	}

	n.lapsCompleted++
	n.logger.Info("lap complete (waypoint-only fallback)", "laps_completed", n.lapsCompleted)
	if n.signRouter != nil {
		n.signRouter.ResetForNewLap()
	}
	debug := n.baseDebug(robotX, robotY, robotYaw)
	debug.Phase = PhaseWaypointWrapFallback
	n.debug = debug
	return true
}

// advancePastPassedWaypoints walks the index past any waypoint the robot
// has already gone by and returns the waypoint now being driven toward,
// matching step()'s advance loop.
//
// Not just one the robot happens to pass within the threshold of: a robot
// that starts somewhere other than exactly on the planned centerline can
// curve past a waypoint without ever entering that radius; left
// un-advanced, the lookahead search keeps re-targeting that same,
// increasingly stale point long after the robot has passed it -- and once
// far enough away, that stale point can itself satisfy the search's
// lookahead-distance test and be selected as the steering target even
// though it is now behind the robot.
//
// The comparison wraps around the seam, so the LAST waypoint gets the same
// pass-by rescue as every other one. Stopping at index+1 < len left
// entering the reached-distance circle as the only way past the final
// point, and a robot running wider than that radius never gets past it,
// never wraps, and so never completes a lap: measured on the 2026-08-06
// counterclockwise round as a robot circling the mat for seven minutes with
// the lap count stuck at zero.
//
// The BEHIND test is an Obstacles-only extension (gated on the router's
// presence and StaleTargetRescue): a robot cutting a corner sharply enough
// can leave both the current and next waypoint reading farther away every
// tick, even though local-frame ahead/behind already shows the chassis has
// swept past them.
func (n *Navigator) advancePastPassedWaypoints(robotX, robotY, robotYaw float64) trackmodel.Waypoint {
	here := trackmodel.Waypoint{X: robotX, Y: robotY}
	rawWP := n.waypoints[n.waypointIndex]

	rescueBehind := n.signRouter != nil && n.cfg.StaleTargetRescue
	cosYaw, sinYaw := 0.0, 0.0
	if rescueBehind {
		cosYaw, sinYaw = math.Cos(robotYaw), math.Sin(robotYaw)
	}

	count := len(n.waypoints)
	for range count {
		nextIndex := n.waypointIndex + 1
		nextWP := n.waypoints[nextIndex%count]
		nextCloser := nextWP.DistanceTo(here) < rawWP.DistanceTo(here)
		rawBehind := rescueBehind && (rawWP.X-robotX)*cosYaw+(rawWP.Y-robotY)*sinYaw <= 0
		if !nextCloser && !rawBehind {
			break
		}
		n.waypointIndex = nextIndex
		if nextIndex >= count {
			// Seam crossed. Leave rawWP on the final waypoint and let the
			// wrap branch count the lap next tick -- walking on into the
			// new lap here would skip waypoints the wrap is about to
			// rewind to.
			break
		}
		rawWP = nextWP
	}
	return rawWP
}

// assessPerception reads the LIDAR and derives forward clearance and both
// risk levels, matching step()'s scan block.
func (n *Navigator) assessPerception(pose trackmodel.Pose) perception {
	scan, haveScan := n.gateway.GetLidarScan()
	p := perception{scan: scan, haveScan: haveScan}

	if haveScan {
		// Converted to a BUMPER gap once, here, rather than at each of the
		// comparisons below: the no-LIDAR fallback assigns a threshold
		// value to this same variable, so the two branches have to leave
		// it in one frame or the degraded path means something different
		// from the measured one.
		p.forwardClearance = controllers.BumperGapAhead(
			n.collisionController.ComputeForwardClearance(scan.RangesM, scan.AnglesRad),
			n.cfg.LidarToFrontBumperM,
		)
		p.risk = n.collisionController.AssessRisk(scan.RangesM, scan.AnglesRad)
		p.escapeRanges = scan.RangesM
		if len(scan.RangesM) > 0 {
			p.minRange = ptr(slices.Min(scan.RangesM))
		}
	} else {
		// No LIDAR: a degraded sensor is not open road. Drive cautiously
		// (slow zone + non-SAFE risk) instead of blasting forward blind.
		p.forwardClearance = n.cfg.SlowDistM
		p.risk = controllers.RiskObstacle
	}
	p.escapeRisk = p.risk

	if haveScan && n.signRouter != nil {
		mapped := make([]controllers.MappedObstacle, 0, len(n.signRouter.RoutedSignPositionsByCorridor()))
		for _, sign := range n.signRouter.RoutedSignPositionsByCorridor() {
			mapped = append(mapped, controllers.MappedObstacle{Position: sign.Waypoint, Corridor: sign.Corridor})
		}
		p.escapeRanges = controllers.MaskMappedObstacles(
			scan.RangesM, scan.AnglesRad, pose, mapped,
			n.cfg.EscapeMaskRadiusM, n.cfg.CornerMinM, n.cfg.CornerMaxM,
		)
		p.escapeRisk = n.collisionController.AssessRisk(p.escapeRanges, scan.AnglesRad)
	}
	return p
}

// driveNormally is step()'s tail: steering selection, the speed ladder, the
// sign-contact evade, the escape trigger, and the normal publish.
func (n *Navigator) driveNormally(pose trackmodel.Pose, p perception) {
	robotX, robotY, robotYaw := pose.X, pose.Y, pose.Yaw
	here := trackmodel.Waypoint{X: robotX, Y: robotY}

	// Steer at a lookahead point, not directly at the (often much closer)
	// next waypoint -- otherwise the lookahead distance is computed but
	// discarded, producing weave on straights and corner cutting.
	//
	// Lookahead selection is gated on crosstrack error (how far off the
	// planned path the robot actually is), not forward LIDAR clearance.
	crosstrack := trackmodel.CrossTrackError(n.waypoints, robotX, robotY)
	// Crosstrack alone arms the short lookahead only after a corner has
	// been missed; the path's own upcoming turn arms it on entry.
	turnAhead := trackmodel.PathTurnAhead(n.waypoints, n.waypointIndex, n.cfg.CornerPreviewDistanceM)
	signAhead := n.signAhead(robotX, robotY, robotYaw)
	lookahead := n.waypointController.SelectLookahead(crosstrack, turnAhead, signAhead)
	// Full waypoint list, not a slice from waypointIndex -- SelectTargetPoint
	// wraps the search around the lap itself; slicing here would cut that
	// wraparound off again.
	steerTarget := n.waypointController.SelectTargetPoint(here, robotYaw, n.waypoints, n.waypointIndex, lookahead)

	steerTarget, signDeformMagnitude, activeSignCount := n.applySignRouting(steerTarget, here, robotYaw)

	dt := 0.0
	if n.cfg.ControlHz > 0 {
		dt = 1.0 / n.cfg.ControlHz
	}
	steering, _, angleError := n.waypointController.ComputeSteering(here, robotYaw, steerTarget, crosstrack, dt)

	speed, clearanceSpeed, headingSpeed := n.selectSpeed(p, angleError, turnAhead, signDeformMagnitude)

	debug := n.baseDebug(robotX, robotY, robotYaw)
	debug.ForwardClearanceM = ptr(p.forwardClearance)
	debug.MinLidarRangeM = p.minRange
	debug.Risk = ptr(p.risk)
	debug.EscapeRisk = ptr(p.escapeRisk)
	debug.CrosstrackErrorM = ptr(crosstrack)
	debug.LookaheadDistance = ptr(lookahead)
	debug.PathTurnAheadRad = ptr(turnAhead)
	debug.SteerTargetX = ptr(steerTarget.X)
	debug.SteerTargetY = ptr(steerTarget.Y)
	debug.AngleErrorRad = ptr(angleError)
	debug.ClearanceSpeedMPS = ptr(clearanceSpeed)
	debug.HeadingSpeedMPS = ptr(headingSpeed)
	debug.SignDeformMagnitudeM = signDeformMagnitude
	debug.ActiveSignCount = activeSignCount

	// Last-resort geometric guard against clipping a routed sign. The two
	// responses that already exist both assume the planner has the sign
	// handled -- the escape mask suppresses any reaction to it, and without
	// that mask the generic escape reverses and swings, which in a 1.0 m
	// corridor trades sign strikes for wall strikes. That assumption holds
	// sighted, where the lane is placed a corridor ahead.
	//
	// Deliberately NOT gated on LIDAR risk: a return reads CRITICAL only at
	// contact range, too late for any steering command to matter, which is
	// why a risk-gated version of this measured flat.
	if n.cfg.SignContactEvade && n.signRouter != nil {
		if evade, evading := n.signEvadeSteer(robotX, robotY, robotYaw); evading {
			steering = navutil.Clamp(steering+evade, -1.0, 1.0)
			speed = math.Min(speed, n.cfg.CreepSpeedMPS())
		}
	}

	if n.tryEscape(pose, p, debug) {
		return
	}

	// Normal publish -- clear the escape escalation, but only once the
	// robot has actually moved since the sequence started. A single
	// normal-drive tick between escape attempts does not mean the escape
	// worked: confirmed on real hardware 2026-08-04, normal_drive ->
	// escape_triggered alternated for 34+ seconds with the robot pinned in
	// place, and an unconditional reset zeroed the count every cycle so it
	// never reached EscalateAfterAttempts.
	if n.escapeSequenceStartXY == nil ||
		math.Hypot(robotX-n.escapeSequenceStartXY.X, robotY-n.escapeSequenceStartXY.Y) >= n.cfg.StuckMoveThreshold {
		n.escapeCount = 0
		n.escapeSequenceStartXY = nil
	}
	n.gateway.PublishDrive(controllers.DriveCommand{SpeedMPS: speed, SteeringNorm: steering})
	debug.Phase = PhaseNormalDrive
	debug.CommandedSpeedMPS = ptr(speed)
	debug.CommandedSteerNorm = ptr(steering)
	debug.EscapeCount = ptr(n.escapeCount)
	n.debug = debug
}

// signAhead reports whether a routed sign is within activation distance and
// ahead of the chassis, the third preview signal alongside crosstrack and
// turn-ahead.
//
// Crosstrack is measured against the raw path, so it never rises during a
// sign pass (the deformation biases the SEARCH's output, not the path the
// search is judged against). RoutedSignPositions is read-only, so this is
// safe to query before DeformWaypoint runs later this tick.
//
// Restricted to signs actually AHEAD along the chassis heading:
// RoutedSignPositions has no direction filter, so without this a
// not-yet-passed sign still alongside or just behind the chassis forces the
// short lookahead just as readily as a genuinely upcoming one.
func (n *Navigator) signAhead(robotX, robotY, robotYaw float64) bool {
	if !n.cfg.SignAwareLookahead || n.signRouter == nil {
		return false
	}
	cosYaw, sinYaw := math.Cos(robotYaw), math.Sin(robotYaw)
	for _, wp := range n.signRouter.RoutedSignPositions() {
		dx, dy := wp.X-robotX, wp.Y-robotY
		if math.Hypot(dx, dy) < n.cfg.ActivationDistM && dx*cosYaw+dy*sinYaw > 0 {
			return true
		}
	}
	return false
}

// applySignRouting runs the sign router over the point steering will
// actually chase, matching step()'s deform block.
//
// Deforming a raw-path CANDIDATE before the lookahead search picked from it
// meant the search, not the sign, decided whether the nudge ever reached
// steering (it almost never did: waypoints are spaced well under the
// lookahead, so the search kept skipping past a single deformed candidate).
// Deforming the search's own output guarantees the bias is exactly what
// gets steered toward.
//
// The router is CALLED even when SignLaneSuppressDeform discards its
// output: it owns engage/pass bookkeeping and RoutedSignPositions (which
// the escape mask reads), none of which the lane transform replaces.
func (n *Navigator) applySignRouting(
	steerTarget, here trackmodel.Waypoint, robotYaw float64,
) (target trackmodel.Waypoint, deformMagnitude *float64, activeSigns *int) {
	if n.signRouter == nil || n.currentCorridor == nil {
		return steerTarget, nil, nil
	}
	observations := n.visionDetections()
	deformed := n.signRouter.DeformWaypoint(steerTarget, here, robotYaw, *n.currentCorridor, observations)
	magnitude := math.Hypot(deformed.X-steerTarget.X, deformed.Y-steerTarget.Y)

	suppress := n.cfg.SignLanePlanner && n.cfg.SignLaneSuppressDeform
	if !suppress {
		steerTarget = deformed
	}
	return steerTarget, ptr(magnitude), ptr(n.signRouter.ActiveSignCount())
}

// visionDetections returns this tick's traffic-sign observations, or an
// empty slice when there is no vision port or it has nothing to report --
// which is what Python's gateway returns in the same situation.
func (n *Navigator) visionDetections() []signrouter.TrafficSignObservation {
	if n.vision == nil {
		return nil
	}
	observations, ok := n.vision.GetVisionDetections()
	if !ok {
		return nil
	}
	return observations
}

// selectSpeed runs the speed ladder, matching step()'s speed block, and
// returns the commanded speed alongside the two attribution values the
// debug snapshot reports (the clearance-only and heading-only choices).
//
// Python also caps the first lap of a DISCOVERING run at
// EXPLORE_LAP_SPEED_FRAC of the ceiling. That branch is gated on
// SignRouter.is_discovering, which this port does not have (blind discovery
// is out of scope -- see signrouter's doc.go), so it is omitted rather than
// approximated by a condition that would fire on sighted runs too. It is
// inert by default in any case: EXPLORE_LAP_SPEED_FRAC ships at 1.0.
func (n *Navigator) selectSpeed(
	p perception, angleError, turnAhead float64, signDeformMagnitude *float64,
) (speed, clearanceSpeed, headingSpeed float64) {
	switch {
	case p.forwardClearance < n.cfg.ContactDistM:
		speed = n.cfg.CreepSpeedMPS()
	case p.forwardClearance < n.cfg.SlowDistM:
		speed = n.cfg.SlowSpeedMPS()
	case p.forwardClearance < n.cfg.MediumDistM:
		speed = n.cfg.MediumSpeedMPS()
	default:
		speed = n.cfg.FastSpeedMPS()
	}
	// Captured before the heading limiter, the envelope clamp and the risk
	// cap all fold into speed. Reporting the post-min value under this name
	// made the two debug fields satisfy final <= heading by construction, so
	// the heading limiter looked innocent on 100% of ticks while it was in
	// fact the binding constraint on most of them.
	clearanceSpeed = speed

	// Never take a sharp turn at a speed the steering actuator cannot keep
	// up with: the servo has a fixed slew rate independent of chassis
	// speed, so a big required correction taken at full speed demands a yaw
	// rate it cannot track -- it saturates, overshoots and oscillates.
	// Clearance alone never catches this: a corner can have 0.50 m+ of open
	// space ahead while still demanding a 90-180 deg correction.
	//
	// Only the CRAWL threshold reduces speed. This ladder briefly mirrored
	// the clearance one; measured on hardware 2026-08-09 that cost 33% of
	// lap time for nothing, since ordinary cornering sits at 23-45 deg of
	// heading error, so the middle rungs taxed every corner on the track
	// rather than catching a dangerous case.
	headingSpeed = n.cfg.FastSpeedMPS()
	if math.Abs(angleError) >= n.cfg.CrawlRad {
		headingSpeed = n.cfg.CreepSpeedMPS()
	}
	speed = math.Min(speed, headingSpeed)

	// Bound the selected cruise speed by the configured envelope. Applied
	// HERE, to a zone speed that is always positive, and not to the final
	// command: clamping that up to the floor would turn every legitimate
	// stop (escape handoff, blocked at both ends) into a crawl the robot
	// cannot be commanded out of.
	speed = math.Min(math.Max(speed, n.cfg.MinSpeedMPS()), n.cfg.MaxSpeedMPS())

	// Never blast past a non-forward obstacle (e.g. a sign alongside the
	// robot) just because the path ahead is clear.
	if p.risk != controllers.RiskSafe {
		speed = math.Min(speed, n.cfg.SlowSpeedMPS())
	}

	// Give the pursuit controller more time to close a sign-avoidance
	// offset. Neither clearance nor heading-error speed reacts to one: a
	// deformation biases the STEERING TARGET sideways without necessarily
	// shrinking forward clearance or growing heading error. Gated on the
	// deformation the router actually applied THIS tick, not proximity to a
	// sign, so it only fires while a correction is genuinely in flight.
	if n.cfg.SignAwareSpeed && signDeformMagnitude != nil &&
		*signDeformMagnitude > n.cfg.SignDeformSpeedThresholdM {
		speed = math.Min(speed, n.cfg.SlowSpeedMPS())
	}

	// First-lap corner caution. A mixed-width corner's PLANNED arc is safe
	// by construction (verified 2026-08-28: clearance to both outer walls
	// never drops below what the straights already have), but real hardware
	// wedged at exactly this kind of corner anyway, which points at CONTROL
	// tracking error eating the plan's margin, not the plan itself. Only
	// the first lap has never actually been driven.
	if n.lapsCompleted == 0 && turnAhead != 0 {
		speed = math.Min(speed, n.cfg.SlowSpeedMPS())
	}
	return speed, clearanceSpeed, headingSpeed
}

// signEvadeSteer is the steering that swings the chassis clear of a routed
// sign it is about to clip, matching _sign_evade_steer.
//
// PREDICTS the contact from geometry rather than waiting for the LIDAR to
// call it CRITICAL. That distinction is the whole mechanism: a return only
// reads CRITICAL at contact range, by which point the chassis is
// essentially already touching. Here the trigger is the sign's own
// along-track distance and lateral clearance, both known meters in advance
// because the router is already tracking the sign's position.
//
// Returns ok=false unless a routed sign is genuinely ahead, within
// SignContactDistM, and predicted to pass closer than the chassis and sign
// half-widths allow -- so a sign the robot is already clearing cleanly is
// never answered with a swerve.
//
// The direction comes from the sign's own bearing, not the router's
// pass-side rule. By this point the rule has failed; which side the robot
// ends up on is a scoring question, contact is a run-ending one.
func (n *Navigator) signEvadeSteer(robotX, robotY, robotYaw float64) (steer float64, ok bool) {
	if n.signRouter == nil {
		return 0, false
	}
	cosYaw, sinYaw := math.Cos(robotYaw), math.Sin(robotYaw)
	// Half-widths, plus the chassis's own: how close the centers may pass.
	needed := n.cfg.ChassisWidthM/2 + n.cfg.SignWidthM/2

	worstAhead, worstLateral, found := 0.0, 0.0, false
	for _, wp := range n.signRouter.RoutedSignPositions() {
		dx, dy := wp.X-robotX, wp.Y-robotY
		ahead := dx*cosYaw + dy*sinYaw
		if ahead <= 0.0 || ahead > n.cfg.SignContactDistM {
			continue
		}
		lateral := -dx*sinYaw + dy*cosYaw
		if math.Abs(lateral) >= needed {
			continue // already going to clear it
		}
		if !found || ahead < worstAhead {
			worstAhead, worstLateral, found = ahead, lateral, true
		}
	}
	// Positive lateral puts the sign to the LEFT, so steer right. A sign
	// dead ahead (lateral 0) still has to be resolved to a side; take the
	// one the ordinary steering is already favoring.
	if !found || worstLateral == 0.0 {
		return 0, false
	}
	return -math.Copysign(n.cfg.SignContactSteerNorm(), worstLateral), true
}

// tryEscape fires an escape maneuver when the masked scan reads CRITICAL,
// matching step()'s escape block. Returns true when it took over the tick.
//
// Judged on the masked scan, so a mapped sign cannot trigger one, and
// steered by the masked scan too: the threat this escape is running from is
// by construction not the sign.
func (n *Navigator) tryEscape(pose trackmodel.Pose, p perception, debug DebugSnapshot) bool {
	if p.escapeRisk != controllers.RiskCritical || !p.haveScan || p.escapeRanges == nil {
		return false
	}

	escapeClearances := controllers.ClearancesFromScan(
		controllers.LidarScan{RangesM: p.escapeRanges, AnglesRad: p.scan.AnglesRad},
		n.collisionController,
		n.collisionController.ThreatHalfFovRad,
		controllers.AggregateMin,
	)
	threatDir := controllers.ThreatDirectionFrom(escapeClearances, n.collisionController.ThreatNoDetectionRangeM)
	maneuver, haveManeuver := n.collisionController.ComputeEscapeManeuver(
		p.escapeRisk, threatDir, p.escapeRanges, p.scan.AnglesRad, &n.direction,
	)

	// Rear clearance is checked against the RAW scan: a sign behind the
	// robot is still something to not reverse into, whoever owns it.
	// Retrace instead of swinging, when asked and when there is enough
	// trail to aim at. Obstacles-only by construction: gated on the
	// router's presence, so Open Challenge's escape behavior is untouched
	// regardless of the flag.
	_, canRetrace := n.retraceSteer(pose.X, pose.Y, pose.Yaw)
	n.retracing = haveManeuver && maneuver.Speed < 0 && n.cfg.RetraceEscape && n.signRouter != nil && canRetrace

	if haveManeuver && n.reversingIntoUnseenWall(maneuver, p.scan) {
		// Blocked at both ends: fall through to the capped creep-speed
		// publish rather than backing into an unseen wall. The stuck
		// detector is the backstop if the robot truly cannot move.
		haveManeuver = false
	}
	if !haveManeuver {
		return false
	}

	if n.escapeCount == 0 {
		n.escapeSequenceStartXY = &trackmodel.Waypoint{X: pose.X, Y: pose.Y}
	}
	n.escapeCount++
	n.beginManeuver(n.maybeEscalate(maneuver))
	n.debug = debug
	n.driveActiveManeuver(pose.X, pose.Y, pose.Yaw, PhaseEscapeTriggered)
	return true
}

// refreshSignLanes rebuilds the planned path onto its pass-side lanes when
// the sign layout changes, matching _refresh_sign_lanes.
//
// No-op unless a SignRouter exists and SignLanePlanner is set, so the Open
// Challenge's path is never rewritten -- it has no router at all, and the
// early return here is what makes that structural rather than a matter of
// the flag's value.
//
// Lanes are always recomputed from laneBaseWaypoints (the path as planned)
// rather than from waypoints: re-laning an already-laned path would stack
// one offset on the next every time the layout was refreshed.
//
// waypointIndex is deliberately NOT re-seeked the way ReplacePath does.
// This transform is 1:1 and order-preserving -- waypoint i of the lane path
// is waypoint i of the base path moved sideways -- so the index still
// denotes the same point on the same lap.
func (n *Navigator) refreshSignLanes() {
	if n.signRouter == nil || !n.cfg.SignLanePlanner {
		return
	}
	fingerprint := laneFingerprintOf(n.signRouter)
	if n.laneFingerprintSet && slices.Equal(fingerprint, n.laneFingerprint) {
		return
	}
	n.laneFingerprint, n.laneFingerprintSet = fingerprint, true

	previous := n.waypoints
	n.waypoints = signrouter.ApplySignLanes(
		n.laneBaseWaypoints,
		n.signRouter.LaneSpecs(),
		signrouter.SignLaneParams{
			LateralOffsetM:    n.cfg.LaneLateralOffsetM(n.signRouterCfg.ChassisHalfDiagonalM),
			RampM:             n.cfg.SignLaneRampM,
			HoldM:             n.cfg.SignLaneHoldM,
			SplitOverlap:      n.cfg.SignLaneSplitOverlap,
			SkipUnsatisfiable: n.cfg.SignLaneSkipUnsatisfiable,
			CornerEntryM:      n.cfg.SignLaneCornerEntryM,
		},
		n.signRouterCfg,
	)
	n.holdCommittedPath(previous, n.cfg.SignLaneCommitAheadM)
	n.applyPathWallBudget()
	n.logger.Info("sign lanes replanned", "signs", len(fingerprint))
}

// laneFingerprintOf derives the sign layout a lane path would be built for,
// standing in for SignRouter.lane_fingerprint (see laneFingerprintEntry).
func laneFingerprintOf(router *signrouter.SignRouter) []laneFingerprintEntry {
	specs := router.LaneSpecs()
	fingerprint := make([]laneFingerprintEntry, 0, len(specs))
	for _, spec := range specs {
		fingerprint = append(fingerprint, laneFingerprintEntry{
			X: spec.Spec.X, Y: spec.Spec.Y, Corridor: spec.Corridor,
		})
	}
	return fingerprint
}

// holdCommittedPath keeps a lane rebuild from moving the path the chassis
// is already on, matching _hold_committed_path.
//
// A lane ramps onto its offset over the approach, which assumes the rebuild
// happens before the robot reaches that stretch. Sighted runs satisfy that
// trivially -- the layout is known at t=0 and the path is built once. A
// discovering run does not, and is instantly off a path it has no runway to
// rejoin. So the near field is pinned to what it already was; new
// information still bends the path, just ahead of the robot rather than
// underneath it.
//
// No-op at commitAheadM 0.0 (the shipped default), and no-op for a sighted
// run either way, since nothing rebuilds after the first tick there.
func (n *Navigator) holdCommittedPath(previous []trackmodel.Waypoint, commitAheadM float64) {
	if commitAheadM <= 0.0 || len(previous) == 0 || len(previous) != len(n.waypoints) {
		return
	}
	pose, ok := n.gateway.GetCurrentPose()
	if !ok {
		return
	}
	cosYaw, sinYaw := math.Cos(pose.Yaw), math.Sin(pose.Yaw)
	held := slices.Clone(n.waypoints)
	for i, old := range previous {
		next := n.waypoints[i]
		if old == next {
			continue
		}
		// Along-track distance in the chassis frame: negative is behind.
		if (next.X-pose.X)*cosYaw+(next.Y-pose.Y)*sinYaw < commitAheadM {
			held[i] = old
		}
	}
	n.waypoints = held
}
