package parking

import (
	"math"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// Phase is the parking maneuver's state, matching shared.domain.enums.ParkPhase
// and the wire ParkPhase in schema/pb/vtitan/nav/v1. A future adapter can map
// each value onto the protobuf constant without reinterpreting behavior.
type Phase int

// ParkCommand is the motor command from the park controller.
type ParkCommand struct {
	// LinearMPS is the forward speed (m/s); negative reverses.
	LinearMPS float64
	// SteeringNorm is the normalized steering command in [-1, 1].
	SteeringNorm float64
	// Done reports whether the maneuver has reached its terminal phase.
	Done bool
	// Phase is the phase that produced this command.
	Phase Phase
}

// ParkController is the two-phase parallel-park controller.
//
// Phase 1 -- STAGE: pure-pursuit toward the staging position in front of the
// bay opening (on the track side).
//
// Phase 2 -- ENTER: pure-pursuit toward the bay center; switches to DONE only
// when the whole footprint is inside the lot AND the heading is parallel to
// the wall within tolerance.
//
// Note: ENTER still pure-pursues a single point, which steers for position
// without controlling the final heading. With the stop condition now requiring
// genuine containment, that is not sufficient to park this chassis -- the
// controller will honestly time out rather than falsely report success. The
// entry maneuver itself is a separate piece of work; see
// platform/docs/internal/2026-07-25-parking-review.md.
type ParkController struct {
	cfg       Config
	section   trackmodel.Section
	direction trackmodel.Direction
	speed     float64
	phase     Phase
	maxFrames int

	framesElapsed   int
	timedOut        bool
	zone            ParkZone
	staging         trackmodel.Waypoint
	repositionLeft  int
	repositionSpeed float64
	repositionSteer float64
	saturatedTicks  int
}

const (
	// PhaseStage is the staging phase: drive to the point in front of the bay.
	PhaseStage Phase = iota
	// PhaseEnter is the entry phase: drive into the bay and correct heading.
	PhaseEnter
	// PhaseDone is the terminal phase: parked cleanly, or given up on budget.
	PhaseDone
)

// NewParkController builds a ParkController from explicit geometry and tuning.
// speed defaults to cfg.DefaultSpeedMPS when zero; maxFrames defaults to
// cfg.DefaultMaxFrames when zero.
// AttemptAfterFinalLap reports whether the round should pursue the bay once
// the final lap is banked, mirroring ParkingParams.ATTEMPT_AFTER_FINAL_LAP.
// The controller is constructed either way -- deferring the pursuit must not
// change what the scenario contains -- so both the navigator's finish branch
// and the sim runner's stop condition consult this rather than the
// controller's mere existence.
func (p *ParkController) AttemptAfterFinalLap() bool { return p.cfg.AttemptAfterFinalLap }

func NewParkController(
	lot ParkingLot,
	section trackmodel.Section,
	direction trackmodel.Direction,
	cfg Config,
	speed float64,
	maxFrames int,
) *ParkController {
	if speed == 0 {
		speed = cfg.DefaultSpeedMPS
	}
	if maxFrames == 0 {
		maxFrames = cfg.DefaultMaxFrames
	}
	zone := BuildZone(
		lot.Block1,
		lot.Block2,
		section,
		direction,
		DefaultParkingLotSpecs,
		DefaultTrackDimensions,
	)
	staging := StagingPos(zone, section, cfg.ApproachClearanceM)
	return &ParkController{
		cfg:       cfg,
		section:   section,
		direction: direction,
		speed:     speed,
		phase:     PhaseStage,
		maxFrames: maxFrames,
		zone:      zone,
		staging:   staging,
	}
}

// IsRepositioning reports whether STAGE/ENTER is mid reverse-and-reorient
// recovery (see pursueWithReposition). CoreNavigator's generic stuck-detector
// escape is blind to parking geometry (the inner keep-out block, the staging
// point) and can fight this maneuver -- e.g. its own low-net-displacement
// check can misfire during a deliberate reverse burst that genuinely IS making
// progress (rotating toward a reachable heading) but doesn't move far in a
// straight line. CoreNavigator should not override this with a generic escape
// while it is active.
func (c *ParkController) IsRepositioning() bool {
	return c.repositionLeft > 0
}

// IsDone reports whether the parking maneuver is complete (cleanly or via
// timeout).
func (c *ParkController) IsDone() bool {
	return c.phase == PhaseDone
}

// IsTimedOut reports whether the maneuver gave up rather than parking cleanly.
// Two ways to give up: exhausting the frame budget, or ENTER reaching the field
// wall without achieving containment. Both mean "stopped, not parked", which is
// the distinction callers actually act on.
func (c *ParkController) IsTimedOut() bool {
	return c.timedOut
}

// Section returns the corridor that contains the parking lot.
func (c *ParkController) Section() trackmodel.Section { return c.section }

// Direction returns the traversal direction the parked heading was chosen to
// match.
func (c *ParkController) Direction() trackmodel.Direction { return c.direction }

// Zone returns the parking lot rectangle and target heading this controller is
// aiming for.
func (c *ParkController) Zone() ParkZone { return c.zone }

// Staging returns the staging position in front of the gap opening.
func (c *ParkController) Staging() trackmodel.Waypoint { return c.staging }

// Update computes the next motor command from the current world pose of the
// robot center (heading 0=east, pi/2=north).
func (c *ParkController) Update(pose trackmodel.Pose) ParkCommand {
	if c.phase == PhaseDone {
		return ParkCommand{LinearMPS: 0.0, SteeringNorm: 0.0, Done: true, Phase: PhaseDone}
	}

	c.framesElapsed++
	if c.framesElapsed > c.maxFrames {
		c.timedOut = true
		c.phase = PhaseDone
		return ParkCommand{LinearMPS: 0.0, SteeringNorm: 0.0, Done: true, Phase: PhaseDone}
	}

	// Early exit: if already inside the zone at any phase, we're done.
	if posInside, yawOK := InsideZone(pose.X, pose.Y, pose.Yaw, c.zone, c.cfg); posInside && yawOK {
		c.phase = PhaseDone
		return ParkCommand{LinearMPS: 0.0, SteeringNorm: 0.0, Done: true, Phase: PhaseDone}
	}

	if c.phase == PhaseStage {
		return c.handleStage(pose)
	}
	return c.handleEnter(pose)
}

// pursueWithReposition runs curvature-based pure pursuit of target, with
// reverse-and-reorient recovery. Shared by both STAGE (toward the staging
// point) and ENTER (toward the gap center) -- both are the same underlying
// problem (drive toward a fixed target point), and both can hit the same
// degenerate case: a target that requires a sharper turn than the chassis's
// minimum turning radius allows on a forward arc. Real pure pursuit (curvature
// from lookahead geometry, clamped only at the physical steering limit)
// replaces what used to be a plain bearing-error * kp controller -- at this
// controller's old kp=2.5, any bearing error past ~23 degrees already
// saturated to full lock, meaning it was never actually proportional in
// practice, just bang-bang.
func (c *ParkController) pursueWithReposition(
	pose trackmodel.Pose,
	target trackmodel.Waypoint,
	phase Phase,
) ParkCommand {
	// An in-progress reposition runs for its full latched duration rather
	// than being re-evaluated (and potentially canceled) every tick -- a
	// single-tick reaction flickers in and out without ever creating enough
	// separation to actually escape the degenerate geometry.
	if c.repositionLeft > 0 {
		c.repositionLeft--
		return ParkCommand{
			LinearMPS:    c.repositionSpeed,
			SteeringNorm: c.repositionSteer,
			Phase:        phase,
		}
	}

	xLocal, yLocal := pose.ToLocalFrame(target)

	if xLocal < 0 {
		// Target is behind the robot: pure pursuit's curvature formula is
		// only valid for a roughly-forward target -- for a rearward one it can
		// produce a plausible-looking (non-saturated) steering command that
		// actually drives away from the target instead of toward it. Reverse
		// immediately rather than trust it.
		return c.startReposition(pose, target, phase)
	}

	steer := PurePursuitSteer(xLocal, yLocal, c.cfg)

	if math.Abs(steer) >= c.cfg.SaturatedSteerThreshold {
		c.saturatedTicks++
	} else {
		c.saturatedTicks = 0
	}

	if c.saturatedTicks >= c.cfg.SaturationStuckTicks {
		// Steering has been pinned at physical lock for a full second
		// straight: the required curvature genuinely exceeds what the chassis
		// can do going forward. Reverse to open room instead of continuing to
		// orbit.
		return c.startReposition(pose, target, phase)
	}

	return ParkCommand{LinearMPS: c.speed, SteeringNorm: steer, Phase: phase}
}

// startReposition latches a reverse-and-reorient recovery burst. See
// pursueWithReposition.
func (c *ParkController) startReposition(
	pose trackmodel.Pose,
	target trackmodel.Waypoint,
	phase Phase,
) ParkCommand {
	bearingErr := BearingError(pose, target)
	c.saturatedTicks = 0
	c.repositionLeft = c.cfg.DefaultMaxFrames
	c.repositionSpeed = c.cfg.RepositionSpeedMPS
	// Sign-flipped for reverse Ackermann geometry (v<0 inverts the yaw-rate
	// response to a given steer sign), biased toward whichever side the target
	// currently bears.
	c.repositionSteer = -navutil.Clamp(c.cfg.RepositionSteerMag*sign(bearingErr), -1.0, 1.0)
	c.repositionLeft--
	return ParkCommand{LinearMPS: c.repositionSpeed, SteeringNorm: c.repositionSteer, Phase: phase}
}

func (c *ParkController) handleStage(pose trackmodel.Pose) ParkCommand {
	dist := math.Hypot(c.staging.X-pose.X, c.staging.Y-pose.Y)

	if c.repositionLeft <= 0 && dist < c.cfg.PosReachDistM {
		c.phase = PhaseEnter
		return c.handleEnter(pose)
	}

	return c.pursueWithReposition(pose, c.staging, PhaseStage)
}

func (c *ParkController) handleEnter(pose trackmodel.Pose) ParkCommand {
	z := c.zone
	rx, ry, robotYaw := pose.X, pose.Y, pose.Yaw

	if posInside, yawOK := InsideZone(rx, ry, robotYaw, z, c.cfg); posInside && yawOK {
		c.phase = PhaseDone
		return ParkCommand{LinearMPS: 0.0, SteeringNorm: 0.0, Done: true, Phase: PhaseDone}
	}

	if FootprintBreachesWall(rx, ry, robotYaw, z, c.cfg) {
		c.timedOut = true
		c.phase = PhaseDone
		return ParkCommand{LinearMPS: 0.0, SteeringNorm: 0.0, Done: true, Phase: PhaseDone}
	}

	if FootprintBreachesMarkers(rx, ry, robotYaw, z, c.cfg) {
		c.timedOut = true
		c.phase = PhaseDone
		return ParkCommand{LinearMPS: 0.0, SteeringNorm: 0.0, Done: true, Phase: PhaseDone}
	}

	return c.pursueWithReposition(pose, trackmodel.Waypoint{X: z.GapCX, Y: z.GapCY}, PhaseEnter)
}

// InsideZone returns (fully_parked, parallel_ok) per the WRO parking rule.
func InsideZone(rx, ry, robotYaw float64, zone ParkZone, cfg Config) (parked, parallelOK bool) {
	yawErr := math.Abs(NormaliseAngle(robotYaw - zone.TargetYaw))
	parked = FootprintInside(rx, ry, robotYaw, zone, cfg.ChassisLengthM, cfg.ChassisWidthM)
	parallelOK = yawErr <= cfg.YawTolerance
	return parked, parallelOK
}

// BearingError returns the signed angle (radians) from the robot's heading to
// the bearing toward target.
func BearingError(pose trackmodel.Pose, target trackmodel.Waypoint) float64 {
	return NormaliseAngle(target.BearingTo(pose.ToWaypoint()) - pose.Yaw)
}

// PurePursuitSteer aims directly at a single fixed target for ParkController.
// It supplies its own lookahead floor rather than at each call site. The
// wheelbase and steering limit come from cfg, mirroring the Python shared
// pure_pursuit_steer (which read RobotSpecs.WHEELBASE and the physical
// steering limit as module globals).
func PurePursuitSteer(xLocal, yLocal float64, cfg Config) float64 {
	return navutil.PurePursuitSteer(
		xLocal,
		yLocal,
		cfg.MinLookaheadDistM,
		cfg.WheelbaseM,
		cfg.MaxSteeringAngleRad,
	)
}

// sign returns +1 for a non-negative x and -1 for a negative x.
func sign(x float64) float64 {
	if x < 0 {
		return -1
	}
	return 1
}

// ParkControllerFromMetadata builds a ParkController from parking-lot geometry.
// It returns nil when lot is nil -- the Open Challenge has no lot, so there is
// no park maneuver to construct (matching the Python original's None return for
// a scenario without parking_lot).
//
// The Python original unpacked a scenario-metadata dict into a ParkingLot and
// resolved direction from starting_conditions. The Go equivalent takes the
// already-parsed lot and direction directly: the caller holds them after
// parsing the scenario, and every real caller already does, so re-deriving them
// from a generic map here would only re-introduce the resolution ambiguity the
// Python version had to guard against.
func ParkControllerFromMetadata(
	lot *ParkingLot,
	section trackmodel.Section,
	direction trackmodel.Direction,
	cfg Config,
) *ParkController {
	if lot == nil {
		return nil
	}
	return NewParkController(*lot, section, direction, cfg, 0, 0)
}

// String returns the phase's wire name, matching
// shared.domain.enums.ParkPhase's values. Crosses a language boundary via
// NavigatorDebugSnapshot.park_phase, so the spelling is a contract.
func (p Phase) String() string {
	switch p {
	case PhaseStage:
		return "stage"
	case PhaseEnter:
		return "enter"
	case PhaseDone:
		return "done"
	default:
		return "unknown"
	}
}
