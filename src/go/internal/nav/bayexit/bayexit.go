package bayexit

import (
	"math"
	"sort"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/corridorfollower"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"
)

// legStallEpsilonM is the wheel travel below which a tick counts as no
// progress at all, matching _LEG_STALL_EPSILON_M. A tenth of a millimetre:
// two orders under the 7.5 mm a free tick covers at creep, so ordinary slow
// motion never reads as a stall, while a chassis held against a surface --
// which reports no travel at all -- registers immediately.
const legStallEpsilonM = 1e-4

// BayExit drives the reverse-then-swing or cycle exit out of the parking
// pocket at the start of a round, matching bay_exit.py's BayExit class.
//
// One instance per round. It holds the two things a scan cannot re-derive:
// where the reverse leg started (the pocket looks the same throughout it)
// and which side is open (the rays that answer that stop meaning anything
// as soon as the chassis rotates -- see Config.Follower.BayExitLatchDirection).
type BayExit struct {
	reverseStartM *float64
	reverseDone   bool
	reverseTicks  int
	forwardTicks  int
	reverseProgM  float64
	openIsLeft    *bool
	openFlips     int
	// openVotesLeft/openVotesRight ballot the open side until the latch is
	// taken, matching _open_votes_left/_open_votes_right.
	openVotesLeft  int
	openVotesRight int

	// Cycle manoeuvre state. Starts on the FORWARD leg: the steered wheels
	// are at the front, so a forward move is the one that rotates the nose
	// out, and the reverse exists only to buy back the room it spends.
	legIsReverse   bool
	legStartM      *float64
	lastTravelledM float64
	legStallTicks  int
	// legTicks are MOVING ticks the current leg has run, bounded by
	// BayExitLegMaxS, matching _leg_ticks. Only meaningful with
	// BayExitClearanceGuard.
	legTicks int
	cyclesN  int
	// settleTicks are ticks of standstill still owed to the servo before
	// this leg may move. Starts at 0: the first arc begins from wherever
	// the wheels already are, and the settle is budgeted at each leg
	// CHANGE.
	settleTicks int

	// ticks the manoeuvre has run, and whether the fallback has fired.
	ticks    int
	switched bool

	// Dead-reckoned pose in the BAY FRAME, for the clearance guard. Signed
	// so +out is the OPEN side and +yaw turns toward it, which makes the
	// fin geometry symmetric and removes the left/right case split.
	drAlong     float64
	drOut       float64
	drYaw       float64
	drPrevM     *float64
	drWheelRad  float64
	guardFlips  int
	guardMinGap *float64
	// guardBlockTicks are consecutive ticks the guard has refused BOTH
	// legs, matching _guard_block_ticks -- the manoeuvre's only deadlock
	// detector.
	guardBlockTicks int

	// Straight-reverse recovery from wall contact. Ticks still owed, and how
	// many times it has fired.
	recoveryTicksLeft int
	contactRecoveries int

	// Rotation achieved since placement, from MEASURED yaw. Accumulated
	// rather than differenced so a wrap at +/-pi does not read as a 360 deg
	// jump, and kept separate from drYaw (which is dead-reckoned for the fin
	// guard and understates the real rotation).
	startYawRad *float64
	prevYawRad  *float64
	rotationRad float64
}

// New returns a zero-valued BayExit, matching BayExit().
func New() *BayExit {
	return &BayExit{}
}

// GuardStats returns (directionFlips, minPredictedGapM, outwardTravelM) for
// the guard, matching the guard_stats property. ok is false when the guard
// never ran (no predicted gap has been recorded).
func (b *BayExit) GuardStats() (directionFlips int, minPredictedGapM float64, ok bool, outwardTravelM float64) {
	if b.guardMinGap == nil {
		return b.guardFlips, 0, false, b.drOut
	}
	return b.guardFlips, *b.guardMinGap, true, b.drOut
}

// Cycles returns completed forward-then-reverse cycles, for diagnostics,
// matching the cycles property.
func (b *BayExit) Cycles() int {
	return b.cyclesN
}

// Legs returns (reverseTicks, forwardTicks, reverseProgressM), for
// diagnostics, matching the legs property.
//
// The reverse gate reads SIGNED odometry, which cancels under rocking: a
// chassis pushed forward as much as it backs registers no progress and the
// manoeuvre never advances to the turn. Distance travelled cannot show
// that -- the path length accumulates either way -- so which leg the ticks
// were spent in has to be counted rather than inferred.
func (b *BayExit) Legs() (reverseTicks, forwardTicks int, reverseProgressM float64) {
	return b.reverseTicks, b.forwardTicks, b.reverseProgM
}

// OpenFlips returns the times the measured open side changed sides during
// the manoeuvre, matching the open_flips property.
func (b *BayExit) OpenFlips() int {
	return b.openFlips
}

// ContactRecoveries returns how many times the nose-against-wall reverse has
// fired this exit, matching the contact_recoveries property.
func (b *BayExit) ContactRecoveries() int {
	return b.contactRecoveries
}

// RotationDeg returns the rotation from the placement heading, in degrees,
// from measured yaw, matching the rotation_deg property.
func (b *BayExit) RotationDeg() float64 {
	return math.Abs(b.rotationRad) * navutil.DegreesPerHalfTurn / math.Pi
}

// RotationComplete reports whether the chassis has turned far enough to
// leave the pocket, matching rotation_complete.
//
// Returns false while no yaw has been supplied, which keeps the manoeuvre on
// its previous behaviour rather than releasing on an unmeasured claim.
func (b *BayExit) RotationComplete(cfg Config) bool {
	if b.startYawRad == nil {
		return false
	}
	return b.RotationDeg() >= cfg.Follower.BayExitTargetYawDeg
}

// trackRotation accumulates rotation since placement, unwrapping at +/-pi,
// matching _track_rotation.
func (b *BayExit) trackRotation(yawRad *float64) {
	if yawRad == nil {
		return
	}
	if b.startYawRad == nil {
		start := *yawRad
		b.startYawRad = &start
		prev := *yawRad
		b.prevYawRad = &prev
		return
	}
	if b.prevYawRad != nil {
		step := *yawRad - *b.prevYawRad
		if step > math.Pi {
			step -= 2.0 * math.Pi
		} else if step < -math.Pi {
			step += 2.0 * math.Pi
		}
		b.rotationRad += step
	}
	prev := *yawRad
	b.prevYawRad = &prev
}

// bicycleYawStep is the yaw the chassis turns over stepM of travel at
// wheelRad of lock, matching _bicycle_yaw_step.
//
// The curvature is FLOORED by cfg.MinTurnRadiusM, exactly as
// AckermannKinematics floors it: the two are the same physical model and had
// already drifted apart. Skipped if MinTurnRadiusM <= 0.
func bicycleYawStep(stepM, wheelRad float64, cfg Config) float64 {
	curvature := math.Tan(wheelRad) * cfg.YawGain / cfg.EffectiveWheelbaseM()
	if cfg.MinTurnRadiusM > 0.0 {
		limit := 1.0 / cfg.MinTurnRadiusM
		curvature = navutil.Clamp(curvature, -limit, limit)
	}
	return stepM * curvature
}

// wallFeasibleYawRad is the greatest yaw the pocket's DEPTH allows at outM
// of outward displacement, matching _wall_feasible_yaw_rad.
func wallFeasibleYawRad(outM float64, cfg Config) float64 {
	reach := outM + cfg.ParkingLot.WallOffsetM
	diagonal := math.Hypot(cfg.ChassisLengthM, cfg.ChassisWidthM)
	sinSum := 2.0 * reach / diagonal
	if sinSum >= 1.0 {
		return math.Pi / 2.0
	}
	return math.Max(0.0, math.Asin(sinSum)-math.Atan2(cfg.ChassisWidthM, cfg.ChassisLengthM))
}

// alongSlackM is the greatest along-wall displacement the pocket physically
// admits, matching _along_slack_m.
func alongSlackM(cfg Config) float64 {
	halfSpacing := cfg.ParkingLot.BlockSpacingFactor * cfg.ChassisLengthM / 2.0
	inner := halfSpacing - cfg.ParkingLot.Width/2.0
	return math.Max(0.0, inner-cfg.ChassisLengthM/2.0)
}

// legSpeed is the speed for one bay-exit leg, as a POSITIVE magnitude,
// matching _leg_speed.
//
// BayExitSpeedMPS overrides the whole chain with an ABSOLUTE value, because
// what this manoeuvre needs is set by torque against static friction at full
// lock, not by any relationship to cruising speed. exitScale is false on the
// legacy pre-guard paths, which never applied BayExitSpeedScale; keeping
// that lets the override reach them without changing what they do when it
// is unset.
func legSpeed(creepSpeedMPS float64, f corridorfollower.Config, reverse, exitScale bool) float64 {
	if f.BayExitSpeedMPS > 0.0 {
		return f.BayExitSpeedMPS
	}
	scale := f.CornerSpeedScale
	if reverse {
		scale = f.ReverseSpeedScale
	}
	if exitScale {
		return creepSpeedMPS * scale * f.BayExitSpeedScale
	}
	return creepSpeedMPS * scale
}

// deadReckon advances the bay-frame pose from wheel odometry and the
// commanded steering, matching _dead_reckon.
//
// A bicycle model driven by the two things the robot genuinely has in the
// pocket: how far the wheels turned, and what angle it asked the servo
// for. No LIDAR, because the pocket cannot be sensed from inside it, and no
// pose estimate, because the localizer is matching a wall model the
// chassis is not yet out among.
//
// The servo's SLEW is modelled rather than assumed instant -- skipping it
// is what made BayExitSteerNorm read as inert, since every command at or
// above a threshold clipped to the same reachable angle.
func (b *BayExit) deadReckon(travelledM, wheelNorm float64, cfg Config) {
	previous := b.drPrevM
	drPrevM := travelledM
	b.drPrevM = &drPrevM
	if previous == nil {
		return
	}
	step := travelledM - *previous
	maxRad := cfg.followerMaxSteeringAngleRad()
	target := navutil.Clamp(wheelNorm, -1.0, 1.0) * maxRad
	slew := cfg.MaxSteeringRateRadPerS / cfg.ControlHz
	b.drWheelRad += navutil.Clamp(target-b.drWheelRad, -slew, slew)
	b.drYaw += bicycleYawStep(step, b.drWheelRad, cfg)
	// The wall behind the pocket CLIPS the rotation, and dead reckoning
	// cannot see it -- unclamped, the guard bounds a pose the chassis can
	// never reach. Computed from drOut BEFORE this tick's own update.
	limit := wallFeasibleYawRad(b.drOut, cfg)
	b.drYaw = navutil.Clamp(b.drYaw, -limit, limit)
	// Bounded for the same reason the yaw is: a modelled pose the pocket
	// forbids is not one the guard may act on.
	slack := alongSlackM(cfg)
	b.drAlong = navutil.Clamp(b.drAlong+step*math.Cos(b.drYaw), -slack, slack)
	b.drOut += step * math.Sin(b.drYaw)
}

// predictedGap is the fin clearance the chassis WOULD have after one more
// step like this one, matching _predicted_gap.
func (b *BayExit) predictedGap(stepM, wheelNorm float64, cfg Config) float64 {
	maxRad := cfg.followerMaxSteeringAngleRad()
	target := navutil.Clamp(wheelNorm, -1.0, 1.0) * maxRad
	slew := cfg.MaxSteeringRateRadPerS / cfg.ControlHz
	wheel := b.drWheelRad + navutil.Clamp(target-b.drWheelRad, -slew, slew)
	yaw := b.drYaw + bicycleYawStep(stepM, wheel, cfg)
	// Same wall clip as deadReckon, for the same reason: a predicted pose
	// the pocket forbids is not a prediction the guard may act on.
	limit := wallFeasibleYawRad(b.drOut, cfg)
	yaw = navutil.Clamp(yaw, -limit, limit)
	slack := alongSlackM(cfg)
	along := navutil.Clamp(b.drAlong+stepM*math.Cos(yaw), -slack, slack)
	out := b.drOut + stepM*math.Sin(yaw)
	corners := rectCorners(along, out, yaw, cfg.ChassisLengthM, cfg.ChassisWidthM)
	fins := finRects(cfg)
	return min(gap(corners, fins[0][:]), gap(corners, fins[1][:]))
}

// resetForSwitch re-origins both manoeuvres' odometry state at the
// handover, matching _reset_for_switch.
//
// Every distance is measured from a remembered starting odometry reading,
// and those readings belong to the manoeuvre that just gave up. Carried
// across, the incoming reverse leg would believe it had already run --
// reverseStartM is set on the first tick of the round, so by the switch it
// is hundreds of ticks stale.
func (b *BayExit) resetForSwitch(travelledM float64) {
	rs := travelledM
	b.reverseStartM = &rs
	b.reverseDone = false
	b.legIsReverse = false
	ls := travelledM
	b.legStartM = &ls
	b.lastTravelledM = travelledM
	b.legStallTicks = 0
	b.legTicks = 0
	b.settleTicks = 0
}

// beginLeg switches legs and budgets the standstill needed to reach the
// new angle, matching _begin_leg.
//
// The pause is COMPUTED, not tuned: the servo covers
// MaxSteeringRateRadPerS/ControlHz radians per tick, and the swing is the
// difference between the two legs' angles, so the tick count follows from
// the geometry and moves correctly if either constant changes.
func (b *BayExit) beginLeg(isReverse bool, travelledM float64, cfg Config, fromNorm float64) {
	f := cfg.Follower
	arc := navutil.Clamp(f.BayExitArcSteerNorm, 0.0, 1.0)
	back := navutil.Clamp(f.BayExitCycleReverseSteerNorm, 0.0, 1.0)
	sign := signForOpenLeft(*b.openIsLeft)
	toNorm := arc * sign
	if isReverse {
		toNorm = -back * sign
	}
	swingRad := math.Abs(toNorm-fromNorm) * cfg.followerMaxSteeringAngleRad()
	perTickRad := cfg.MaxSteeringRateRadPerS / cfg.ControlHz
	settle := 0
	if perTickRad > 0 {
		settle = int(math.Ceil(swingRad / perTickRad))
	}
	b.settleTicks = settle
	b.legIsReverse = isReverse
	ls := travelledM
	b.legStartM = &ls
	b.legStallTicks = 0
	b.legTicks = 0
	// The standstill would otherwise read as a stall on its very first
	// moving tick, since travel during it is zero by construction.
	b.lastTravelledM = travelledM
}

// signForOpenLeft is +1.0 when the open side is left, -1.0 otherwise --
// the small "which sign for this side" idiom bay_exit.py repeats inline.
func signForOpenLeft(openIsLeft bool) float64 {
	if openIsLeft {
		return 1.0
	}
	return -1.0
}

// guardedCommand shuffles out of the pocket bounded by PREDICTED
// CLEARANCE, never by contact, matching _guarded_command.
//
// The legal replacement for the stall-bounded cycle. Same shape -- steer
// toward the open side, alternate forward and reverse -- but the leg ends
// when the NEXT pose would come within BayExitClearanceMarginM of a fin,
// which is a prediction rather than a collision. Rule 9.24.7 ends the
// round on the touch the old backstop waited for.
//
// Both legs steer toward the open side. Forward swings the nose out;
// reverse with the same lock walks the tail back along the arc it came
// down, which returns most of the yaw and buys the room for the next
// forward leg to gain more "out" than it gives back -- the classic
// tight-slot exit, accumulating outward displacement without ever needing
// a surface to push against.
func (b *BayExit) guardedCommand(
	travelledM, creepSpeedMPS float64, cfg Config, openIsLeft bool,
) controllers.DriveCommand {
	f := cfg.Follower
	// Subtracted rather than lowering BayExitClearanceMarginM directly, so
	// the effective threshold can go negative (tolerating a predicted
	// OVERLAP) without loosening that field's own floor, which exists to
	// stop the margin being set backwards by accident. Ships 0.0 = inert,
	// matching BAY_EXIT_CLEARANCE_TOLERANCE_M.
	margin := f.BayExitClearanceMarginM - f.BayExitClearanceToleranceM
	sign := signForOpenLeft(openIsLeft)
	magnitude := navutil.Clamp(f.BayExitArcSteerNorm, 0.0, 1.0)
	// Both legs hold the SAME lock, and the dead-reckoned frame is signed
	// so +yaw is toward the open side -- so the guard's geometry never
	// needs a left/right case split. The caller-facing command is re-signed
	// below.
	wheelNorm := magnitude
	b.deadReckon(travelledM, wheelNorm, cfg)
	if b.guardMinGap == nil {
		g := b.predictedGap(0.0, wheelNorm, cfg)
		b.guardMinGap = &g
	}

	// TIME bound on the leg, and the only bound this path has otherwise: the
	// gap check below ends a leg when the PREDICTED fin gap closes, and that
	// prediction is dead-reckoned from wheel travel -- so a leg whose wheel
	// has stalled cannot produce the evidence that would end it, and runs
	// until the whole manoeuvre times out. Counted only on ticks past any
	// settle, so the budget is the moving part of the leg.
	b.legTicks++
	legMaxTicks := int(math.Ceil(f.BayExitLegMaxS * cfg.ControlHz))
	if legMaxTicks < 1 {
		legMaxTicks = 1
	}
	if b.legTicks >= legMaxTicks {
		b.legIsReverse = !b.legIsReverse
		b.legTicks = 0
		b.cyclesN++
		return controllers.DriveCommand{SpeedMPS: 0.0, SteeringNorm: magnitude * sign}
	}

	// BayExitSpeedScale multiplies BOTH legs on top of the corner/reverse
	// scale, matching _guarded_command. It is the lever on the COAST the
	// guard has to predict past: commanding zero does not stop the
	// chassis, it decays with tau and travels a further v*tau, against an
	// along-wall budget of tens of millimetres.
	speed := legSpeed(creepSpeedMPS, f, b.legIsReverse, true)
	step := speed / cfg.ControlHz
	if b.legIsReverse {
		step = -speed / cfg.ControlHz
	}
	// Look a STOPPING DISTANCE ahead, not a single tick: commanding zero
	// does not stop the chassis, it coasts a further v*tau.
	coastM := speed * cfg.SpeedResponseTauS
	reach := step + math.Copysign(coastM, step)
	g := b.predictedGap(reach, wheelNorm, cfg)
	*b.guardMinGap = min(*b.guardMinGap, g)

	// "Will this step be clear" is the right question only while the MODEL
	// is clear. Once the dead-reckoned body overlaps a fin, predictedGap is
	// negative at EVERY reach, so both legs are refused and nothing moves.
	// Reachable from inside the fault, the admissible leg is the one that
	// IMPROVES the gap rather than the one that clears the margin.
	held := b.predictedGap(0.0, wheelNorm, cfg)
	recovering := f.BayExitGuardOverlapRecovery && held <= margin && g > held
	if g <= margin && !recovering {
		// End the leg on the PREDICTION -- nothing has been touched -- and
		// pay the servo swing before the next one moves.
		b.legIsReverse = !b.legIsReverse
		b.legTicks = 0
		b.guardFlips++
		b.cyclesN++
		b.guardBlockTicks++
		return controllers.DriveCommand{SpeedMPS: 0.0, SteeringNorm: magnitude * sign}
	}

	b.guardBlockTicks = 0
	if b.legIsReverse {
		b.reverseTicks++
	} else {
		b.forwardTicks++
	}
	if b.legIsReverse {
		return controllers.DriveCommand{SpeedMPS: -speed, SteeringNorm: magnitude * sign}
	}
	return controllers.DriveCommand{SpeedMPS: speed, SteeringNorm: magnitude * sign}
}

// cycleCommand alternates a steered forward arc with a straight reverse,
// until clear, matching _cycle_command.
//
// A shuffle at CONSTANT steering magnitude provably cannot accumulate:
// dy/dtheta = sin(theta) / (k tan(delta)) is independent of speed and its
// sign, so y is a state function of theta and any cycle that returns theta
// to its start returns y with it. Asymmetric legs break it honestly
// instead -- the forward leg arcs toward the open side, the reverse backs
// STRAIGHT, so the theta the arc won is kept while the room it spent is
// bought back.
//
// Legs are latched: the previous gate compared reverse-start-minus-
// travelled against a threshold that the FORWARD leg drives back down, so
// it flapped between two opposed commands. Transitions here are one-way
// within a cycle: forward until the way ahead closes, reverse a bounded
// distance, repeat.
//
// The VERY FIRST forward leg (before any beginLeg call has ever run) has
// no anchored legStartM, so its distance bound reads as permanently zero
// and it can only end via the stall backstop below -- matching
// bay_exit.py's documented default-model behavior of running ONE
// continuous forward arc until the CALLER's separate IsClear check ends
// the manoeuvre externally, with the reverse leg never running at all
// unless the chassis genuinely jams against a fin.
func (b *BayExit) cycleCommand(
	travelledM, creepSpeedMPS float64, cfg Config, openIsLeft bool,
) controllers.DriveCommand {
	f := cfg.Follower
	arc := navutil.Clamp(f.BayExitArcSteerNorm, 0.0, 1.0)
	back := navutil.Clamp(f.BayExitCycleReverseSteerNorm, 0.0, 1.0)
	sign := signForOpenLeft(openIsLeft)
	target := arc * sign
	if b.legIsReverse {
		target = -back * sign
	}

	// Steer FIRST, then drive. Commanding the angle and the motion
	// together let the servo slew while the leg ran and the leg ended
	// before the angle arrived. Slewing at a standstill costs ticks but no
	// travel, and travel is the only thing a 7.5 cm pocket is short of.
	if b.settleTicks > 0 {
		b.settleTicks--
		return controllers.DriveCommand{SpeedMPS: 0.0, SteeringNorm: target}
	}

	// Stall is the primary leg-end signal, not distance: wheel odometry
	// stops accumulating exactly when the chassis is blocked, so any leg
	// bounded only by distance runs FOREVER once it jams. Counted only on
	// ticks that COMMAND motion -- the settle above returns first, so a
	// deliberate standstill is never mistaken for a jam.
	if math.Abs(travelledM-b.lastTravelledM) < legStallEpsilonM {
		b.legStallTicks++
	} else {
		b.legStallTicks = 0
	}
	b.lastTravelledM = travelledM
	stalled := b.legStallTicks >= f.BayExitLegStallTicks

	if b.legIsReverse {
		b.reverseTicks++
		legStart := travelledM
		if b.legStartM != nil {
			legStart = *b.legStartM
		}
		b.reverseProgM = legStart - travelledM
		if stalled || b.reverseProgM >= f.BayExitCycleReverseM {
			b.beginLeg(false, travelledM, cfg, target)
			b.cyclesN++
		}
		return controllers.DriveCommand{
			SpeedMPS:     -legSpeed(creepSpeedMPS, f, true, true),
			SteeringNorm: target,
		}
	}

	b.forwardTicks++
	// Bounded by GEOMETRY plus a stall backstop, NOT by forward clearance
	// -- the live pipeline reads noisy, fluctuating short ranges inside
	// the pocket, so a forward-cone minimum is dominated by surroundings
	// and noise rather than the arc's own progress.
	//
	// legStartM is nil until the first ever beginLeg call (matching
	// bay_exit.py's `self._leg_start_m or travelled_m`, None being falsy),
	// so travelledM-legStart reads as zero against itself here on the
	// VERY FIRST forward leg -- that first leg can only end via the stall
	// backstop below, which anchors legStartM for every leg after it.
	legStart := travelledM
	if b.legStartM != nil {
		legStart = *b.legStartM
	}
	if stalled || travelledM-legStart >= f.BayExitForwardM {
		b.beginLeg(true, travelledM, cfg, target)
	}
	return controllers.DriveCommand{
		SpeedMPS:     legSpeed(creepSpeedMPS, f, false, true),
		SteeringNorm: target,
	}
}

// IsClear reports whether the chassis is out of the pocket and normal
// driving can resume, matching the static is_clear method.
//
// The caller must not simply return on this: the direction-settle block is
// what rebuilds the path for the committed direction and calls
// ReplacePath, and skipping it hands the planner a stale plan still
// pointing at waypoint 0 while the robot has driven out of the bay.
func IsClear(scan controllers.LidarScan, cfg Config) bool {
	f := cfg.Follower
	return navutil.ForwardClearance(scan, f.ForwardArcHalfFovRad, f.MinValidRangeM) >=
		f.MinForwardClearanceM
}

// noseInContact reports whether the forward arc says the nose is touching,
// or too close to see, matching the static _nose_in_contact method.
//
// Two readings mean the same thing here and both must count: a forward
// clearance BELOW BayExitContactDistM is a wall within a chassis nose of the
// bumper, and NO valid returns at all is the same wall, closer still --
// ForwardClearance drops everything under MinValidRangeM and reports +Inf.
func noseInContact(scan controllers.LidarScan, cfg Config) bool {
	f := cfg.Follower
	clearance := navutil.ForwardClearance(scan, f.ForwardArcHalfFovRad, f.MinValidRangeM)
	if math.IsInf(clearance, 1) {
		return true
	}
	return clearance < f.BayExitContactDistM
}

// openSideScore is how open a sector is: valid fraction times median valid
// range, matching _open_side_score.
//
// A NO-RETURN AT THE POCKET WALL IS THE SIGNAL, NOT AN ABSENCE OF ONE: the
// gateway substitutes lidarMaxRangeM for every dropout, and the wall in the
// pocket is close enough that a large share of rays facing it return
// nothing at all. Compared as ranges, the substituted max beats the open
// corridor's real range and the side reads BACKWARDS -- hence scoring by
// valid fraction times median rather than either alone.
func openSideScore(scan controllers.LidarScan, centerRad, halfWidthRad, lidarMaxRangeM float64) float64 {
	ceiling := lidarMaxRangeM * 0.99
	total := 0
	valid := make([]float64, 0, len(scan.RangesM))
	for i, a := range scan.AnglesRad {
		if math.Abs(navutil.WrapAngle(a-centerRad)) > halfWidthRad {
			continue
		}
		total++
		r := scan.RangesM[i]
		if r > 0.0 && r < ceiling {
			valid = append(valid, r)
		}
	}
	if len(valid) == 0 || total == 0 {
		return 0.0
	}
	sort.Float64s(valid)
	n := len(valid)
	var med float64
	if n%2 == 1 {
		med = valid[n/2]
	} else {
		med = (valid[n/2-1] + valid[n/2]) / 2.0
	}
	return float64(len(valid)) / float64(total) * med
}

// resolveOpenSide is which way is out, latched once because it is a fact
// about the layout, matching _resolve_open_side.
//
// Scored over a SECTOR rather than a single ray at +/-90 deg: the gateway
// substitutes max range for a dropout, and the ray facing the near pocket
// wall drops out far more often than the one facing open space, so a single
// ray reads backwards a meaningful fraction of the time. Polled for several
// ticks before the latch is taken, rather than deciding on the first scan.
func (b *BayExit) resolveOpenSide(scan controllers.LidarScan, cfg Config) bool {
	f := cfg.Follower
	halfWidth := f.BayExitOpenSideSectorDeg * math.Pi / navutil.DegreesPerHalfTurn
	left := openSideScore(scan, math.Pi/2, halfWidth, cfg.LidarMaxRangeM)
	right := openSideScore(scan, -math.Pi/2, halfWidth, cfg.LidarMaxRangeM)
	openIsLeft := left > right

	if b.openIsLeft == nil {
		if openIsLeft {
			b.openVotesLeft++
		} else {
			b.openVotesRight++
		}
		votes := b.openVotesLeft + b.openVotesRight
		if f.BayExitLatchDirection && votes < f.BayExitOpenSideVotes {
			return openIsLeft
		}
		openIsLeft = b.openVotesLeft > b.openVotesRight
	} else if f.BayExitLatchDirection {
		openIsLeft = *b.openIsLeft
	}
	if b.openIsLeft != nil && openIsLeft != *b.openIsLeft {
		b.openFlips++
	}
	o := openIsLeft
	b.openIsLeft = &o
	return openIsLeft
}

// Command backs out of the parking pocket, then swings the nose to the
// open side, matching the command method.
//
// Pivoting straight from a centred placement does not work: the pocket is
// 0.45 m along the wall against a 0.30 m chassis, so there is only ~7.5 cm
// of slack at each end, and the nose reaches the marker before it has
// rotated clear. So reverse first, to double the room ahead, then turn
// hard.
//
// Which way to turn is not a guess: the lot is always against the OUTER
// wall, so its opening faces the inner block, and a lap always turns
// toward the inner block -- open side, inner side and corner-turn side are
// the same side by track design. Being a fact about the layout, it is read
// once and latched rather than re-derived every tick -- see
// Config.Follower.BayExitLatchDirection.
//
// rangesM/anglesRad are the LIDAR scan; travelledM is signed wheel
// odometry (a quadrature encoder counts DOWN in reverse, so progress on
// the reverse leg is start-minus-current); creepSpeedMPS is the blind-phase
// creep speed both legs scale from; yawRad is the current yaw estimate
// (nil while unavailable), used only for BayExitTargetYawDeg's rotation
// release.
func (b *BayExit) Command(
	scan controllers.LidarScan, travelledM, creepSpeedMPS float64, cfg Config, yawRad *float64,
) controllers.DriveCommand {
	f := cfg.Follower
	if b.reverseStartM == nil {
		rs := travelledM
		b.reverseStartM = &rs
	}

	openIsLeft := b.resolveOpenSide(scan, cfg)

	b.ticks++
	b.trackRotation(yawRad)

	// Nose against the wall is a STATE, not an absence of data, and it is
	// answered before any leg logic: a chassis in contact cannot steer its
	// way out, because the wheels that would turn it are the ones being
	// held. Back straight off first, then let the normal legs resume with
	// room to rotate in.
	if f.BayExitContactRecoveryTicks > 0 && (b.recoveryTicksLeft > 0 || noseInContact(scan, cfg)) {
		if b.recoveryTicksLeft <= 0 {
			b.recoveryTicksLeft = f.BayExitContactRecoveryTicks
			b.contactRecoveries++
		}
		b.recoveryTicksLeft--
		return controllers.DriveCommand{
			SpeedMPS:     -legSpeed(creepSpeedMPS, f, true, true),
			SteeringNorm: 0.0,
		}
	}

	// Turned far enough AND the way out is actually open. Rotation alone is
	// not enough to drive out on -- the target angle is where the chassis
	// stops lying across the pocket, not where it is guaranteed to be aimed
	// down the corridor. AFTER the contact check, not before: a chassis
	// that has turned far enough AND is touching must still back off first.
	if b.RotationComplete(cfg) && IsClear(scan, cfg) {
		return controllers.DriveCommand{
			SpeedMPS:     legSpeed(creepSpeedMPS, f, false, true),
			SteeringNorm: 0.0,
		}
	}

	// The clearance guard supersedes both contact-bounded exits, so it is
	// answered before their fallback bookkeeping runs at all -- but only
	// while it is still BOUNDING legs rather than refusing every one of
	// them. A guard doing its job alternates block and motion, so a long
	// UNBROKEN run of blocks is the signature that separates the two.
	guardTrapped := f.BayExitGuardBlockTicks > 0 && b.guardBlockTicks >= f.BayExitGuardBlockTicks
	if f.BayExitClearanceGuard && !guardTrapped {
		return b.guardedCommand(travelledM, creepSpeedMPS, cfg, openIsLeft)
	}

	// Which exit is driving. After BayExitFallbackFrames the OTHER one
	// takes over, once: the two are complementary and which one the real
	// robot needs is unknown, so covering both beats betting on one.
	useCycle := f.BayExitCycle
	if f.BayExitFallbackFrames > 0 && b.ticks > f.BayExitFallbackFrames {
		useCycle = !useCycle
		if !b.switched {
			b.switched = true
			b.resetForSwitch(travelledM)
		}
	}
	if useCycle {
		return b.cycleCommand(travelledM, creepSpeedMPS, cfg, openIsLeft)
	}

	// Wheel distance is SIGNED -- comparing current-minus-start gives a
	// negative that is below any positive threshold forever, which
	// reversed until the tail hit the rear fin.
	b.reverseProgM = *b.reverseStartM - travelledM
	if f.BayExitLatchReverse && b.reverseProgM >= f.BayExitReverseM {
		// One-shot once latching is on. The forward leg drives this same
		// quantity back DOWN, so without the latch the gate returns to
		// reverse on the very next tick and the manoeuvre chatters
		// between two opposed commands instead of holding the turn.
		b.reverseDone = true
	}
	if !b.reverseDone && b.reverseProgM < f.BayExitReverseM {
		b.reverseTicks++
		// Steering is INVERTED on the reverse, the same way
		// FollowCorridor's reverse branch inverts it: backing up swings
		// the nose away from the steer direction, so steering toward the
		// WALL walks the nose out toward the open corridor.
		reverseSteer := navutil.Clamp(f.BayExitReverseSteerNorm, 0.0, 1.0)
		reverseNorm := reverseSteer
		if openIsLeft {
			reverseNorm = -reverseSteer
		}
		if f.BayExitHoldSteer {
			// Hold the FORWARD leg's angle instead of returning to
			// centre -- the servo's slew never reaches full lock inside
			// a stroke this short if every reverse tick re-commands
			// centre. NOT the same as BayExitReverseSteerNorm, which
			// applies the INVERTED sign and so slews even further, to
			// opposite lock (refuted 2026-08-29).
			mag := navutil.Clamp(f.BayExitSteerNorm, 0.0, 1.0)
			reverseNorm = -mag
			if openIsLeft {
				reverseNorm = mag
			}
		}
		return controllers.DriveCommand{
			SpeedMPS:     -legSpeed(creepSpeedMPS, f, true, false),
			SteeringNorm: reverseNorm,
		}
	}

	// Magnitude is tuned, not pinned at full lock: full lock spins the
	// chassis about its own centre and the pocket has no room to rotate
	// in; what gets the robot out is translation.
	b.forwardTicks++
	magnitude := navutil.Clamp(f.BayExitSteerNorm, 0.0, 1.0)
	steer := -magnitude
	if openIsLeft {
		steer = magnitude
	}
	return controllers.DriveCommand{SpeedMPS: legSpeed(creepSpeedMPS, f, false, false), SteeringNorm: steer}
}

// followerMaxSteeringAngleRad is cfg.Follower.MaxSteeringAngleRad, matching
// RobotSpecs.MAX_WHEEL_ANGLE_DEG converted to radians -- reused from
// corridorfollower.Config rather than duplicated, since that field is
// already profile-sourced there.
func (c Config) followerMaxSteeringAngleRad() float64 {
	return c.Follower.MaxSteeringAngleRad
}
