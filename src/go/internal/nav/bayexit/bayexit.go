package bayexit

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
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

	// Cycle manoeuvre state. Starts on the FORWARD leg: the steered wheels
	// are at the front, so a forward move is the one that rotates the nose
	// out, and the reverse exists only to buy back the room it spends.
	legIsReverse   bool
	legStartM      *float64
	lastTravelledM float64
	legStallTicks  int
	cyclesN        int
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
	b.drYaw += step * math.Tan(b.drWheelRad) / cfg.EffectiveWheelbaseM() * cfg.YawGain
	b.drAlong += step * math.Cos(b.drYaw)
	b.drOut += step * math.Sin(b.drYaw)
}

// predictedGap is the fin clearance the chassis WOULD have after one more
// step like this one, matching _predicted_gap.
func (b *BayExit) predictedGap(stepM, wheelNorm float64, cfg Config) float64 {
	maxRad := cfg.followerMaxSteeringAngleRad()
	target := navutil.Clamp(wheelNorm, -1.0, 1.0) * maxRad
	slew := cfg.MaxSteeringRateRadPerS / cfg.ControlHz
	wheel := b.drWheelRad + navutil.Clamp(target-b.drWheelRad, -slew, slew)
	yaw := b.drYaw + stepM*math.Tan(wheel)/cfg.EffectiveWheelbaseM()*cfg.YawGain
	along := b.drAlong + stepM*math.Cos(yaw)
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
	margin := f.BayExitClearanceMarginM
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

	// BayExitSpeedScale multiplies BOTH legs on top of the corner/reverse
	// scale, matching _guarded_command. It is the lever on the COAST the
	// guard has to predict past: commanding zero does not stop the
	// chassis, it decays with tau and travels a further v*tau, against an
	// along-wall budget of tens of millimetres.
	speedScale := f.CornerSpeedScale
	if b.legIsReverse {
		speedScale = f.ReverseSpeedScale
	}
	speed := creepSpeedMPS * speedScale * f.BayExitSpeedScale
	step := speed / cfg.ControlHz
	if b.legIsReverse {
		step = -speed / cfg.ControlHz
	}
	g := b.predictedGap(step, wheelNorm, cfg)
	*b.guardMinGap = min(*b.guardMinGap, g)
	if g <= margin {
		// Reverse the leg rather than push on. The chassis has not
		// touched anything -- this fires on the prediction.
		b.legIsReverse = !b.legIsReverse
		b.guardFlips++
		b.cyclesN++
		speedScale = f.CornerSpeedScale
		if b.legIsReverse {
			speedScale = f.ReverseSpeedScale
		}
		speed = creepSpeedMPS * speedScale * f.BayExitSpeedScale
	}
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
			SpeedMPS:     -creepSpeedMPS * f.ReverseSpeedScale * f.BayExitSpeedScale,
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
		SpeedMPS:     creepSpeedMPS * f.CornerSpeedScale * f.BayExitSpeedScale,
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
func IsClear(rangesM, anglesRad []float64, cfg Config) bool {
	f := cfg.Follower
	return navutil.ForwardClearance(rangesM, anglesRad, f.ForwardArcHalfFovRad, f.MinValidRangeM) >=
		f.MinForwardClearanceM
}

// resolveOpenSide is which way is out, latched once because it is a fact
// about the layout, matching _resolve_open_side.
//
// Single rays at +/-90 deg, so in the pocket one is the outer wall and the
// other is open corridor. They point ACROSS the pocket only while the
// chassis is still parallel to the wall; once it rotates they point along
// it, at a fin on each side, and a flip inverts the steering sign --
// turning the escape into a re-entry. Hence the latch, and hence counting
// the flips rather than assuming stability.
func (b *BayExit) resolveOpenSide(rangesM, anglesRad []float64, cfg Config) bool {
	left := navutil.NearestRay(rangesM, anglesRad, math.Pi/2)
	right := navutil.NearestRay(rangesM, anglesRad, -math.Pi/2)
	openIsLeft := left > right
	if cfg.Follower.BayExitLatchDirection && b.openIsLeft != nil {
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
// creep speed both legs scale from.
func (b *BayExit) Command(
	rangesM, anglesRad []float64, travelledM, creepSpeedMPS float64, cfg Config,
) controllers.DriveCommand {
	f := cfg.Follower
	if b.reverseStartM == nil {
		rs := travelledM
		b.reverseStartM = &rs
	}

	openIsLeft := b.resolveOpenSide(rangesM, anglesRad, cfg)

	b.ticks++
	// The clearance guard supersedes both contact-bounded exits, so it is
	// answered before their fallback bookkeeping runs at all.
	if f.BayExitClearanceGuard {
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
			SpeedMPS:     -creepSpeedMPS * f.ReverseSpeedScale,
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
	return controllers.DriveCommand{SpeedMPS: creepSpeedMPS * f.CornerSpeedScale, SteeringNorm: steer}
}

// followerMaxSteeringAngleRad is cfg.Follower.MaxSteeringAngleRad, matching
// RobotSpecs.MAX_WHEEL_ANGLE_DEG converted to radians -- reused from
// corridorfollower.Config rather than duplicated, since that field is
// already profile-sourced there.
func (c Config) followerMaxSteeringAngleRad() float64 {
	return c.Follower.MaxSteeringAngleRad
}
