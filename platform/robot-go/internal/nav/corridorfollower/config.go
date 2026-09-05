package corridorfollower

import (
	"math"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
)

// TurnSide overrides the clearance-based side choice in the back-off and
// corner branches, matching corridor_follower.py's TurnSide.
type TurnSide int

// Config is the follower's tuning, matching CorridorFollowerParams plus the
// handful of values it reads from neighboring sections.
type Config struct {
	// TurnClearanceM is how close the wall ahead must be before the corner
	// branch commits to a turn.
	TurnClearanceM float64
	// NarrowTurnClearanceM replaces TurnClearanceM once the creep-phase width
	// readings classify the corridor as NARROW. The wide-corridor threshold
	// leaves no direction-settling window in a narrow one.
	NarrowTurnClearanceM float64
	// CenteringGainDegPerM is the proportional term on lateral offset.
	//
	// SHIPS AT ZERO. Centring is currently heading-only: the damping term
	// below does the work, and the offset term is disabled rather than
	// removed so it can be re-enabled without re-deriving it. A port that
	// "fixed" this to a non-zero value would change real behavior.
	CenteringGainDegPerM float64
	// HeadingGain damps the centering term against the nearest track axis.
	HeadingGain float64
	// MaxCenteringSteerDeg clamps the centering branch.
	MaxCenteringSteerDeg float64
	// MaxCornerSteerDeg is the corner and back-off branches' own steer angle.
	// Deliberately NOT shared with the centering clamp: one is sized by the arc
	// having to fit inside TurnClearanceM, the other by the 2026-08-07 limit
	// cycle, and after the simulator was calibrated those wanted opposite
	// values.
	MaxCornerSteerDeg float64
	// SteerCapFromCommitDistance re-derives each branch's steering cap from
	// the distance at which that branch commits, instead of every branch
	// sharing MaxCornerSteerDeg. See SteerCapNorm. Matches
	// steer_cap_from_commit_distance.
	SteerCapFromCommitDistance bool
	// CornerSpeedScale and ReverseSpeedScale scale the creep speed in the
	// corner and back-off branches.
	CornerSpeedScale  float64
	ReverseSpeedScale float64
	// TurnArcHalfFovDeg and TurnOpenRangeM define "is there still a way
	// through" -- an arc wide enough to contain the corridor's own axis when
	// the chassis is oblique.
	TurnArcHalfFovDeg float64
	TurnOpenRangeM    float64
	// CornerLeakMarginM is how far past the widest legal corridor a side ray
	// may read before that side stops counting as a wall to center against.
	CornerLeakMarginM float64
	// MinForwardClearanceM is the safety floor: closer than this and there is
	// no room to drive on, whatever the layout.
	MinForwardClearanceM float64
	// MinReverseClearanceM is the equivalent behind, gating the back-off.
	MinReverseClearanceM float64

	// WideWidthM is the widest legal corridor, for the corner-leak limit.
	WideWidthM float64
	// NarrowWidthM is the narrowest legal corridor, used to classify
	// believedWidthM.
	NarrowWidthM float64
	// DecisionBoundaryM snaps a believed width to NARROW or WIDE.
	DecisionBoundaryM float64

	// MaxSteeringAngleRad converts the physical road-wheel angles above into
	// the normalized command the drive port takes.
	MaxSteeringAngleRad float64
	// ForwardArcHalfFovRad and MinValidRangeM parameterize the forward
	// clearance reading.
	ForwardArcHalfFovRad float64
	MinValidRangeM       float64
	// MaxInTrackRangeM rejects a ray longer than anything the mat can contain
	// when testing for a way through.
	MaxInTrackRangeM float64

	// BayExitReverseM is how far to back straight out of the parking pocket
	// before turning, on the legacy reverse-then-swing exit. Bounded by
	// GEOMETRY (no rear sensing on this mount), and sharp -- see the shipped
	// TOML comment before re-sweeping.
	BayExitReverseM float64
	// BayExitSteerNorm is the swing-out steering magnitude, 0..1 of full
	// lock, on the legacy exit. MEASURED INERT 2026-08-29 (something
	// downstream saturates before this reaches the wheels) but kept at full
	// lock since nothing refutes the geometry argument for it.
	BayExitSteerNorm float64
	// BayExitReverseSteerNorm is the steering magnitude DURING the legacy
	// exit's reverse leg, applied with the sign INVERTED the way
	// FollowCorridor's reverse branch already does. REFUTED 2026-08-29 at
	// any non-zero value; kept at 0.0 (straight reverse) so the refutation
	// stays recorded.
	BayExitReverseSteerNorm float64
	// BayExitHoldSteer holds the forward leg's steering through the legacy
	// exit's reverse leg instead of centring, so the servo's slew is not
	// thrown away every cycle.
	BayExitHoldSteer bool
	// BayExitFallbackFrames is the ticks to give the configured exit before
	// switching to the OTHER one; 0 = never. The reverse-then-swing and
	// cycle exits are complementary under the sim contact model (each
	// 254/256 where the other is 0/256), and which applies to the real
	// robot is unknown.
	BayExitFallbackFrames int
	// BayExitCycle selects the alternating arc/straight-reverse exit
	// instead of reverse-then-swing.
	BayExitCycle bool
	// BayExitCycleReverseM is how far the cycle manoeuvre's straight
	// reverse runs before arcing again. Its OWN constant, not shared with
	// BayExitReverseM -- the two manoeuvres want different values for the
	// same-named quantity.
	BayExitCycleReverseM float64
	// BayExitArcSteerNorm is the cycle manoeuvre's forward-arc steering
	// magnitude, 0..1 of full lock. Moderate on purpose: full lock pivots
	// the chassis about its own centre and translates nothing.
	BayExitArcSteerNorm float64
	// BayExitForwardM is how far the cycle manoeuvre's forward arc runs
	// before backing up again. Bounded by GEOMETRY, deliberately longer
	// than the pocket's own slack so overshooting hands the leg's end to
	// the stall backstop.
	BayExitForwardM float64
	// BayExitCycleReverseSteerNorm is the cycle manoeuvre's reverse-leg
	// steering, applied OPPOSITE to the arc (the classic three-point turn).
	// 0 backs straight, which is what ships.
	BayExitCycleReverseSteerNorm float64
	// BayExitClearanceGuard bounds the cycle legs by PREDICTED FIN
	// CLEARANCE instead of by contact -- the legal replacement, since a
	// contact-bounded leg-end IS the 9.24.7 violation. Dead-reckons the
	// chassis pose in the bay frame and models the two fins from
	// ParkingLotSpecs; no LIDAR, no contact.
	BayExitClearanceGuard bool
	// BayExitClearanceMarginM is the fin clearance the guard refuses to go
	// below, absorbing dead-reckoning error. Only meaningful with
	// BayExitClearanceGuard.
	BayExitClearanceMarginM float64
	// BayExitLegStallTicks is ticks of no wheel travel that end a
	// cycle-manoeuvre leg and start the other -- the PRIMARY leg-end
	// signal, ahead of distance or clearance. Only meaningful with
	// BayExitCycle.
	BayExitLegStallTicks int
	// BayExitLatchDirection decides which side is open once, on the first
	// tick, instead of every tick -- re-deriving it mid-manoeuvre reads
	// noise once the chassis has rotated off-parallel to the wall.
	BayExitLatchDirection bool
	// BayExitLatchReverse commits to the forward turn once the legacy
	// exit's reverse leg has finished, instead of re-testing the gate every
	// tick (which chatters between two opposed commands).
	BayExitLatchReverse bool
	// BayExitMaxFrames is the ticks the bay-exit manoeuvre may hold control
	// before handing over; 0 = forever. BayExit is the only manoeuvre in
	// the stack with no give-up path by default.
	BayExitMaxFrames int
}

// TurnSide values. TurnSideNone preserves the plain clearance-based
// behavior, matching Python's `forced_turn_side=None`.
const (
	TurnSideNone TurnSide = iota
	TurnSideLeft
	TurnSideRight
)

// Shipped defaults, matching
// platform/config/navigation/blind_nav/corridor_follower.toml and the
// neighboring sections each cross-referenced value comes from.
const (
	// halvesPerWidth turns a left-minus-right difference into the chassis's
	// offset from the corridor centerline, which is half of it.
	halvesPerWidth = 2.0

	// DefaultTurnClearanceM matches turn_clearance_m.
	DefaultTurnClearanceM = 0.60
	// DefaultNarrowTurnClearanceM matches narrow_turn_clearance_m.
	DefaultNarrowTurnClearanceM = 0.40
	// DefaultCenteringGainDegPerM matches centering_gain_deg_per_m, which
	// ships at zero -- see Config.CenteringGainDegPerM.
	DefaultCenteringGainDegPerM = 0.0
	// DefaultHeadingGain matches heading_gain.
	DefaultHeadingGain = 0.767945
	// DefaultMaxCenteringSteerDeg matches max_centering_steer_deg.
	DefaultMaxCenteringSteerDeg = 13.75
	// DefaultMaxCornerSteerDeg matches max_corner_steer_deg.
	DefaultMaxCornerSteerDeg = 21.25
	// DefaultSteerCapFromCommitDistance matches
	// steer_cap_from_commit_distance.
	DefaultSteerCapFromCommitDistance = true
	// DefaultCornerSpeedScale matches corner_speed_scale.
	DefaultCornerSpeedScale = 0.6
	// DefaultReverseSpeedScale matches reverse_speed_scale.
	DefaultReverseSpeedScale = 0.6
	// DefaultTurnArcHalfFovDeg matches turn_arc_half_fov_deg.
	DefaultTurnArcHalfFovDeg = 15.0
	// DefaultTurnOpenRangeM matches turn_open_range_m.
	DefaultTurnOpenRangeM = 1.00
	// DefaultCornerLeakMarginM matches corner_leak_margin_m.
	DefaultCornerLeakMarginM = 0.35
	// DefaultMinForwardClearanceM matches min_forward_clearance_m.
	DefaultMinForwardClearanceM = 0.30
	// DefaultMinReverseClearanceM matches min_reverse_clearance_m.
	DefaultMinReverseClearanceM = 0.30

	// DefaultWideWidthM/DefaultNarrowWidthM/DefaultDecisionBoundaryM match
	// track.toml's [corridor] section and corridor_estimator.toml.
	DefaultWideWidthM        = 1.0
	DefaultNarrowWidthM      = 0.6
	DefaultDecisionBoundaryM = 0.80

	// DefaultMaxSteeringAngleRad matches controllers.DefaultMaxSteeringAngleRad.
	DefaultMaxSteeringAngleRad = 1.2252
	// DefaultForwardArcHalfFovDeg matches lidar_sectors.DIRECTION_ARC_HALF_FOV_DEG.
	DefaultForwardArcHalfFovDeg = 8.0
	// DefaultMinValidRangeM matches lidar_sectors.MIN_VALID_RANGE_M.
	DefaultMinValidRangeM = 0.05
	// DefaultMaxInTrackRangeM matches direction_estimator.MAX_IN_TRACK_RANGE_M.
	DefaultMaxInTrackRangeM = 4.5

	// DefaultBayExitReverseM matches BAY_EXIT_REVERSE_M.
	DefaultBayExitReverseM = 0.05
	// DefaultBayExitSteerNorm matches BAY_EXIT_STEER_NORM.
	DefaultBayExitSteerNorm = 1.0
	// DefaultBayExitReverseSteerNorm matches BAY_EXIT_REVERSE_STEER_NORM.
	DefaultBayExitReverseSteerNorm = 0.0
	// DefaultBayExitHoldSteer matches BAY_EXIT_HOLD_STEER.
	DefaultBayExitHoldSteer = true
	// DefaultBayExitFallbackFrames matches BAY_EXIT_FALLBACK_FRAMES.
	DefaultBayExitFallbackFrames = 0
	// DefaultBayExitCycle matches BAY_EXIT_CYCLE.
	DefaultBayExitCycle = true
	// DefaultBayExitCycleReverseM matches BAY_EXIT_CYCLE_REVERSE_M.
	DefaultBayExitCycleReverseM = 0.09
	// DefaultBayExitArcSteerNorm matches BAY_EXIT_ARC_STEER_NORM.
	DefaultBayExitArcSteerNorm = 0.3
	// DefaultBayExitForwardM matches BAY_EXIT_FORWARD_M.
	DefaultBayExitForwardM = 0.08
	// DefaultBayExitCycleReverseSteerNorm matches BAY_EXIT_CYCLE_REVERSE_STEER_NORM.
	DefaultBayExitCycleReverseSteerNorm = 0.0
	// DefaultBayExitClearanceGuard matches BAY_EXIT_CLEARANCE_GUARD.
	DefaultBayExitClearanceGuard = false
	// DefaultBayExitClearanceMarginM matches BAY_EXIT_CLEARANCE_MARGIN_M.
	DefaultBayExitClearanceMarginM = 0.005
	// DefaultBayExitLegStallTicks matches BAY_EXIT_LEG_STALL_TICKS.
	DefaultBayExitLegStallTicks = 6
	// DefaultBayExitLatchDirection matches BAY_EXIT_LATCH_DIRECTION.
	DefaultBayExitLatchDirection = true
	// DefaultBayExitLatchReverse matches BAY_EXIT_LATCH_REVERSE.
	DefaultBayExitLatchReverse = false
	// DefaultBayExitMaxFrames matches BAY_EXIT_MAX_FRAMES.
	DefaultBayExitMaxFrames = 0
)

// DefaultConfig returns the Config matching the shipped TOML defaults.
func DefaultConfig() Config {
	return Config{
		TurnClearanceM:       DefaultTurnClearanceM,
		NarrowTurnClearanceM: DefaultNarrowTurnClearanceM,
		CenteringGainDegPerM: DefaultCenteringGainDegPerM,
		HeadingGain:          DefaultHeadingGain,
		MaxCenteringSteerDeg: DefaultMaxCenteringSteerDeg,
		MaxCornerSteerDeg:    DefaultMaxCornerSteerDeg,

		SteerCapFromCommitDistance: DefaultSteerCapFromCommitDistance,
		CornerSpeedScale:           DefaultCornerSpeedScale,
		ReverseSpeedScale:          DefaultReverseSpeedScale,
		TurnArcHalfFovDeg:          DefaultTurnArcHalfFovDeg,
		TurnOpenRangeM:             DefaultTurnOpenRangeM,
		CornerLeakMarginM:          DefaultCornerLeakMarginM,
		MinForwardClearanceM:       DefaultMinForwardClearanceM,
		MinReverseClearanceM:       DefaultMinReverseClearanceM,

		WideWidthM:        DefaultWideWidthM,
		NarrowWidthM:      DefaultNarrowWidthM,
		DecisionBoundaryM: DefaultDecisionBoundaryM,

		MaxSteeringAngleRad:  DefaultMaxSteeringAngleRad,
		ForwardArcHalfFovRad: DefaultForwardArcHalfFovDeg * math.Pi / navutil.DegreesPerHalfTurn,
		MinValidRangeM:       DefaultMinValidRangeM,
		MaxInTrackRangeM:     DefaultMaxInTrackRangeM,

		BayExitReverseM:              DefaultBayExitReverseM,
		BayExitSteerNorm:             DefaultBayExitSteerNorm,
		BayExitReverseSteerNorm:      DefaultBayExitReverseSteerNorm,
		BayExitHoldSteer:             DefaultBayExitHoldSteer,
		BayExitFallbackFrames:        DefaultBayExitFallbackFrames,
		BayExitCycle:                 DefaultBayExitCycle,
		BayExitCycleReverseM:         DefaultBayExitCycleReverseM,
		BayExitArcSteerNorm:          DefaultBayExitArcSteerNorm,
		BayExitForwardM:              DefaultBayExitForwardM,
		BayExitCycleReverseSteerNorm: DefaultBayExitCycleReverseSteerNorm,
		BayExitClearanceGuard:        DefaultBayExitClearanceGuard,
		BayExitClearanceMarginM:      DefaultBayExitClearanceMarginM,
		BayExitLegStallTicks:         DefaultBayExitLegStallTicks,
		BayExitLatchDirection:        DefaultBayExitLatchDirection,
		BayExitLatchReverse:          DefaultBayExitLatchReverse,
		BayExitMaxFrames:             DefaultBayExitMaxFrames,
	}
}
