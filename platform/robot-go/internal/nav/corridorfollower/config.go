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
}

// TurnSide values. TurnSideNone preserves the plain clearance-based
// behavior, matching Python's `forced_turn_side=None`.
const (
	TurnSideNone TurnSide = iota
	TurnSideLeft
	TurnSideRight
)

// Shipped defaults, matching
// platform/shared/config/navigation/blind_nav/corridor_follower.toml and the
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
		CornerSpeedScale:     DefaultCornerSpeedScale,
		ReverseSpeedScale:    DefaultReverseSpeedScale,
		TurnArcHalfFovDeg:    DefaultTurnArcHalfFovDeg,
		TurnOpenRangeM:       DefaultTurnOpenRangeM,
		CornerLeakMarginM:    DefaultCornerLeakMarginM,
		MinForwardClearanceM: DefaultMinForwardClearanceM,
		MinReverseClearanceM: DefaultMinReverseClearanceM,

		WideWidthM:        DefaultWideWidthM,
		NarrowWidthM:      DefaultNarrowWidthM,
		DecisionBoundaryM: DefaultDecisionBoundaryM,

		MaxSteeringAngleRad:  DefaultMaxSteeringAngleRad,
		ForwardArcHalfFovRad: DefaultForwardArcHalfFovDeg * math.Pi / navutil.DegreesPerHalfTurn,
		MinValidRangeM:       DefaultMinValidRangeM,
		MaxInTrackRangeM:     DefaultMaxInTrackRangeM,
	}
}
