package waypoints

import "github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"

// Config parameterizes waypoint generation. DefaultConfig's field values
// mirror shared.config.navigation_tuning.waypoint.WaypointParams'
// Pydantic defaults; ConfigFor loads the real values from
// src/config/navigation/waypoint/waypoints.toml via
// internal/config/profile, falling back to DefaultConfig's literals when
// no config root is supplied or loading fails.
type Config struct {
	// DedupeDistanceM is the distance below which consecutive generated
	// waypoints are treated as duplicates.
	DedupeDistanceM float64
	// WideCenterBiasM/WideCenterBiasSide are the magnitude and boundary
	// CenterBiasForCorridor shifts a WIDE corridor's centerline toward
	// (above NarrowWidthThresholdM).
	WideCenterBiasM    float64
	WideCenterBiasSide trackmodel.CorridorSide
	// NarrowCenterBiasM/NarrowCenterBiasSide are the same, for a NARROW
	// corridor (at or below NarrowWidthThresholdM).
	NarrowCenterBiasM    float64
	NarrowCenterBiasSide trackmodel.CorridorSide
	// UnconfirmedWidthInnerBiasM replaces NarrowCenterBiasM for a narrow
	// corridor still sitting on the blind PRIOR rather than a measurement,
	// always toward the inner block. Matches
	// waypoints.UNCONFIRMED_WIDTH_INNER_BIAS_M.
	//
	// A blind round begins believing every corridor narrow, and both
	// hypotheses share the fixed OUTER wall, so the entire width error lands
	// as a lateral shift of the planned line: believed-narrow (0.6 m) plans
	// at MAX-0.30, confirmed-wide (1.0 m) at MAX-0.60. Confirming moves the
	// line 0.30 m in one 50 ms tick -- ten times what the chassis can travel,
	// measured across six hardware runs, which threw heading error past the
	// crawl threshold and pinned the limiter for 82-100% of the following
	// ticks. Pre-positioning inward while the belief is still a guess shrinks
	// that step to 0.30 - this value.
	//
	// A SEPARATE field rather than a raised NarrowCenterBiasM: that value is
	// 0.0 on evidence, and raising it would pay the cost in corridors that
	// are genuinely narrow, where measured inward tracking drift (~0.07 m)
	// already eats most of the margin. This one is handed straight back on a
	// confirmed-narrow reading.
	//
	// The ceiling is CLEARANCE. Do NOT raise it above 0.05 without
	// re-measuring: 0.15 leaves a truly-narrow corridor 0.053 m of inner
	// margin against that same 0.07 m of drift, and measured -16 cases.
	UnconfirmedWidthInnerBiasM float64
	// DeferCurrentCorridorReplan holds a width change back until the robot
	// has left the corridor it describes, matching
	// waypoints.DEFER_CURRENT_CORRIDOR_REPLAN. Read by
	// internal/nav/widthbelief, not by this package's own planning -- it
	// lives here because it is a field of the same TOML section and belongs
	// with the bias it complements.
	DeferCurrentCorridorReplan bool
	// NarrowWidthThresholdM is the corridor width at or below which
	// Narrow* applies instead of Wide*.
	NarrowWidthThresholdM float64
	// NumIntermediateArcPoints is the number of interior sample points
	// per corner arc.
	NumIntermediateArcPoints int
	// StraightWaypointCount is the number of evenly-spaced waypoints
	// generated along a straight corridor segment.
	StraightWaypointCount int
	// ArcRadius is the ceiling on the corner arc radius (m), forwarded to
	// CornerArcRadius. Each corner sizes its own arc from the two corridors
	// it joins; this only caps the result. Matches waypoints.ARC_RADIUS.
	ArcRadius float64
	// CornerArcAssumeWide sizes every corner as if both corridors were WIDE,
	// making the arc (and therefore the turn-entry point, r back from a 90
	// deg corner) independent of a belief that starts out wrong -- matching
	// waypoints.CORNER_ARC_ASSUME_WIDE. Blind rounds begin believing every
	// corridor narrow, so a narrow->wide corner would otherwise plan a late
	// entry; turning early into a wider corridor is the safe failure.
	CornerArcAssumeWide bool
	// MaxCoordM is the track's outer boundary (profile.TrackConfig.Track.
	// MaxCoord), used for validate_bounds' track-extent check.
	MaxCoordM float64
	// ChassisWidthM is RobotSpecs.WIDTH, used by ValidatePathFeasibility to
	// confirm the chassis fits the narrowest corridor once biased off centre.
	ChassisWidthM float64
	// ObstaclesCenterBiasM matches OBSTACLES_CENTER_BIAS_M: the flat,
	// uniform center-bias override for the Obstacles Challenge, in place of
	// the Wide/Narrow split -- every Obstacles corridor is 1.0 m by rule, so
	// there is no narrow case for it to describe. Callers building an
	// Obstacles round pass this as CenterBiasForCorridor's overrideM
	// (formerly a package-local literal in internal/sim/scenario, now
	// sourced from waypoints.toml like every other bias here).
	ObstaclesCenterBiasM float64
}

// Default* match WaypointParams' field defaults.
const (
	DefaultDedupeDistanceM            = 0.001
	DefaultWideCenterBiasM            = 0.10
	DefaultNarrowCenterBiasM          = 0.0
	DefaultUnconfirmedWidthInnerBiasM = 0.05
	DefaultDeferCurrentCorridorReplan = true
	DefaultNarrowWidthThresholdM      = 0.8
	DefaultNumIntermediateArcPoints   = 3
	DefaultStraightWaypointCount      = 8
	DefaultArcRadius                  = 0.45
	DefaultCornerArcAssumeWide        = true
	// DefaultObstaclesCenterBiasM matches the shipped
	// obstacles_center_bias_m key.
	DefaultObstaclesCenterBiasM = 0.15
)

// DefaultConfig returns the Config matching the Python tuning defaults:
// both center-bias sides default to Inner, matching
// WIDE_CENTER_BIAS_SIDE/NARROW_CENTER_BIAS_SIDE's "inner" defaults.
func DefaultConfig() Config {
	return Config{
		DedupeDistanceM:            DefaultDedupeDistanceM,
		WideCenterBiasM:            DefaultWideCenterBiasM,
		WideCenterBiasSide:         trackmodel.Inner,
		NarrowCenterBiasM:          DefaultNarrowCenterBiasM,
		NarrowCenterBiasSide:       trackmodel.Inner,
		UnconfirmedWidthInnerBiasM: DefaultUnconfirmedWidthInnerBiasM,
		DeferCurrentCorridorReplan: DefaultDeferCurrentCorridorReplan,
		NarrowWidthThresholdM:      DefaultNarrowWidthThresholdM,
		NumIntermediateArcPoints:   DefaultNumIntermediateArcPoints,
		StraightWaypointCount:      DefaultStraightWaypointCount,
		ArcRadius:                  DefaultArcRadius,
		CornerArcAssumeWide:        DefaultCornerArcAssumeWide,
		ObstaclesCenterBiasM:       DefaultObstaclesCenterBiasM,
	}
}
