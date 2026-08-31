package waypoints

import "github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"

// Config parameterizes waypoint generation. DefaultConfig's field values
// mirror shared.config.navigation_tuning.waypoint.WaypointParams'
// Pydantic defaults; ConfigFor loads the real values from
// platform/shared/config/navigation/waypoint/waypoints.toml via
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
}

// Default* match WaypointParams' field defaults.
const (
	DefaultDedupeDistanceM          = 0.001
	DefaultWideCenterBiasM          = 0.10
	DefaultNarrowCenterBiasM        = 0.0
	DefaultNarrowWidthThresholdM    = 0.8
	DefaultNumIntermediateArcPoints = 3
	DefaultStraightWaypointCount    = 8
	DefaultArcRadius                = 0.45
	DefaultCornerArcAssumeWide      = true
)

// DefaultConfig returns the Config matching the Python tuning defaults:
// both center-bias sides default to Inner, matching
// WIDE_CENTER_BIAS_SIDE/NARROW_CENTER_BIAS_SIDE's "inner" defaults.
func DefaultConfig() Config {
	return Config{
		DedupeDistanceM:          DefaultDedupeDistanceM,
		WideCenterBiasM:          DefaultWideCenterBiasM,
		WideCenterBiasSide:       trackmodel.Inner,
		NarrowCenterBiasM:        DefaultNarrowCenterBiasM,
		NarrowCenterBiasSide:     trackmodel.Inner,
		NarrowWidthThresholdM:    DefaultNarrowWidthThresholdM,
		NumIntermediateArcPoints: DefaultNumIntermediateArcPoints,
		StraightWaypointCount:    DefaultStraightWaypointCount,
		ArcRadius:                DefaultArcRadius,
		CornerArcAssumeWide:      DefaultCornerArcAssumeWide,
	}
}
