package waypoints

import "github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"

// Config parameterizes waypoint generation. Field defaults mirror
// shared.config.navigation_tuning.waypoint.WaypointParams' Pydantic
// defaults, restated as literals rather than profile-loaded for the same
// reason internal/nav/directionestimator.Config is (see that package's
// doc.go): internal/config/profile doesn't cover the navigation-tuning
// TOML tree yet.
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
}

// Default* match WaypointParams' field defaults.
const (
	DefaultDedupeDistanceM          = 0.001
	DefaultWideCenterBiasM          = 0.10
	DefaultNarrowCenterBiasM        = 0.0
	DefaultNarrowWidthThresholdM    = 0.8
	DefaultNumIntermediateArcPoints = 3
	DefaultStraightWaypointCount    = 8
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
	}
}
