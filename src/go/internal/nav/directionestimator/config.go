package directionestimator

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"
)

// Config parameterizes InferDirection, DirectionEstimator, and
// DirectionFromParkingBay. See doc.go for why these are literal defaults
// rather than profile-loaded.
type Config struct {
	// AlignmentToleranceRad is the maximum heading error against the
	// nearest track axis for a side-ray reading to be trusted. Off axis
	// the side rays cut a diagonal and read long for no good reason.
	AlignmentToleranceRad float64
	// MaxInTrackRangeM rejects a side ray longer than this as unable to be
	// a wall of this track -- a LIDAR dropout reads as max range, which
	// reads as "this side is open", exactly the signal being looked for.
	MaxInTrackRangeM float64
	// PlausibleSpanThresholdM is the maximum left+right range sum that
	// still represents one corridor; exceeding it means a ray ran off
	// into an adjacent corridor rather than both reading the current
	// one's walls.
	PlausibleSpanThresholdM float64
	// MinAsymmetryM is the minimum left/right difference for a sweep to
	// count as evidence rather than noise.
	MinAsymmetryM float64
	// MinVotes is the number of agreeing observations DirectionEstimator
	// requires before settling.
	MinVotes int
	// DirectionArcHalfFovDeg is the forward cone half-width (degrees)
	// DirectionFromParkingBay's forward-clearance check uses -- NOT the
	// collision-avoidance front sector's own FOV, a separate, narrower
	// cone (see utils.py's field docstring).
	DirectionArcHalfFovDeg float64
	// MinValidRangeM is the no-return/invalid-reading floor shared by
	// every LIDAR sector check.
	MinValidRangeM float64
	// MinForwardClearanceM is the forward clearance below which
	// DirectionFromParkingBay treats the scan as the boxed-in bay case
	// (matches CorridorFollowerParams.MIN_FORWARD_CLEARANCE_M).
	MinForwardClearanceM float64
	// BayWallClearanceM is the max distance to the nearer side wall for
	// DirectionFromParkingBay to treat the robot as hard against it
	// (matches CorridorFollowerParams.BAY_WALL_CLEARANCE_M).
	BayWallClearanceM float64
	// TurnClearanceM is the min distance to the farther side for
	// DirectionFromParkingBay to treat it as open corridor (matches
	// CorridorFollowerParams.TURN_CLEARANCE_M).
	TurnClearanceM float64
}

// Default* match
// shared.config.navigation_tuning.blind_nav.DirectionEstimatorParams' and
// CorridorFollowerParams'/LidarSectorParams' field defaults.
const (
	DefaultAlignmentToleranceDeg  = 25.0
	DefaultMaxInTrackRangeM       = 4.5
	DefaultPlausibleSpanM         = 1.25
	DefaultMinAsymmetryM          = 0.20
	DefaultMinVotes               = 5
	DefaultDirectionArcHalfFovDeg = 8.0
	DefaultMinValidRangeM         = 0.05
	DefaultMinForwardClearanceM   = 0.30
	DefaultBayWallClearanceM      = 0.20
	DefaultTurnClearanceM         = 0.60
)

// DefaultConfig returns the Config matching the Python tuning defaults.
func DefaultConfig() Config {
	return Config{
		AlignmentToleranceRad:   DefaultAlignmentToleranceDeg * math.Pi / navutil.DegreesPerHalfTurn,
		MaxInTrackRangeM:        DefaultMaxInTrackRangeM,
		PlausibleSpanThresholdM: DefaultPlausibleSpanM,
		MinAsymmetryM:           DefaultMinAsymmetryM,
		MinVotes:                DefaultMinVotes,
		DirectionArcHalfFovDeg:  DefaultDirectionArcHalfFovDeg,
		MinValidRangeM:          DefaultMinValidRangeM,
		MinForwardClearanceM:    DefaultMinForwardClearanceM,
		BayWallClearanceM:       DefaultBayWallClearanceM,
		TurnClearanceM:          DefaultTurnClearanceM,
	}
}
