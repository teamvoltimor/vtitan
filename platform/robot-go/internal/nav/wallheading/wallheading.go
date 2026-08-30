// Package wallheading reads absolute heading off the walls, to bound what
// the gyro cannot. Ports src/navigation/wall_heading.py.
//
// Heading is the only quantity in the state estimate that nothing corrects.
// Position is anchored by internal/nav/localization matching scans against
// the walls, but that solver takes yaw AS GIVEN -- so yaw comes from the IMU
// alone, and a BNO085 in UART-RVC mode is 6-axis with no magnetometer and no
// absolute reference. Its error is a ramp, not a bound.
//
// The measurements say that is the axis that decides rounds. Across the 28
// Open Challenge fixtures, 20 cm of position error costs nothing at all,
// while 0.1 deg/s of gyro drift takes 28/28 to 6/28, and 0.5% of gyro scale
// error takes it to 25/28.
//
// # The observation
//
// The track is a Manhattan world: the outer boundary and every face of the
// inner block run along x or y. So in the robot frame every wall segment lies
// at -yaw modulo 90 degrees, and a single sweep therefore OBSERVES absolute
// heading -- mod 90, with the ambiguity resolved by whichever of the four
// candidates is nearest the IMU's own estimate.
//
// That makes heading bounded by the same geometry that already bounds
// position, using a sensor the robot already has, and it does not care how
// long the round has been running.
//
// # How
//
// Consecutive returns lying on the same surface give a segment whose
// direction is a wall direction. Taking the circular mean of 4*theta folds
// the four 90-degree-apart candidates onto one another, so they reinforce
// instead of canceling; dividing the resulting angle by 4 recovers the
// offset.
//
// Traffic signs and parking blocks are axis-aligned in this track too, so
// they contribute to the same estimate rather than corrupting it. A ROTATED
// obstacle would corrupt it, which is why this is a weighted vote with a
// concentration test rather than a mean over everything returned.
package wallheading

import (
	"math"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
)

// Config is the wall-heading tuning, matching WallHeadingParams.
type Config struct {
	MinConcentration float64
	BaselineRays     int
	MaxSegmentJumpM  float64
	MinSegmentM      float64
	NearMaxRangeM    float64
	MinReturns       int
}

// Shipped defaults, matching
// platform/shared/config/navigation/sensors/wall_heading.toml.
const (
	// DefaultMinConcentration matches min_concentration.
	DefaultMinConcentration = 0.55
	// DefaultBaselineRays matches baseline_rays.
	DefaultBaselineRays = 15
	// DefaultMaxSegmentJumpM matches max_segment_jump_m.
	DefaultMaxSegmentJumpM = 0.30
	// DefaultMinSegmentM matches min_segment_m.
	DefaultMinSegmentM = 0.02
	// DefaultNearMaxRangeM matches near_max_range_m.
	DefaultNearMaxRangeM = 11.0
	// DefaultMinReturns matches min_returns.
	DefaultMinReturns = 3

	// quarterTurn is the period of the wall-direction ambiguity: a Manhattan
	// world's walls repeat every 90 degrees.
	quarterTurn = math.Pi / 2
	// foldFactor maps directions 90 degrees apart onto the same angle, so
	// the four candidates reinforce instead of canceling.
	foldFactor = 4.0
)

// DefaultConfig returns the Config matching the shipped TOML defaults.
func DefaultConfig() Config {
	return Config{
		MinConcentration: DefaultMinConcentration,
		BaselineRays:     DefaultBaselineRays,
		MaxSegmentJumpM:  DefaultMaxSegmentJumpM,
		MinSegmentM:      DefaultMinSegmentM,
		NearMaxRangeM:    DefaultNearMaxRangeM,
		MinReturns:       DefaultMinReturns,
	}
}

// EstimateYawFromWalls returns the absolute yaw implied by the wall
// directions in this sweep, and ok=false when the sweep shows no clear
// rectilinear structure.
//
// rangesM must already be sanitized (no NaN/inf). priorYaw is used ONLY to
// pick which of the four 90-degree-apart candidates is meant -- it does not
// pull the answer, which is decided entirely by the walls.
func EstimateYawFromWalls(
	rangesM []float64,
	anglesRad []float64,
	priorYaw float64,
	cfg Config,
) (yaw float64, ok bool) {
	if len(rangesM) < cfg.MinReturns || len(rangesM) != len(anglesRad) {
		return 0, false
	}
	baseline := cfg.BaselineRays
	if baseline <= 0 || len(rangesM) <= baseline {
		return 0, false
	}

	// Points in the robot frame.
	xs := make([]float64, len(rangesM))
	ys := make([]float64, len(rangesM))
	for i, r := range rangesM {
		xs[i] = r * math.Cos(anglesRad[i])
		ys[i] = r * math.Sin(anglesRad[i])
	}

	// Weighted circular mean of the folded segment directions, accumulated
	// as a resultant vector rather than a complex sum -- same quantity as
	// Python's sum(w * exp(4j*theta)), just kept in two components.
	var sumRe, sumIm, sumWeights float64
	for i := 0; i+baseline < len(rangesM); i++ {
		near, far := rangesM[i], rangesM[i+baseline]
		dx, dy := xs[i+baseline]-xs[i], ys[i+baseline]-ys[i]
		segLen := math.Hypot(dx, dy)

		// Both endpoints must be real returns on ONE surface.
		if near >= cfg.NearMaxRangeM || far >= cfg.NearMaxRangeM ||
			math.Abs(far-near) >= cfg.MaxSegmentJumpM ||
			segLen <= cfg.MinSegmentM {
			continue
		}

		folded := foldFactor * math.Atan2(dy, dx)
		// Weighted by segment length, so a long wall outvotes a short
		// fragment.
		sumRe += segLen * math.Cos(folded)
		sumIm += segLen * math.Sin(folded)
		sumWeights += segLen
	}

	if sumWeights == 0 {
		return 0, false
	}
	resultantRe, resultantIm := sumRe/sumWeights, sumIm/sumWeights
	if math.Hypot(resultantRe, resultantIm) < cfg.MinConcentration {
		return 0, false
	}

	// Offset of the wall grid in the robot frame, in [-45, 45) degrees.
	offset := math.Atan2(resultantIm, resultantRe) / foldFactor
	// Robot-frame wall direction is -yaw mod 90, so yaw is -offset mod 90.
	candidate := -offset
	// Pick the candidate nearest the prior: the walls decide the value, the
	// prior only decides which quadrant is meant.
	quadrant := math.Round((priorYaw - candidate) / quarterTurn)
	return candidate + quadrant*quarterTurn, true
}

// HeadingError is the signed difference between a wall-derived yaw and the
// current estimate.
func HeadingError(measuredYaw, priorYaw float64) float64 {
	return navutil.WrapAngle(measuredYaw - priorYaw)
}
