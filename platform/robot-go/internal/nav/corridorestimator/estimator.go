package corridorestimator

import (
	"math"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/racetracker"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// Config is the estimator's tuning, matching CorridorEstimatorParams plus the
// two legal corridor widths and the alignment gate borrowed from the
// direction estimator.
type Config struct {
	// MinSamples is how many readings a corridor must accumulate before its
	// estimate is trusted. Readings are attributed by heading, so a handful
	// taken while the chassis swings through a corner can land in the wrong
	// corridor; requiring a dozen costs about half a second at 20 Hz and
	// makes a single mis-filed sweep unable to move the answer.
	MinSamples int
	// PlausibleWidthMarginM is how far outside the legal band a measured
	// width may fall and still be treated as a plausibility FAILURE (a
	// corner) rather than accepted.
	PlausibleWidthMarginM float64
	// DecisionBoundaryM is the width at which a raw measurement snaps to WIDE
	// rather than NARROW. The midpoint of the two legal widths by default,
	// but tunable independently of them: shifting it off-center trades a
	// false NARROW for a false WIDE without moving either corridor width.
	DecisionBoundaryM float64
	// AlignmentToleranceRad is the maximum heading error against the nearest
	// track axis for a side-ray measurement to be trusted. Shared with the
	// direction estimator, which gates on the same geometry.
	AlignmentToleranceRad float64

	NarrowWidthM float64
	WideWidthM   float64
}

// Measurement is one wall-to-wall width reading, matching
// shared.domain.models.CorridorWidthMeasurement.
type Measurement struct {
	WidthM            float64
	IsPlausible       bool
	IsAligned         bool
	AlignmentErrorRad float64
	SideRangeLeftM    float64
	SideRangeRightM   float64
}

// Shipped defaults, matching
// platform/shared/config/navigation/blind_nav/corridor_estimator.toml and
// track.toml's [corridor] section.
const (
	// DefaultMinSamples matches corridor_estimator.toml's min_samples.
	DefaultMinSamples = 12
	// DefaultPlausibleWidthMarginM matches plausible_width_margin_m.
	DefaultPlausibleWidthMarginM = 0.25
	// DefaultDecisionBoundaryM matches decision_boundary_m.
	DefaultDecisionBoundaryM = 0.80
	// DefaultNarrowWidthM matches track.toml's [corridor] narrow.
	DefaultNarrowWidthM = 0.6
	// DefaultWideWidthM matches track.toml's [corridor] wide.
	DefaultWideWidthM = 1.0
	// DefaultAlignmentToleranceDeg matches the direction estimator's
	// ALIGNMENT_TOLERANCE, which this shares.
	DefaultAlignmentToleranceDeg = 25.0
)

// DefaultConfig returns the Config matching the shipped TOML defaults.
func DefaultConfig() Config {
	return Config{
		MinSamples:            DefaultMinSamples,
		PlausibleWidthMarginM: DefaultPlausibleWidthMarginM,
		DecisionBoundaryM:     DefaultDecisionBoundaryM,
		AlignmentToleranceRad: DefaultAlignmentToleranceDeg * math.Pi / navutil.DegreesPerHalfTurn,
		NarrowWidthM:          DefaultNarrowWidthM,
		WideWidthM:            DefaultWideWidthM,
	}
}

// MeasureCorridorWidth returns the wall-to-wall width through the robot, and
// ok=false when this scan cannot say.
//
// No map and no position estimate are needed: the LIDAR sits at the chassis
// center, so the range directly left plus the range directly right spans wall
// to wall through the robot, wherever in the corridor it happens to be.
//
// Two gates keep bad readings out. ALIGNMENT, because the side rays only span
// the corridor when the chassis is roughly parallel to it -- off-axis they cut
// a longer diagonal. PLAUSIBILITY, because at a corner the inward ray misses
// the inner block entirely and runs off down the next corridor, giving a
// nonsense total.
func MeasureCorridorWidth(
	rangesM []float64,
	anglesRad []float64,
	yaw float64,
	cfg Config,
) (Measurement, bool) {
	if len(rangesM) == 0 || len(rangesM) != len(anglesRad) {
		return Measurement{}, false
	}

	// Heading error against the nearest track axis; corridors always run
	// along one.
	axisError := navutil.WrapAngle(yaw - math.Round(yaw/navutil.QuarterTurnRad)*navutil.QuarterTurnRad)
	isAligned := math.Abs(axisError) <= cfg.AlignmentToleranceRad

	left := navutil.NearestRay(rangesM, anglesRad, navutil.QuarterTurnRad)
	right := navutil.NearestRay(rangesM, anglesRad, -navutil.QuarterTurnRad)

	width := 0.0
	if isAligned {
		// Project the diagonal the off-axis rays actually cut back onto the
		// corridor normal.
		width = (left + right) * math.Cos(axisError)
	}

	minPlausible := cfg.NarrowWidthM - cfg.PlausibleWidthMarginM
	maxPlausible := cfg.WideWidthM + cfg.PlausibleWidthMarginM
	isPlausible := width > minPlausible && width < maxPlausible

	if !isAligned || !isPlausible {
		return Measurement{}, false
	}
	return Measurement{
		WidthM:            width,
		IsPlausible:       true,
		IsAligned:         true,
		AlignmentErrorRad: math.Abs(axisError),
		SideRangeLeftM:    left,
		SideRangeRightM:   right,
	}, true
}

// ClassifyWidth snaps a raw measurement to whichever of the two legal widths
// it is.
func ClassifyWidth(widthM float64, cfg Config) float64 {
	if widthM < cfg.DecisionBoundaryM {
		return cfg.NarrowWidthM
	}
	return cfg.WideWidthM
}

// SectionFromHeading returns which corridor the robot is in, from heading
// alone.
//
// On a rectangular loop driven in a known direction each section is traveled
// along a different bearing -- south-bound on the east side going clockwise,
// north-bound on the west side, and so on -- so the four (section, direction)
// travel vectors are all distinct. Snapping the heading to the nearest one
// identifies the corridor outright, with no reference to the map.
func SectionFromHeading(yaw float64, direction trackmodel.Direction) trackmodel.Section {
	headingX, headingY := math.Cos(yaw), math.Sin(yaw)

	best := trackmodel.North
	bestDot := math.Inf(-1)
	// Iterated in declaration order rather than over a map, so a tie between
	// two equally-aligned sections resolves deterministically.
	for _, section := range []trackmodel.Section{
		trackmodel.North, trackmodel.South, trackmodel.East, trackmodel.West,
	} {
		normal, ok := racetracker.TravelNormalFor(section, direction)
		if !ok {
			continue
		}
		if dot := headingX*normal.NX + headingY*normal.NY; dot > bestDot {
			best, bestDot = section, dot
		}
	}
	return best
}
