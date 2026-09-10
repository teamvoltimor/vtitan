package waypoints

import (
	"fmt"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// PathPlannability is the result of checking whether a path fits within
// corridor constraints, matching shared.domain.models.PathPlannability.
type PathPlannability struct {
	IsFeasible    bool
	MinRequiredM  float64
	MinAvailableM float64
	MarginM       float64
	Reason        string
}

// CenterBiasForCorridor is the signed centerline shift for ONE corridor,
// positive toward the inner block, matching generation.py's
// center_bias_for_corridor.
//
// Narrow corridors take cfg.NarrowCenterBiasM and wide ones
// cfg.WideCenterBiasM, split at cfg.NarrowWidthThresholdM -- the same
// absolute shift spends a much larger fraction of a narrow corridor's
// margin than of a wide one's, so a single value is either unsafe narrow
// or slow wide. Magnitude and side are chosen together, from the same
// width class.
//
// overrideM (the Obstacles Challenge's OBSTACLES_CENTER_BIAS_M), when
// non-nil, applies UNIFORMLY with the WIDE side, ignoring the split --
// Obstacles corridors are all 1.0m by rule, so there is no narrow case
// for it to describe.
//
// confirmed says whether this corridor's width has been MEASURED rather
// than assumed. An UNCONFIRMED narrow corridor takes
// cfg.UnconfirmedWidthInnerBiasM toward the inner block instead of
// NarrowCenterBiasM, pre-positioning the line so confirming the width moves
// it less -- see that field for the 0.30 m step this shrinks.
//
// overrideM takes precedence over confirmed: an explicitly-passed magnitude
// was swept and measured as one number, and a belief-dependent substitution
// underneath it would silently make it mean two.
func CenterBiasForCorridor(
	widthM float64,
	cfg Config,
	overrideM *float64,
	confirmed bool,
) float64 {
	var magnitude float64
	var side trackmodel.CorridorSide
	switch {
	case overrideM != nil:
		magnitude, side = *overrideM, cfg.WideCenterBiasSide
	case widthM <= cfg.NarrowWidthThresholdM && !confirmed:
		magnitude, side = cfg.UnconfirmedWidthInnerBiasM, trackmodel.Inner
	case widthM <= cfg.NarrowWidthThresholdM:
		magnitude, side = cfg.NarrowCenterBiasM, cfg.NarrowCenterBiasSide
	default:
		magnitude, side = cfg.WideCenterBiasM, cfg.WideCenterBiasSide
	}
	if side == trackmodel.Inner {
		return magnitude
	}
	return -magnitude
}

// ValidatePathFeasibility checks whether the chassis fits the narrowest
// corridor once biased off center, matching generation.py's
// validate_path_feasibility.
//
// The corner arcs deliberately don't appear here: CornerArcRadius caps
// every arc at the clearance the straights already have, so a corner can
// never be the tightest point on the path. chassisWidthM is
// profile.RobotConfig.Chassis.Width; only centerBiasM's magnitude
// matters, since biasing either way moves the chassis toward one wall by
// the same amount.
func ValidatePathFeasibility(
	minCorridorWidthM, centerBiasM, chassisWidthM float64,
) PathPlannability {
	bias := centerBiasM
	if bias < 0 {
		bias = -bias
	}
	required := chassisWidthM + 2*bias
	margin := minCorridorWidthM - required
	if margin > 0 {
		return PathPlannability{
			IsFeasible: true, MinRequiredM: required, MinAvailableM: minCorridorWidthM, MarginM: margin,
		}
	}
	return PathPlannability{
		IsFeasible:    false,
		MinRequiredM:  required,
		MinAvailableM: minCorridorWidthM,
		MarginM:       margin,
		Reason: fmt.Sprintf(
			"corridor too narrow: required %.3fm, got %.3fm",
			required,
			minCorridorWidthM,
		),
	}
}
