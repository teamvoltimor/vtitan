package controllers

import (
	"math"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/waypoints"
)

// SectorRanges is the aggregated LIDAR measurements over an angular sector,
// matching shared.domain.models.SectorRanges.
type SectorRanges struct {
	BearingRad  float64
	HalfFovRad  float64
	MeanRangeM  float64
	MinRangeM   float64
	MaxRangeM   float64
	ValidCount  int
	WedgeMasked bool
}

// SectorGeometry bundles the mount-specific sector parameters shared by
// every sector query -- the fixed tail of arguments sector_ranges/
// _sector_to_model thread through every call in the Python original,
// grouped here so a caller (CollisionAvoidanceController) builds it once
// rather than repeating an 8+ argument list at each call site.
type SectorGeometry struct {
	// SelfDetectionThresholdM is the range floor used instead of
	// MinValidRangeM when a sector filters self-detection (chassis/cable
	// reflection).
	SelfDetectionThresholdM float64
	// MinValidRangeM is the no-return/invalid-reading floor.
	MinValidRangeM float64
	// NoDataRangeM is the fallback range reported when a sector has no
	// valid rays at all.
	NoDataRangeM float64
	// LidarMaxRangeM is RobotSpecs.LIDAR_MAX_RANGE -- the sensor's spec
	// ceiling, used with NoReturnMarginM to exclude the hardware gateway's
	// fabricated no-return substitute.
	LidarMaxRangeM float64
	// BlindWedgeLeftMinRad/BlindWedgeLeftMaxRad/BlindWedgeRightMinRad/
	// BlindWedgeRightMaxRad are the two rear-corner mount-occlusion
	// wedges, excluded by angle regardless of range.
	BlindWedgeLeftMinRad, BlindWedgeLeftMaxRad   float64
	BlindWedgeRightMinRad, BlindWedgeRightMaxRad float64
}

// MappedObstacle pairs a mapped obstacle's world position with the
// corridor it was observed in, matching mask_mapped_obstacles' mapped_xy
// parameter (a Sequence[tuple[Waypoint, Section]]).
type MappedObstacle struct {
	Position trackmodel.Waypoint
	Corridor trackmodel.Section
}

// NoReturnMarginM matches sectors._NO_RETURN_MARGIN_M: how far below
// LidarMaxRangeM still counts as the hardware gateway's fabricated
// no-return substitute (LIDAR_MAX_RANGE), not a genuine long reading. See
// that constant's Python doc comment for the full reasoning -- the
// hardware gateway sanitizes NaN/inf rays to LIDAR_MAX_RANGE before any
// sector code sees the scan, so a literal Inf check alone is dead code
// against a real scan.
const NoReturnMarginM = 0.05

// Measured reports whether anything in this sector was actually measured.
// The numeric fields cannot answer this on their own -- they all fall back
// to NoDataRangeM, indistinguishable by distance alone from wide-open
// road -- so any gate that reads a range as permission (reverse,
// accelerate, commit to a maneuver) must check this first.
func (s SectorRanges) Measured() bool {
	return s.ValidCount > 0
}

// synthesizeAngles builds a full [-pi, pi) sweep of n angles, matching the
// Python sector helpers' fallback for lidar_angles=None.
func synthesizeAngles(n int) []float64 {
	return navutil.AngleFan(n)
}

// resolveAngles returns anglesRad unchanged if non-nil, else a synthesized
// full sweep sized to rangesM -- matching every sector helper's
// lidar_angles=None fallback.
func resolveAngles(rangesM, anglesRad []float64) []float64 {
	if anglesRad != nil {
		return anglesRad
	}
	return synthesizeAngles(len(rangesM))
}

// SectorRangeValues returns the valid ranges whose bearing falls within
// center +/- halfFovRad, matching sectors.sector_ranges.
//
// filterSelfDetection also discards rays no farther than
// geo.SelfDetectionThresholdM -- mount occlusion or cable clutter
// reflecting the chassis itself, not a real obstacle. Only pass true for
// side/rear sectors: never for the pure-forward bearing, where a genuine
// near-contact inside that radius must still register as a threat.
//
// applyBlindWedgeMask=false skips the wedge exclusion -- used by
// SectorToModel to tell "this bearing is a known blind spot" apart from
// "genuinely nothing out there" by re-running the same query with the mask
// lifted.
func SectorRangeValues(
	rangesM, anglesRad []float64, centerRad, halfFovRad float64, filterSelfDetection bool,
	geo SectorGeometry, applyBlindWedgeMask bool,
) []float64 {
	if len(rangesM) == 0 {
		return nil
	}
	angles := resolveAngles(rangesM, anglesRad)

	minValid := geo.MinValidRangeM
	if filterSelfDetection {
		minValid = geo.SelfDetectionThresholdM
	}

	out := make([]float64, 0, len(rangesM))
	for i, r := range rangesM {
		a := angles[i]
		delta := navutil.WrapAngle(a - centerRad)
		if math.Abs(delta) > halfFovRad {
			continue
		}
		if !(r > minValid) {
			continue
		}
		if !(r < geo.LidarMaxRangeM-NoReturnMarginM) {
			continue
		}
		if math.IsInf(r, 0) || math.IsNaN(r) {
			continue
		}
		if applyBlindWedgeMask && inBlindWedge(a, geo) {
			continue
		}
		out = append(out, r)
	}
	return out
}

// inBlindWedge reports whether raw bearing a (not the sector-relative
// delta) falls inside either rear-corner mount-occlusion wedge.
func inBlindWedge(a float64, geo SectorGeometry) bool {
	inLeft := a >= geo.BlindWedgeLeftMinRad && a <= geo.BlindWedgeLeftMaxRad
	inRight := a >= geo.BlindWedgeRightMinRad && a <= geo.BlindWedgeRightMaxRad
	return inLeft || inRight
}

// SectorToModel computes aggregate metrics for an angular sector as a
// SectorRanges, matching sectors._sector_to_model.
func SectorToModel(
	rangesM, anglesRad []float64,
	centerRad, halfFovRad float64,
	filterSelfDetection bool,
	geo SectorGeometry,
) SectorRanges {
	valid := SectorRangeValues(
		rangesM,
		anglesRad,
		centerRad,
		halfFovRad,
		filterSelfDetection,
		geo,
		true,
	)

	wedgeMasked := false
	if len(valid) == 0 {
		unmasked := SectorRangeValues(
			rangesM,
			anglesRad,
			centerRad,
			halfFovRad,
			filterSelfDetection,
			geo,
			false,
		)
		wedgeMasked = len(unmasked) > 0
	}

	if len(valid) == 0 {
		return SectorRanges{
			BearingRad:  centerRad,
			HalfFovRad:  halfFovRad,
			MeanRangeM:  geo.NoDataRangeM,
			MinRangeM:   geo.NoDataRangeM,
			MaxRangeM:   geo.NoDataRangeM,
			ValidCount:  0,
			WedgeMasked: wedgeMasked,
		}
	}

	sum, minV, maxV := 0.0, valid[0], valid[0]
	for _, v := range valid {
		sum += v
		minV = min(minV, v)
		maxV = max(maxV, v)
	}
	return SectorRanges{
		BearingRad:  centerRad,
		HalfFovRad:  halfFovRad,
		MeanRangeM:  sum / float64(len(valid)),
		MinRangeM:   minV,
		MaxRangeM:   maxV,
		ValidCount:  len(valid),
		WedgeMasked: false,
	}
}

// ForwardPathRanges returns the ranges of points ahead of the robot inside
// its driving lane, matching sectors._forward_path_ranges.
//
// A point at bearing theta (0 = forward) and range r sits at lateral offset
// r*sin(theta) from the robot's centreline. Only points that are ahead
// (cos(theta) > 0) and within pathHalfWidthM of the centreline are in the
// robot's path. A no-return ray fabricated as lidarMaxRangeM (the hardware
// gateway's substitute for a real grazing-incidence echo) is excluded via
// the upper bound, not just literal +Inf -- see NoReturnMarginM.
func ForwardPathRanges(
	rangesM, anglesRad []float64,
	pathHalfWidthM, minValidRangeM, lidarMaxRangeM float64,
) []float64 {
	if len(rangesM) == 0 {
		return nil
	}
	angles := resolveAngles(rangesM, anglesRad)

	out := make([]float64, 0, len(rangesM))
	for i, r := range rangesM {
		a := angles[i]
		// The lateral offset is only consulted for a ray that points ahead,
		// and roughly half a 360-degree fan does not, so computing its Sin
		// before the test evaluated it for every ray in the sweep. Split so
		// the Sin is reached only when it can matter; a NaN bearing still
		// fails the first test exactly as it failed the combined one.
		if !(math.Cos(a) > 0.0) {
			continue
		}
		if !(math.Abs(r*math.Sin(a)) < pathHalfWidthM) {
			continue
		}
		if !(r > minValidRangeM) || !(r < lidarMaxRangeM-NoReturnMarginM) {
			continue
		}
		out = append(out, r)
	}
	return out
}

// ForwardPathHasRays reports whether any ray at all falls geometrically
// inside the forward lane, matching sectors._forward_path_has_rays.
// Ignores validity entirely -- it answers "did the scan sweep this lane",
// not "is the lane clear", letting AssessRisk tell a genuinely-empty scan
// window apart from a lane that had rays but every one was a no-return.
func ForwardPathHasRays(rangesM, anglesRad []float64, pathHalfWidthM float64) bool {
	if len(rangesM) == 0 {
		return false
	}
	angles := resolveAngles(rangesM, anglesRad)
	for i, r := range rangesM {
		a := angles[i]
		if math.Cos(a) > 0.0 && math.Abs(r*math.Sin(a)) < pathHalfWidthM {
			return true
		}
	}
	return false
}

// MaskMappedObstacles blanks the LIDAR returns that land on an obstacle the
// planner already owns, matching sectors.mask_mapped_obstacles.
//
// A traffic sign the sign router is actively routing around is not a
// reactive-layer concern: the planner has a deliberate plan to pass it at a
// gap narrower than the contact threshold, so an unmasked scan would fire
// the escape maneuver on every sign pass. So the split is by provenance,
// not distance: a return attributable to a mapped, actively-routed sign is
// withheld, while walls and unknown returns keep the full guard. A ray is
// only attributed to a sign if its endpoint is within radiusM AND
// robotPose itself is currently in that sign's corridor (cornerMinM/
// cornerMaxM -- see waypoints.CorridorForPosition) -- proximity alone is
// not trustworthy under a wrong-but-consistent believed pose.
//
// Masked rays are set to +Inf rather than dropped, so the returned slice
// stays index-aligned with anglesRad; +Inf is already this package's
// no-return sentinel (SectorRangeValues/ForwardPathRanges both exclude it).
func MaskMappedObstacles(
	rangesM, anglesRad []float64, robotPose trackmodel.Pose, mappedXY []MappedObstacle,
	radiusM, cornerMinM, cornerMaxM float64,
) []float64 {
	out := make([]float64, len(rangesM))
	copy(out, rangesM)
	if len(out) == 0 || len(mappedXY) == 0 || radiusM <= 0.0 {
		return out
	}

	angles := resolveAngles(rangesM, anglesRad)
	robotCorridor := waypoints.CorridorForPosition(robotPose.X, robotPose.Y, cornerMinM, cornerMaxM)

	// Only an obstacle mapped to the robot's OWN corridor can mask a ray, but
	// that is checked after the endpoint -- a Cos, a Sin and a Hypot per ray --
	// has already been computed. When none qualify the loop below provably
	// cannot write to out, so the whole sweep is skippable.
	masksAnything := false
	for _, mapped := range mappedXY {
		if mapped.Corridor == robotCorridor {
			masksAnything = true
			break
		}
	}
	if !masksAnything {
		return out
	}

	for i, r := range out {
		if !math.IsInf(r, 0) && math.IsNaN(r) {
			continue
		}
		if math.IsInf(r, 0) {
			continue // no endpoint to attribute
		}
		bearing := angles[i] + robotPose.Yaw
		endX := robotPose.X + r*math.Cos(bearing)
		endY := robotPose.Y + r*math.Sin(bearing)

		for _, mapped := range mappedXY {
			if mapped.Corridor != robotCorridor {
				continue
			}
			if math.Hypot(endX-mapped.Position.X, endY-mapped.Position.Y) < radiusM {
				out[i] = math.Inf(1)
				break
			}
		}
	}
	return out
}
