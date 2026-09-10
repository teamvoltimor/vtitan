package diag

import (
	"math"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
)

// sectorQuery names one angular sector to aggregate a scan over, matching
// the (center_rad, half_fov_rad, filter_self_detection) triple
// clearances_from_scan (platform/robot/src/navigation/clearances.py) passes
// to CollisionAvoidanceController.sector_ranges for each of front/left/right.
type sectorQuery struct {
	CenterRad           float64
	HalfFOVRad          float64
	FilterSelfDetection bool
}

const (
	// frontCenterRad is 0 rad (straight ahead), matching clearances_from_scan's
	// front sector.
	frontCenterRad = 0.0
	// leftCenterRad is +pi/2 (robot's left), matching clearances_from_scan's
	// left sector.
	leftCenterRad = math.Pi / 2
	// rightCenterRad is -pi/2 (robot's right), matching clearances_from_scan's
	// right sector.
	rightCenterRad = -math.Pi / 2
	// fullSweepRad is the angular span of one revolution, used to derive
	// the per-ray step from the ray count.
	//
	// This used to also carry an assumed angle_min of -pi, porting
	// _lidar_clearances (telemetry_bridge_node.py), which synthesizes
	// np.linspace(-pi, pi, n, endpoint=False) rather than reading the
	// message's own angle_min/angle_increment. That assumption is a
	// property of the ROS2 publisher Python listens to (sllidar_ros2, which
	// does publish -pi..pi), not of a scan in general: this stack's
	// lidar-node sorts by corrected bearing and publishes [0, 2pi), so the
	// same synthesis put every bearing half a turn out. The origin now
	// comes from the message.
	fullSweepRad = 2 * math.Pi
)

// sectorMeanM returns the mean range (m) of every valid ray in ranges whose
// synthesized bearing falls within query's sector, or 0 when no ray
// qualifies — matching clearances_from_scan's own
// `front_m = float(reducer(front)) if front.size else 0.0` fallback.
// Mirrors sector_ranges + np.mean from
// platform/robot/src/navigation/control/controllers/collision_avoidance/sectors.py,
// scoped to what the OLED summary path actually exercises (mean aggregate,
// no rear-sector logic — the back reading is never part of TelemetrySummaryWire).
func sectorMeanM(ranges []float32, angleMinRad float64, cfg Config, query sectorQuery) float64 {
	rayCount := len(ranges)
	if rayCount == 0 {
		return 0
	}

	lowerBoundM := cfg.MinValidRangeM
	if query.FilterSelfDetection {
		lowerBoundM = cfg.SelfDetectionThresholdM
	}

	angleStepRad := fullSweepRad / float64(rayCount)
	sumM := 0.0
	validCount := 0
	for index, rawRangeM := range ranges {
		rangeM := float64(rawRangeM)
		bearingRad := navutil.WrapAngle(angleMinRad + float64(index)*angleStepRad)

		if !isValidRangeM(rangeM, lowerBoundM, cfg.MaxValidRangeM) {
			continue
		}
		if inWedge(bearingRad, cfg.BlindWedgeLeft) || inWedge(bearingRad, cfg.BlindWedgeRight) {
			continue
		}
		if math.Abs(navutil.WrapAngle(bearingRad-query.CenterRad)) > query.HalfFOVRad {
			continue
		}

		sumM += rangeM
		validCount++
	}

	if validCount == 0 {
		return 0
	}
	return sumM / float64(validCount)
}

// isValidRangeM reports whether rangeM is a genuine, in-bounds echo: finite,
// strictly above lowerBoundM (no-return / self-detection exclusion) and
// strictly below upperBoundM (excludes the hardware gateway's fabricated
// far-range no-return substitute — see DefaultMaxValidRangeM).
func isValidRangeM(rangeM, lowerBoundM, upperBoundM float64) bool {
	return !math.IsNaN(rangeM) && !math.IsInf(rangeM, 0) && rangeM > lowerBoundM && rangeM < upperBoundM
}

// inWedge reports whether bearingRad falls inside wedge, when enabled.
// Compares the raw (unwrapped) bearing directly against wedge's bounds,
// matching sector_ranges' own unwrapped `(angles >= min) & (angles <= max)`
// blind-wedge check.
func inWedge(bearingRad float64, wedge AngleWedge) bool {
	return wedge.Enabled && bearingRad >= wedge.MinRad && bearingRad <= wedge.MaxRad
}
