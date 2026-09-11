package controllers_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
)

// -- TestSelfDetectionFilter: chassis/cable reflections at <= 0.08m on
// side/rear sectors must not permanently read as a wall.

// TestComputeRearClearance_SelfReflectionDoesNotBlockReverse ports
// test_rear_self_reflection_does_not_block_reverse.
func TestComputeRearClearance_SelfReflectionDoesNotBlockReverse(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, math.Pi)
	setSector(ranges, i, 4, 0.05) // inside the self-detection radius

	if got := controller.ComputeRearClearance(scanObj(ranges, angles)); got <= 5.0 {
		t.Errorf("ComputeRearClearance() = %v, want > 5.0", got)
	}
}

// TestComputeRearClearance_RealWallBeyondTheChassisIsStillDetected ports
// test_rear_real_wall_beyond_the_chassis_is_still_detected.
//
// 0.35 m is OUTSIDE the body: the chassis rear face sits
// LidarToRearBumperM = 0.2722 m behind the sensor, so with
// RearSelfDetectionFromChassis (the shipped value), anything no farther
// than that at this bearing is the robot seeing itself, not a wall. This
// used to read 0.15 m, which the OLD 0.08 m scalar accepted as "beyond
// self-detection radius" but which is not a place a wall can physically
// be -- exactly the defect the chassis-geometry filter exists to catch (see
// TestComputeRearClearance_ChassisReturnIsNotReportedAsAWall).
func TestComputeRearClearance_RealWallBeyondTheChassisIsStillDetected(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, math.Pi)
	setSector(ranges, i, 4, 0.35) // outside the chassis: a real return

	if got := controller.ComputeRearClearance(scanObj(ranges, angles)); math.Abs(got-0.35) > 1e-9 {
		t.Errorf("ComputeRearClearance() = %v, want %v", got, 0.35)
	}
}

// TestComputeRearClearance_ChassisReturnIsNotReportedAsAWall ports
// test_a_rear_return_inside_the_chassis_is_the_robot_not_a_wall: the defect
// that suppressed every escape for a whole hardware round
// (run_20260906_192424) -- a rear return well inside the chassis boundary
// (0.125 m, against a 0.2722 m rear face) must not become the rear sector's
// minimum via the uniform self-detection scalar (which sits at 0.08 m,
// entirely inside the body at this bearing).
func TestComputeRearClearance_ChassisReturnIsNotReportedAsAWall(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, math.Pi)
	setSector(ranges, i, 4, 0.125) // inside the chassis at this bearing

	rear := controller.RearSector(scanObj(ranges, angles))
	if !rear.Measured() {
		t.Fatal("RearSector().Measured() = false, want true (rest of the sector is open)")
	}
	if rear.MinRangeM <= controllers.DefaultLidarToRearBumperM {
		t.Errorf("RearSector().MinRangeM = %v, want > %v (the chassis rear face)",
			rear.MinRangeM, controllers.DefaultLidarToRearBumperM)
	}
	if got := controller.ComputeRearClearance(scanObj(ranges, angles)); math.Abs(got-0.125) < 1e-9 {
		t.Errorf("ComputeRearClearance() = %v, want != 0.125 (must not report the self-return)", got)
	}
}

// TestDetectThreatDirection_SideSelfReflectionDoesNotReportAsThreat ports
// test_side_self_reflection_does_not_report_as_threat.
func TestDetectThreatDirection_SideSelfReflectionDoesNotReportAsThreat(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, math.Pi/2)
	setSector(ranges, i, 4, 0.05)

	if got := controller.DetectThreatDirection(scanObj(ranges, angles)); got != controllers.ThreatNone {
		t.Errorf("DetectThreatDirection() = %v, want %v", got, controllers.ThreatNone)
	}
}

// TestDetectThreatDirection_ForwardNearContactIsNotFiltered ports
// test_forward_near_contact_is_not_filtered: the forward bearing must never
// be self-detection filtered -- a genuine near-contact obstacle has to
// still register. 0.06 sits just above MIN_VALID_RANGE_M (0.05), deliberate
// so this pins the self-detection exemption, not the invalid-reading floor.
func TestDetectThreatDirection_ForwardNearContactIsNotFiltered(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, 0.0)
	setSector(ranges, i, 4, 0.06)

	if got := controller.DetectThreatDirection(scanObj(ranges, angles)); got != controllers.ThreatFront {
		t.Errorf("DetectThreatDirection() = %v, want %v", got, controllers.ThreatFront)
	}
}

// -- TestNoReturnRaysExcluded: a single no-return (+inf) ray inside a
// sector must not poison that sector's mean/min/max to infinity.

// TestComputeForwardClearance_IgnoresAStrayNoReturnRay ports
// test_forward_clearance_ignores_a_stray_no_return_ray.
func TestComputeForwardClearance_IgnoresAStrayNoReturnRay(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, 0.0)
	ranges[i] = math.Inf(1) // one no-return ray inside the forward sector

	clearance := controller.ComputeForwardClearance(scanObj(ranges, angles))
	if !math.IsInf(clearance, 0) && math.IsNaN(clearance) {
		t.Fatalf("ComputeForwardClearance() = %v, want a finite value", clearance)
	}
	if math.Abs(clearance-lidarDefaultFar) > 1e-9 {
		t.Errorf("ComputeForwardClearance() = %v, want %v", clearance, lidarDefaultFar)
	}
}

// TestComputeForwardClearance_AllNoReturnFallsBackToDefault ports
// test_forward_clearance_all_no_return_falls_back_to_default.
func TestComputeForwardClearance_AllNoReturnFallsBackToDefault(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(math.Inf(1))

	if got := controller.ComputeForwardClearance(scanObj(ranges, angles)); got != 10.0 {
		t.Errorf("ComputeForwardClearance() = %v, want 10.0", got)
	}
}

// -- TestForwardPathRisk: risk is judged over the forward driving lane, not
// the full 360 sweep.

// TestAssessRisk_SideWallsAreNotRisk ports test_side_walls_are_not_risk:
// close walls to the left/right, outside the driving lane, must not read as
// risk -- otherwise speed pins to a crawl for a whole lap.
func TestAssessRisk_SideWallsAreNotRisk(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	for _, center := range []float64{math.Pi / 2, -math.Pi / 2} {
		i := angleToIndex(angles, center)
		setSector(ranges, i, 6, 0.35)
	}

	if got := controller.AssessRisk(scanObj(ranges, angles)); got != controllers.RiskSafe {
		t.Errorf("AssessRisk() = %v, want %v", got, controllers.RiskSafe)
	}
}

// TestAssessRisk_ForwardObstacleWithinContactIsCritical ports
// test_forward_obstacle_within_contact_is_critical.
func TestAssessRisk_ForwardObstacleWithinContactIsCritical(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, 0.0)
	setSector(ranges, i, 4, 0.08) // < contact_dist (0.10)

	if got := controller.AssessRisk(scanObj(ranges, angles)); got != controllers.RiskCritical {
		t.Errorf("AssessRisk() = %v, want %v", got, controllers.RiskCritical)
	}
}

// TestAssessRisk_ForwardObstacleInSlowZoneIsObstacle ports
// test_forward_obstacle_in_slow_zone_is_obstacle.
func TestAssessRisk_ForwardObstacleInSlowZoneIsObstacle(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, 0.0)
	setSector(ranges, i, 4, 0.20) // contact_dist < 0.20 < slow_dist (0.25)

	if got := controller.AssessRisk(scanObj(ranges, angles)); got != controllers.RiskObstacle {
		t.Errorf("AssessRisk() = %v, want %v", got, controllers.RiskObstacle)
	}
}

// TestAssessRisk_WholeForwardLaneNoReturnIsCriticalNotSafe ports
// test_whole_forward_lane_no_return_is_critical_not_safe: the hardware
// gateway sanitizes NaN/inf no-return rays to LIDAR_MAX_RANGE before this
// ever sees them -- grazing incidence off something very close reads
// exactly like this. A forward lane where every ray is this fabricated
// value must NOT read as "wide open, SAFE": that was the root cause of a
// real 60cm-corridor corner going undetected on hardware.
func TestAssessRisk_WholeForwardLaneNoReturnIsCriticalNotSafe(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, 0.0)
	setSector(ranges, i, 4, controller.Geometry.LidarMaxRangeM)

	if got := controller.AssessRisk(scanObj(ranges, angles)); got != controllers.RiskCritical {
		t.Errorf("AssessRisk() = %v, want %v", got, controllers.RiskCritical)
	}
}

// TestAssessRisk_GenuinelyEmptyScanIsStillSafe ports
// test_genuinely_empty_scan_is_still_safe: only a lane that HAD rays but
// every one was a no-return escalates (previous test) -- a scan with
// nothing in it at all has nothing to judge and must stay SAFE, or every
// startup tick before the first scan arrives would read as a false
// emergency.
func TestAssessRisk_GenuinelyEmptyScanIsStillSafe(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	if got := controller.AssessRisk(controllers.LidarScan{}); got != controllers.RiskSafe {
		t.Errorf("AssessRisk(controllers.LidarScan{}) = %v, want %v", got, controllers.RiskSafe)
	}
}

// -- TestBlindWedgeMasking: the rear-left/-right mount wedges self-collide
// at every range, not just close ones, so these rays must be excluded by
// angle regardless of range.

// TestDetectThreatDirection_LeftWedgeSelfCollisionDoesNotRegister ports
// test_left_wedge_self_collision_does_not_register_as_back_threat.
func TestDetectThreatDirection_LeftWedgeSelfCollisionDoesNotRegister(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, -140.0*math.Pi/180.0) // inside the left wedge (-180..-115)
	setSector(
		ranges,
		i,
		4,
		0.02,
	) // self-collision range, would otherwise scream "threat"

	if got := controller.DetectThreatDirection(scanObj(ranges, angles)); got != controllers.ThreatNone {
		t.Errorf("DetectThreatDirection() = %v, want %v", got, controllers.ThreatNone)
	}
}

// TestDetectThreatDirection_RightWedgeSelfCollisionDoesNotRegister ports
// test_right_wedge_self_collision_does_not_register_as_back_threat.
func TestDetectThreatDirection_RightWedgeSelfCollisionDoesNotRegister(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, 140.0*math.Pi/180.0) // inside the right wedge (115..180)
	setSector(ranges, i, 4, 0.02)

	if got := controller.DetectThreatDirection(scanObj(ranges, angles)); got != controllers.ThreatNone {
		t.Errorf("DetectThreatDirection() = %v, want %v", got, controllers.ThreatNone)
	}
}

// TestDetectThreatDirection_RealWallJustOutsideLeftWedgeStillDetected ports
// test_real_wall_just_outside_left_wedge_still_detected.
func TestDetectThreatDirection_RealWallJustOutsideLeftWedgeStillDetected(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, -110.0*math.Pi/180.0) // just outside the wedge (< -115 boundary)
	setSector(
		ranges,
		i,
		4,
		0.15,
	) // above self_detection_threshold_m (0.08): a real return

	if got := controller.DetectThreatDirection(scanObj(ranges, angles)); got != controllers.ThreatRight {
		t.Errorf("DetectThreatDirection() = %v, want %v", got, controllers.ThreatRight)
	}
}

// TestComputeRearClearance_IgnoresWedgeSelfCollision ports
// test_rear_clearance_ignores_wedge_self_collision.
func TestComputeRearClearance_IgnoresWedgeSelfCollision(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, 150.0*math.Pi/180.0) // inside the right wedge, within the rear sector
	setSector(ranges, i, 4, 0.02)

	if got := controller.ComputeRearClearance(scanObj(ranges, angles)); got <= minRearClearance {
		t.Errorf("ComputeRearClearance() = %v, want > %v", got, minRearClearance)
	}
}
