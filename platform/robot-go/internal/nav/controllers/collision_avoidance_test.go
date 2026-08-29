package controllers_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
)

// -- TestThreatDirection: ports test_collision_avoidance_controller.py's
// TestThreatDirection class. A wall behind the robot must report "back",
// never "front" -- on the current chassis the rear mount sits in the
// LIDAR's blind wedge, so a wall directly behind actually reports NONE,
// which is the regression these pin.

// TestDetectThreatDirection_WallBehindReportsNoneNotFront ports
// test_wall_behind_reports_back_not_front.
func TestDetectThreatDirection_WallBehindReportsNoneNotFront(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	// Close returns near +-pi (rear cone), including index 0 (= -pi).
	for k := range rearSectorIndices {
		ranges[k] = lidarCloseThreat
	}
	for k := numRays - rearSectorIndices; k < numRays; k++ {
		ranges[k] = lidarCloseThreat
	}

	if got := controller.DetectThreatDirection(ranges, angles); got != controllers.ThreatNone {
		t.Errorf(
			"DetectThreatDirection() = %v, want %v (rear is masked by the blind wedge)", got, controllers.ThreatNone,
		)
	}
}

// TestDetectThreatDirection_WallBehindBackEvenWithoutAngles ports
// test_wall_behind_back_even_without_angles: when angles are omitted, index
// 0 is assumed to be -pi, so a close ray there is still the rear.
func TestDetectThreatDirection_WallBehindBackEvenWithoutAngles(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	ranges := newScan(lidarDefaultFar)
	for k := range rearSectorIndices {
		ranges[k] = lidarCloseThreat
	}
	for k := numRays - rearSectorIndices; k < numRays; k++ {
		ranges[k] = lidarCloseThreat
	}

	got := controller.DetectThreatDirection(ranges, nil)
	if got != controllers.ThreatNone {
		t.Errorf("DetectThreatDirection(nil angles) = %v, want %v (rear masked)", got, controllers.ThreatNone)
	}
}

// TestDetectThreatDirection_WallAheadReportsFront ports
// test_wall_ahead_reports_front.
func TestDetectThreatDirection_WallAheadReportsFront(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, 0.0)
	setSector(ranges, i, forwardSectorIndices, lidarCloseThreat)

	if got := controller.DetectThreatDirection(ranges, angles); got != controllers.ThreatFront {
		t.Errorf("DetectThreatDirection() = %v, want %v", got, controllers.ThreatFront)
	}
}

// TestDetectThreatDirection_ObstacleLeftReportsLeft ports
// test_obstacle_left_reports_left.
func TestDetectThreatDirection_ObstacleLeftReportsLeft(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, math.Pi/2) // +pi/2 = left
	setSector(ranges, i, forwardSectorIndices, lidarCloseThreat)

	if got := controller.DetectThreatDirection(ranges, angles); got != controllers.ThreatLeft {
		t.Errorf("DetectThreatDirection() = %v, want %v", got, controllers.ThreatLeft)
	}
}

// TestDetectThreatDirection_AllClearReportsNone ports
// test_all_clear_reports_none.
func TestDetectThreatDirection_AllClearReportsNone(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()

	if got := controller.DetectThreatDirection(newScan(lidarDefaultFar), angles); got != controllers.ThreatNone {
		t.Errorf("DetectThreatDirection() = %v, want %v", got, controllers.ThreatNone)
	}
}

// -- TestClearance: ports the TestClearance class.

// TestComputeForwardClearance_IgnoresRearWall ports
// test_forward_clearance_ignores_rear_wall: the path ahead is clear even
// though a wall sits behind.
func TestComputeForwardClearance_IgnoresRearWall(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	for k := range rearSectorIndices {
		ranges[k] = lidarCloseThreat
	}
	for k := numRays - rearSectorIndices; k < numRays; k++ {
		ranges[k] = lidarCloseThreat
	}

	if got := controller.ComputeForwardClearance(ranges, angles); got <= minForwardClearance {
		t.Errorf("ComputeForwardClearance() = %v, want > %v", got, minForwardClearance)
	}
}

// TestComputeRearClearance_RearWallReadsAsNoData ports
// test_rear_clearance_sees_rear_wall: the rear mount sits in the LIDAR's
// blind wedge on the current chassis, so rear rays are masked and rear
// clearance reads as the no-data fallback rather than the actual wall
// distance.
func TestComputeRearClearance_RearWallReadsAsNoData(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	for k := range rearSectorIndices {
		ranges[k] = lidarCloseThreat
	}
	for k := numRays - rearSectorIndices; k < numRays; k++ {
		ranges[k] = lidarCloseThreat
	}

	if got := controller.ComputeRearClearance(ranges, angles); got != controller.Geometry.NoDataRangeM {
		t.Errorf("ComputeRearClearance() = %v, want %v", got, controller.Geometry.NoDataRangeM)
	}
}

// TestComputeRearClearance_ClearWhenOnlyFrontBlocked ports
// test_rear_clearance_clear_when_only_front_blocked.
func TestComputeRearClearance_ClearWhenOnlyFrontBlocked(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, 0.0)
	setSector(ranges, i, forwardSectorIndices, lidarCloseThreat)

	if got := controller.ComputeRearClearance(ranges, angles); got <= minRearClearance {
		t.Errorf("ComputeRearClearance() = %v, want > %v", got, minRearClearance)
	}
}

// -- TestRearSectorVisibility: a reverse gate must be able to tell "nothing
// behind" from "cannot see".

// TestRearSector_NormalScanIsNotMeasured ports test_normal_scan_is_measured:
// on the current chassis the rear mount occupies the LIDAR's blind wedge, so
// even a normal scan's rear sector is masked and reports as not measured --
// "cannot see" rather than "nothing behind".
func TestRearSector_NormalScanIsNotMeasured(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	if controller.RearSector(newScan(lidarDefaultFar), anglesFullRotation()).Measured() {
		t.Error("RearSector(...).Measured() = true, want false (rear mount is in the blind wedge)")
	}
}

// TestRearSector_NoValidRearRaysIsNotMeasured ports
// test_no_valid_rear_rays_is_not_measured: every ray in the rear half is a
// no-return, the case a chassis whose mount occludes the last ~25deg slot
// would see on every scan. The clearance number alone cannot express this --
// it reads as wide-open road, which is what made the reverse guard fail
// open.
func TestRearSector_NoValidRearRaysIsNotMeasured(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	for i, a := range angles {
		if math.Abs(a) >= math.Pi/2 {
			ranges[i] = 0.0
		}
	}

	if got := controller.ComputeRearClearance(ranges, angles); got != controller.Geometry.NoDataRangeM {
		t.Errorf("ComputeRearClearance() = %v, want %v", got, controller.Geometry.NoDataRangeM)
	}
	if controller.RearSector(ranges, angles).Measured() {
		t.Error("RearSector(...).Measured() = true, want false")
	}
}

// TestRearSector_EmptyScanIsNotMeasured ports test_empty_scan_is_not_measured.
func TestRearSector_EmptyScanIsNotMeasured(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	if controller.RearSector(nil, nil).Measured() {
		t.Error("RearSector(nil, nil).Measured() = true, want false")
	}
}

// -- TestEscapeDoesNotReverseIntoRearWall.

// TestComputeEscapeManeuver_RearThreatYieldsNoManeuver ports
// test_rear_threat_yields_no_kturn: a rear wall classifies as ThreatBack;
// the escape table only reverses for a ThreatFront, so a rear wall never
// triggers a reverse K-turn.
func TestComputeEscapeManeuver_RearThreatYieldsNoManeuver(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	_, ok := controller.ComputeEscapeManeuver(controllers.RiskCritical, controllers.ThreatBack, nil, nil, nil)
	if ok {
		t.Error("ComputeEscapeManeuver(back) ok = true, want false")
	}
}

// -- TestKTurnSteersTowardClearerSide: Ackermann reverse flips yaw response:
// +steering swings the nose right while reversing, -steering swings it
// left. The K-turn must pick its sign from which side is actually clearer.

// TestComputeEscapeManeuver_SteersLeftWhenLeftIsClearer ports
// test_steers_left_when_left_is_clearer.
func TestComputeEscapeManeuver_SteersLeftWhenLeftIsClearer(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, -math.Pi/2) // tighten the right side
	setSector(ranges, i, 6, 0.15)

	maneuver, ok := controller.ComputeEscapeManeuver(
		controllers.RiskCritical, controllers.ThreatFront, ranges, angles, nil,
	)
	if !ok {
		t.Fatal("ComputeEscapeManeuver() ok = false, want true")
	}
	if maneuver.Steering >= 0 {
		t.Errorf("Steering = %v, want < 0 (swing toward the clearer left side)", maneuver.Steering)
	}
}

// TestComputeEscapeManeuver_SteersRightWhenRightIsClearer ports
// test_steers_right_when_right_is_clearer.
func TestComputeEscapeManeuver_SteersRightWhenRightIsClearer(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, math.Pi/2) // tighten the left side
	setSector(ranges, i, 6, 0.15)

	maneuver, ok := controller.ComputeEscapeManeuver(
		controllers.RiskCritical, controllers.ThreatFront, ranges, angles, nil,
	)
	if !ok {
		t.Fatal("ComputeEscapeManeuver() ok = false, want true")
	}
	if maneuver.Steering <= 0 {
		t.Errorf("Steering = %v, want > 0 (swing toward the clearer right side)", maneuver.Steering)
	}
}

// TestComputeEscapeManeuver_DefaultsToRightWithoutLidarData ports
// test_defaults_to_right_without_lidar_data.
func TestComputeEscapeManeuver_DefaultsToRightWithoutLidarData(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	maneuver, ok := controller.ComputeEscapeManeuver(controllers.RiskCritical, controllers.ThreatFront, nil, nil, nil)
	if !ok {
		t.Fatal("ComputeEscapeManeuver() ok = false, want true")
	}
	if maneuver.Steering <= 0 {
		t.Errorf("Steering = %v, want > 0 (default without LIDAR data)", maneuver.Steering)
	}
}

// -- TestSideCorrectionFlipsSignWhenReversing.

// TestComputeEscapeManeuver_LeftThreatCreepingSteersRight ports
// test_left_threat_creeping_steers_right.
func TestComputeEscapeManeuver_LeftThreatCreepingSteersRight(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, math.Pi/2)
	setSector(ranges, i, 6, 0.20) // left threat, not yet touching (> contact_dist)

	maneuver, ok := controller.ComputeEscapeManeuver(
		controllers.RiskCritical, controllers.ThreatLeft, ranges, angles, nil,
	)
	if !ok {
		t.Fatal("ok = false, want true")
	}
	if maneuver.Speed <= 0 {
		t.Errorf("Speed = %v, want > 0 (creeping forward, not reversing)", maneuver.Speed)
	}
	if maneuver.Steering >= 0 {
		t.Errorf("Steering = %v, want < 0 (forward frame: negative swings the nose right)", maneuver.Steering)
	}
}

// TestComputeEscapeManeuver_LeftThreatTouchingReversesAndFlipsSign ports
// test_left_threat_touching_reverses_and_flips_sign: between
// self_detection_threshold_m (0.08, filtered as chassis reflection below
// this) and contact_dist (0.10, "already touching" at or below this).
func TestComputeEscapeManeuver_LeftThreatTouchingReversesAndFlipsSign(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, math.Pi/2)
	setSector(ranges, i, 6, 0.09)

	maneuver, ok := controller.ComputeEscapeManeuver(
		controllers.RiskCritical, controllers.ThreatLeft, ranges, angles, nil,
	)
	if !ok {
		t.Fatal("ok = false, want true")
	}
	if maneuver.Speed >= 0 {
		t.Errorf("Speed = %v, want < 0 (reversing)", maneuver.Speed)
	}
	if maneuver.Steering <= 0 {
		t.Errorf(
			"Steering = %v, want > 0 (reverse frame: positive swings the nose right, still away)", maneuver.Steering,
		)
	}
}

// TestComputeEscapeManeuver_RightThreatCreepingSteersLeft ports
// test_right_threat_creeping_steers_left.
func TestComputeEscapeManeuver_RightThreatCreepingSteersLeft(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, -math.Pi/2)
	setSector(ranges, i, 6, 0.20)

	maneuver, ok := controller.ComputeEscapeManeuver(
		controllers.RiskCritical, controllers.ThreatRight, ranges, angles, nil,
	)
	if !ok {
		t.Fatal("ok = false, want true")
	}
	if maneuver.Speed <= 0 {
		t.Errorf("Speed = %v, want > 0", maneuver.Speed)
	}
	if maneuver.Steering <= 0 {
		t.Errorf("Steering = %v, want > 0 (forward frame: positive swings the nose left)", maneuver.Steering)
	}
}

// TestComputeEscapeManeuver_RightThreatTouchingReversesAndFlipsSign ports
// test_right_threat_touching_reverses_and_flips_sign.
func TestComputeEscapeManeuver_RightThreatTouchingReversesAndFlipsSign(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, -math.Pi/2)
	setSector(ranges, i, 6, 0.09)

	maneuver, ok := controller.ComputeEscapeManeuver(
		controllers.RiskCritical, controllers.ThreatRight, ranges, angles, nil,
	)
	if !ok {
		t.Fatal("ok = false, want true")
	}
	if maneuver.Speed >= 0 {
		t.Errorf("Speed = %v, want < 0", maneuver.Speed)
	}
	if maneuver.Steering >= 0 {
		t.Errorf(
			"Steering = %v, want < 0 (reverse frame: negative swings the nose left, still away)", maneuver.Steering,
		)
	}
}

// -- TestSideCorrectionReversesWhenTouchingForward: a robot pinned at an
// angle into a corner can be touching in front while its side sector still
// reads clear -- SIDE_CORRECTION's "already touching" check must also look
// forward.

// TestComputeEscapeManeuver_LeftThreatWithForwardContactReverses ports
// test_left_threat_with_forward_contact_reverses.
func TestComputeEscapeManeuver_LeftThreatWithForwardContactReverses(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	leftI := angleToIndex(angles, math.Pi/2)
	setSector(ranges, leftI, 6, 0.20) // left sector alone reads clear (not touching)
	frontI := angleToIndex(angles, 0.0)
	setSector(ranges, frontI, 4, 0.08) // but forward is already at contact_dist

	maneuver, ok := controller.ComputeEscapeManeuver(
		controllers.RiskCritical, controllers.ThreatLeft, ranges, angles, nil,
	)
	if !ok {
		t.Fatal("ok = false, want true")
	}
	if maneuver.Speed >= 0 {
		t.Errorf("Speed = %v, want < 0 (reverses instead of creeping into the wall)", maneuver.Speed)
	}
	if maneuver.Steering <= 0 {
		t.Errorf("Steering = %v, want > 0 (reverse frame: still swings away)", maneuver.Steering)
	}
}

// TestComputeEscapeManeuver_RightThreatWithForwardContactReverses ports
// test_right_threat_with_forward_contact_reverses.
func TestComputeEscapeManeuver_RightThreatWithForwardContactReverses(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	rightI := angleToIndex(angles, -math.Pi/2)
	setSector(ranges, rightI, 6, 0.20)
	frontI := angleToIndex(angles, 0.0)
	setSector(ranges, frontI, 4, 0.08)

	maneuver, ok := controller.ComputeEscapeManeuver(
		controllers.RiskCritical, controllers.ThreatRight, ranges, angles, nil,
	)
	if !ok {
		t.Fatal("ok = false, want true")
	}
	if maneuver.Speed >= 0 {
		t.Errorf("Speed = %v, want < 0", maneuver.Speed)
	}
	if maneuver.Steering >= 0 {
		t.Errorf("Steering = %v, want < 0 (reverse frame: still swings away)", maneuver.Steering)
	}
}

// TestComputeEscapeManeuver_LeftThreatWithForwardAndSideClearStillCreeps
// ports test_left_threat_with_forward_and_side_clear_still_creeps: a
// regression guard -- forward clear + side clear must still creep, not
// reverse on every tick regardless of actual clearance.
func TestComputeEscapeManeuver_LeftThreatWithForwardAndSideClearStillCreeps(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	leftI := angleToIndex(angles, math.Pi/2)
	setSector(ranges, leftI, 6, 0.20)

	maneuver, ok := controller.ComputeEscapeManeuver(
		controllers.RiskCritical, controllers.ThreatLeft, ranges, angles, nil,
	)
	if !ok {
		t.Fatal("ok = false, want true")
	}
	if maneuver.Speed <= 0 {
		t.Errorf("Speed = %v, want > 0 (still creeping)", maneuver.Speed)
	}
}

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

	if got := controller.ComputeRearClearance(ranges, angles); got <= 5.0 {
		t.Errorf("ComputeRearClearance() = %v, want > 5.0", got)
	}
}

// TestComputeRearClearance_RealWallBeyondSelfRadiusStillMasked ports
// test_rear_real_wall_beyond_self_radius_still_detected: the rear mount is
// in the blind wedge on the current chassis, so a real wall just beyond the
// self-detection radius is STILL masked and reads as the no-data fallback,
// per the Python test's own comment about the current (post-2026-08-22)
// mount geometry.
func TestComputeRearClearance_RealWallBeyondSelfRadiusStillMasked(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, math.Pi)
	setSector(ranges, i, 4, 0.15) // beyond self-detection radius: a real return

	if got := controller.ComputeRearClearance(ranges, angles); got != controller.Geometry.NoDataRangeM {
		t.Errorf("ComputeRearClearance() = %v, want %v", got, controller.Geometry.NoDataRangeM)
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

	if got := controller.DetectThreatDirection(ranges, angles); got != controllers.ThreatNone {
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

	if got := controller.DetectThreatDirection(ranges, angles); got != controllers.ThreatFront {
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

	clearance := controller.ComputeForwardClearance(ranges, angles)
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

	if got := controller.ComputeForwardClearance(ranges, angles); got != 10.0 {
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

	if got := controller.AssessRisk(ranges, angles); got != controllers.RiskSafe {
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

	if got := controller.AssessRisk(ranges, angles); got != controllers.RiskCritical {
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

	if got := controller.AssessRisk(ranges, angles); got != controllers.RiskObstacle {
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

	if got := controller.AssessRisk(ranges, angles); got != controllers.RiskCritical {
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
	if got := controller.AssessRisk(nil, nil); got != controllers.RiskSafe {
		t.Errorf("AssessRisk(nil, nil) = %v, want %v", got, controllers.RiskSafe)
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
	setSector(ranges, i, 4, 0.02)                   // self-collision range, would otherwise scream "threat"

	if got := controller.DetectThreatDirection(ranges, angles); got != controllers.ThreatNone {
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

	if got := controller.DetectThreatDirection(ranges, angles); got != controllers.ThreatNone {
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
	setSector(ranges, i, 4, 0.15)                   // above self_detection_threshold_m (0.08): a real return

	if got := controller.DetectThreatDirection(ranges, angles); got != controllers.ThreatRight {
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

	if got := controller.ComputeRearClearance(ranges, angles); got <= minRearClearance {
		t.Errorf("ComputeRearClearance() = %v, want > %v", got, minRearClearance)
	}
}
