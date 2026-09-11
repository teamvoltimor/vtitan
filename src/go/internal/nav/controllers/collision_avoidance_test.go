package controllers_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
)

// -- TestThreatDirection: ports test_collision_avoidance_controller.py's
// TestThreatDirection class. A wall behind the robot must report "back",
// never "front".

// TestDetectThreatDirection_WallBehindReportsBackNotFront ports
// test_wall_behind_reports_back_not_front.
func TestDetectThreatDirection_WallBehindReportsBackNotFront(t *testing.T) {
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

	// Between 2026-08-22 and 2026-08-31 the rear was fully masked by the
	// blind wedges, so a wall behind was not seen at all and this asserted
	// ThreatNone. The wedges were re-measured on the current mount
	// (-155..-120 / 120..160, a ~40 deg slot at +/-160..180), so the rear
	// is visible again and the original behaviour is back.
	if got := controller.DetectThreatDirection(scanObj(ranges, angles)); got != controllers.ThreatBack {
		t.Errorf("DetectThreatDirection() = %v, want %v", got, controllers.ThreatBack)
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

	got := controller.DetectThreatDirection(scanObj(ranges, nil))
	if got != controllers.ThreatBack {
		t.Errorf("DetectThreatDirection(nil angles) = %v, want %v", got, controllers.ThreatBack)
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

	if got := controller.DetectThreatDirection(scanObj(ranges, angles)); got != controllers.ThreatFront {
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

	if got := controller.DetectThreatDirection(scanObj(ranges, angles)); got != controllers.ThreatLeft {
		t.Errorf("DetectThreatDirection() = %v, want %v", got, controllers.ThreatLeft)
	}
}

// TestDetectThreatDirection_AllClearReportsNone ports
// test_all_clear_reports_none.
func TestDetectThreatDirection_AllClearReportsNone(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()

	if got := controller.DetectThreatDirection(
		scanObj(newScan(lidarDefaultFar), angles),
	); got != controllers.ThreatNone {
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

	if got := controller.ComputeForwardClearance(scanObj(ranges, angles)); got <= minForwardClearance {
		t.Errorf("ComputeForwardClearance() = %v, want > %v", got, minForwardClearance)
	}
}

// TestComputeRearClearance_SeesRearWall ports
// test_rear_clearance_sees_rear_wall: the re-measured wedges (2026-08-31)
// leave a ~40 deg readable slot at the rear (+/-160..180), so rear rays are
// visible and rear clearance reads the actual wall distance rather than the
// no-data fallback -- unlike the 2026-08-22..2026-08-31 window when the
// mount fully masked the rear.
func TestComputeRearClearance_SeesRearWall(t *testing.T) {
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

	if got := controller.ComputeRearClearance(scanObj(ranges, angles)); math.Abs(got-lidarCloseThreat) > 1e-9 {
		t.Errorf("ComputeRearClearance() = %v, want %v", got, lidarCloseThreat)
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

	if got := controller.ComputeRearClearance(scanObj(ranges, angles)); got <= minRearClearance {
		t.Errorf("ComputeRearClearance() = %v, want > %v", got, minRearClearance)
	}
}

// -- TestRearSectorVisibility: a reverse gate must be able to tell "nothing
// behind" from "cannot see".

// TestRearSector_NormalScanIsMeasured ports test_normal_scan_is_measured:
// the 2026-08-31 wedge re-measurement restored a ~40 deg readable slot at
// the rear (+/-160..180), so a normal scan's rear sector reports measured
// again -- unlike the 2026-08-22..2026-08-31 window when the mount fully
// masked the rear ("cannot see" rather than "nothing behind").
func TestRearSector_NormalScanIsMeasured(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	if !controller.RearSector(scanObj(newScan(lidarDefaultFar), anglesFullRotation())).Measured() {
		t.Error("RearSector(...).Measured() = false, want true (rear slot is readable again)")
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

	if got := controller.ComputeRearClearance(scanObj(ranges, angles)); got != controller.Geometry.NoDataRangeM {
		t.Errorf("ComputeRearClearance() = %v, want %v", got, controller.Geometry.NoDataRangeM)
	}
	if controller.RearSector(scanObj(ranges, angles)).Measured() {
		t.Error("RearSector(...).Measured() = true, want false")
	}
}

// TestRearSector_EmptyScanIsNotMeasured ports test_empty_scan_is_not_measured.
func TestRearSector_EmptyScanIsNotMeasured(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	if controller.RearSector(controllers.LidarScan{}).Measured() {
		t.Error("RearSector(controllers.LidarScan{}).Measured() = true, want false")
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
	_, ok := controller.ComputeEscapeManeuver(
		controllers.RiskCritical,
		controllers.ThreatBack,
		controllers.LidarScan{},
		nil,
	)
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
		controllers.RiskCritical, controllers.ThreatFront, scanObj(ranges, angles), nil,
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
		controllers.RiskCritical, controllers.ThreatFront, scanObj(ranges, angles), nil,
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
	maneuver, ok := controller.ComputeEscapeManeuver(
		controllers.RiskCritical,
		controllers.ThreatFront,
		controllers.LidarScan{},
		nil,
	)
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
		controllers.RiskCritical, controllers.ThreatLeft, scanObj(ranges, angles), nil,
	)
	if !ok {
		t.Fatal("ok = false, want true")
	}
	if maneuver.Speed <= 0 {
		t.Errorf("Speed = %v, want > 0 (creeping forward, not reversing)", maneuver.Speed)
	}
	if maneuver.Steering >= 0 {
		t.Errorf(
			"Steering = %v, want < 0 (forward frame: negative swings the nose right)",
			maneuver.Steering,
		)
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
		controllers.RiskCritical, controllers.ThreatLeft, scanObj(ranges, angles), nil,
	)
	if !ok {
		t.Fatal("ok = false, want true")
	}
	if maneuver.Speed >= 0 {
		t.Errorf("Speed = %v, want < 0 (reversing)", maneuver.Speed)
	}
	if maneuver.Steering <= 0 {
		t.Errorf(
			"Steering = %v, want > 0 (reverse frame: positive swings the nose right, still away)",
			maneuver.Steering,
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
		controllers.RiskCritical, controllers.ThreatRight, scanObj(ranges, angles), nil,
	)
	if !ok {
		t.Fatal("ok = false, want true")
	}
	if maneuver.Speed <= 0 {
		t.Errorf("Speed = %v, want > 0", maneuver.Speed)
	}
	if maneuver.Steering <= 0 {
		t.Errorf(
			"Steering = %v, want > 0 (forward frame: positive swings the nose left)",
			maneuver.Steering,
		)
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
		controllers.RiskCritical, controllers.ThreatRight, scanObj(ranges, angles), nil,
	)
	if !ok {
		t.Fatal("ok = false, want true")
	}
	if maneuver.Speed >= 0 {
		t.Errorf("Speed = %v, want < 0", maneuver.Speed)
	}
	if maneuver.Steering >= 0 {
		t.Errorf(
			"Steering = %v, want < 0 (reverse frame: negative swings the nose left, still away)",
			maneuver.Steering,
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
		controllers.RiskCritical, controllers.ThreatLeft, scanObj(ranges, angles), nil,
	)
	if !ok {
		t.Fatal("ok = false, want true")
	}
	if maneuver.Speed >= 0 {
		t.Errorf(
			"Speed = %v, want < 0 (reverses instead of creeping into the wall)",
			maneuver.Speed,
		)
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
		controllers.RiskCritical, controllers.ThreatRight, scanObj(ranges, angles), nil,
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
		controllers.RiskCritical, controllers.ThreatLeft, scanObj(ranges, angles), nil,
	)
	if !ok {
		t.Fatal("ok = false, want true")
	}
	if maneuver.Speed <= 0 {
		t.Errorf("Speed = %v, want > 0 (still creeping)", maneuver.Speed)
	}
}
