package controllers_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// TestSectorToModel_SectorFullyInsideWedgeReportsWedgeMasked ports
// TestBlindWedgeMasking.test_sector_fully_inside_wedge_reports_wedge_masked:
// a sector whose only candidate rays sit inside a blind wedge (not "no rays
// swept this bearing at all") must say so via WedgeMasked, so a caller can
// tell "cannot see" apart from "genuinely nothing out there".
func TestSectorToModel_SectorFullyInsideWedgeReportsWedgeMasked(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	const wedgeBearingRad = -140.0 * math.Pi / 180.0
	i := angleToIndex(angles, wedgeBearingRad)
	setSector(ranges, i, 2, 0.02) // only rays available are inside the left wedge

	sr := controllers.SectorToModel(
		ranges,
		angles,
		wedgeBearingRad,
		5.0*math.Pi/180.0,
		true,
		controller.Geometry,
	)

	if sr.ValidCount != 0 {
		t.Errorf("ValidCount = %d, want 0", sr.ValidCount)
	}
	if !sr.WedgeMasked {
		t.Error("WedgeMasked = false, want true (every candidate ray was wedge-excluded)")
	}
}

// TestSectorToModel_SectorWithNoRaysAtAllIsNotWedgeMasked ports
// TestBlindWedgeMasking.test_sector_with_no_rays_at_all_is_not_reported_as_wedge_masked:
// a sector with genuinely no data anywhere (every ray a no-return) must NOT
// claim wedge masking -- that would misattribute a sensor dropout to mount
// geometry.
func TestSectorToModel_SectorWithNoRaysAtAllIsNotWedgeMasked(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(math.Inf(1))

	sr := controllers.SectorToModel(
		ranges,
		angles,
		0.0,
		controller.FrontHalfFovRad,
		true,
		controller.Geometry,
	)

	if sr.ValidCount != 0 {
		t.Errorf("ValidCount = %d, want 0", sr.ValidCount)
	}
	if sr.WedgeMasked {
		t.Error("WedgeMasked = true, want false (genuinely no data, not a wedge artifact)")
	}
}

// TestMaskMappedObstacles_ReturnOnAMappedPositionIsMasked ports
// TestMaskMappedObstacles.test_return_on_a_mapped_position_is_masked.
func TestMaskMappedObstacles_ReturnOnAMappedPositionIsMasked(t *testing.T) {
	t.Parallel()

	const maskRadiusM = 0.12
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, 0.0)
	setSector(ranges, i, forwardSectorIndices, 0.08) // inside contact_dist

	mapped := []controllers.MappedObstacle{
		{Position: trackmodel.Waypoint{X: 0.08, Y: 0.0}, Corridor: trackmodel.South},
	}
	masked := controllers.MaskMappedObstacles(
		ranges, angles, trackmodel.Pose{}, mapped, maskRadiusM, cornerMinM, cornerMaxM,
	)

	if !math.IsInf(masked[i], 1) {
		t.Errorf(
			"masked[%d] = %v, want +Inf (a return landing on a mapped sign must be withheld)",
			i,
			masked[i],
		)
	}
}

// TestMaskMappedObstacles_UnmappedReturnAtTheSameRangeIsUntouched ports
// TestMaskMappedObstacles.test_unmapped_return_at_the_same_range_is_untouched:
// the guard is removed for the mapped obstacle only, not for the range.
func TestMaskMappedObstacles_UnmappedReturnAtTheSameRangeIsUntouched(t *testing.T) {
	t.Parallel()

	const maskRadiusM = 0.12
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, 0.0)
	setSector(ranges, i, forwardSectorIndices, 0.08)

	// Mapped sign is off to the side; the forward return belongs to nothing.
	mapped := []controllers.MappedObstacle{
		{Position: trackmodel.Waypoint{X: 0.0, Y: 0.9}, Corridor: trackmodel.South},
	}
	masked := controllers.MaskMappedObstacles(
		ranges, angles, trackmodel.Pose{}, mapped, maskRadiusM, cornerMinM, cornerMaxM,
	)

	if math.Abs(masked[i]-0.08) > 1e-9 {
		t.Errorf("masked[%d] = %v, want 0.08 (unrelated to the mapped obstacle)", i, masked[i])
	}
}

// TestMaskMappedObstacles_MappedPositionsAreWorldFrameNotRobotFrame ports
// TestMaskMappedObstacles.test_mapped_positions_are_world_frame_not_robot_frame:
// ray endpoints are placed using the robot's pose, so a robot-frame
// (mis)interpretation would silently mask the wrong bearing off the yaw=0
// case every other test in this file uses.
func TestMaskMappedObstacles_MappedPositionsAreWorldFrameNotRobotFrame(t *testing.T) {
	t.Parallel()

	const maskRadiusM = 0.12
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, 0.0)
	setSector(ranges, i, forwardSectorIndices, 0.08)
	// Robot at (1.0, 2.0) facing north: the forward return lands at
	// (1.0, 2.08), NOT at (0.08, 0).
	pose := trackmodel.Pose{X: 1.0, Y: 2.0, Yaw: math.Pi / 2}

	worldMapped := []controllers.MappedObstacle{
		{Position: trackmodel.Waypoint{X: 1.0, Y: 2.08}, Corridor: trackmodel.North},
	}
	maskedWorld := controllers.MaskMappedObstacles(
		ranges, angles, pose, worldMapped, maskRadiusM, cornerMinM, cornerMaxM,
	)

	bodyMapped := []controllers.MappedObstacle{
		{Position: trackmodel.Waypoint{X: 0.08, Y: 0.0}, Corridor: trackmodel.North},
	}
	maskedBody := controllers.MaskMappedObstacles(
		ranges, angles, pose, bodyMapped, maskRadiusM, cornerMinM, cornerMaxM,
	)

	if !math.IsInf(maskedWorld[i], 1) {
		t.Errorf("maskedWorld[%d] = %v, want +Inf", i, maskedWorld[i])
	}
	if math.Abs(maskedBody[i]-0.08) > 1e-9 {
		t.Errorf(
			"maskedBody[%d] = %v, want 0.08 (a robot-frame position must not match)",
			i,
			maskedBody[i],
		)
	}
}

// TestMaskMappedObstacles_CrossCorridorCoincidenceIsNotMasked ports
// TestMaskMappedObstacles.test_cross_corridor_coincidence_is_not_masked: a
// routed sign's raw XY landing near an unrelated ray endpoint is not enough
// to mask it -- the robot must also currently be in that sign's own
// corridor, guarding against a wrong-but-consistent believed pose
// reprojecting an unmapped obstacle onto an already-routed sign's
// coordinates by coincidence.
func TestMaskMappedObstacles_CrossCorridorCoincidenceIsNotMasked(t *testing.T) {
	t.Parallel()

	const maskRadiusM = 0.12
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, 0.0)
	setSector(ranges, i, forwardSectorIndices, 0.08)
	// Robot at (1.5, 0.5) is in the SOUTH corridor; the forward return lands
	// at (1.56, 0.5), also SOUTH.
	pose := trackmodel.Pose{X: 1.5, Y: 0.5, Yaw: 0.0}

	// A "routed sign" whose raw XY coincides with that ray endpoint, but
	// which the router itself placed in the NORTH corridor.
	mapped := []controllers.MappedObstacle{
		{Position: trackmodel.Waypoint{X: 1.56, Y: 0.5}, Corridor: trackmodel.North},
	}
	masked := controllers.MaskMappedObstacles(
		ranges,
		angles,
		pose,
		mapped,
		maskRadiusM,
		cornerMinM,
		cornerMaxM,
	)

	if math.Abs(masked[i]-0.08) > 1e-9 {
		t.Errorf(
			"masked[%d] = %v, want 0.08 (a same-XY coincidence in a different corridor must not mask)",
			i,
			masked[i],
		)
	}
}

// TestMaskMappedObstacles_ZeroRadiusIsANoOp ports half of
// TestMaskMappedObstacles.test_zero_radius_and_empty_map_are_no_ops:
// radius_m=0 is the documented off-switch, so masking must reproduce the raw
// scan exactly, not approximately.
func TestMaskMappedObstacles_ZeroRadiusIsANoOp(t *testing.T) {
	t.Parallel()

	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	ranges[angleToIndex(angles, 0.0)] = 0.08

	mapped := []controllers.MappedObstacle{
		{Position: trackmodel.Waypoint{X: 0.08, Y: 0.0}, Corridor: trackmodel.South},
	}
	masked := controllers.MaskMappedObstacles(
		ranges,
		angles,
		trackmodel.Pose{},
		mapped,
		0.0,
		cornerMinM,
		cornerMaxM,
	)

	for k := range ranges {
		if masked[k] != ranges[k] {
			t.Fatalf(
				"masked[%d] = %v, want %v (radius_m=0 must be a no-op)",
				k,
				masked[k],
				ranges[k],
			)
		}
	}
}

// TestMaskMappedObstacles_EmptyMapIsANoOp ports the other half of
// test_zero_radius_and_empty_map_are_no_ops: an empty mapped-obstacle list
// disables the split just as completely as a zero radius.
func TestMaskMappedObstacles_EmptyMapIsANoOp(t *testing.T) {
	t.Parallel()

	const maskRadiusM = 0.12
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	ranges[angleToIndex(angles, 0.0)] = 0.08

	masked := controllers.MaskMappedObstacles(
		ranges, angles, trackmodel.Pose{}, nil, maskRadiusM, cornerMinM, cornerMaxM,
	)

	for k := range ranges {
		if masked[k] != ranges[k] {
			t.Fatalf(
				"masked[%d] = %v, want %v (an empty map must be a no-op)",
				k,
				masked[k],
				ranges[k],
			)
		}
	}
}

// TestMaskMappedObstacles_NoReturnRaysStayInfiniteAndNeverBecomeNaN ports
// TestMaskMappedObstacles.test_no_return_rays_stay_infinite_and_never_become_nan:
// inf*cos(theta) is +-inf and inf-inf is NaN, so an unguarded distance test
// would quietly turn every no-return ray into NaN -- which compares false
// everywhere and would corrupt downstream min/mean instead of failing
// loudly.
func TestMaskMappedObstacles_NoReturnRaysStayInfiniteAndNeverBecomeNaN(t *testing.T) {
	t.Parallel()

	const maskRadiusM = 0.12
	angles := anglesFullRotation()
	ranges := newScan(math.Inf(1))

	mapped := []controllers.MappedObstacle{
		{Position: trackmodel.Waypoint{X: 0.08, Y: 0.0}, Corridor: trackmodel.South},
	}
	masked := controllers.MaskMappedObstacles(
		ranges, angles, trackmodel.Pose{}, mapped, maskRadiusM, cornerMinM, cornerMaxM,
	)

	for k, v := range masked {
		if math.IsNaN(v) {
			t.Fatalf("masked[%d] = NaN, want +Inf (no-return rays must never become NaN)", k)
		}
		if !math.IsInf(v, 1) {
			t.Fatalf("masked[%d] = %v, want +Inf", k, v)
		}
	}
}

// TestMaskMappedObstacles_InputScanIsNotMutated ports
// TestMaskMappedObstacles.test_input_scan_is_not_mutated: the raw scan still
// governs speed and the rear gate, so masking must return a copy rather than
// editing the caller's slice in place.
func TestMaskMappedObstacles_InputScanIsNotMutated(t *testing.T) {
	t.Parallel()

	const maskRadiusM = 0.12
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, 0.0)
	ranges[i] = 0.08

	mapped := []controllers.MappedObstacle{
		{Position: trackmodel.Waypoint{X: 0.08, Y: 0.0}, Corridor: trackmodel.South},
	}
	controllers.MaskMappedObstacles(
		ranges,
		angles,
		trackmodel.Pose{},
		mapped,
		maskRadiusM,
		cornerMinM,
		cornerMaxM,
	)

	if math.Abs(ranges[i]-0.08) > 1e-9 {
		t.Errorf("ranges[%d] = %v after masking, want unchanged 0.08", i, ranges[i])
	}
}
