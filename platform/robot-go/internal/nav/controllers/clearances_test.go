package controllers_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
)

// TestLidarClearances_AnyBlockedTriggersBelowThreshold ports
// TestLidarClearancesModel.test_any_blocked_triggers_below_threshold.
func TestLidarClearances_AnyBlockedTriggersBelowThreshold(t *testing.T) {
	t.Parallel()

	c := controllers.LidarClearances{FrontM: 0.05, LeftM: 2.0, RightM: 2.0, BackM: 2.0}
	if !c.AnyBlocked(0.1) {
		t.Error("AnyBlocked(0.1) = false, want true (front is closer than the threshold)")
	}
	if c.AnyBlocked(0.02) {
		t.Error("AnyBlocked(0.02) = true, want false (nothing is closer than the threshold)")
	}
}

// TestLidarClearances_MostConstrainedSidePicksSmallest ports
// TestLidarClearancesModel.test_most_constrained_side_picks_smallest.
func TestLidarClearances_MostConstrainedSidePicksSmallest(t *testing.T) {
	t.Parallel()

	c := controllers.LidarClearances{FrontM: 2.0, LeftM: 0.1, RightM: 2.0, BackM: 2.0}
	if got := c.MostConstrainedSide(); got != controllers.ThreatLeft {
		t.Errorf("MostConstrainedSide() = %v, want %v", got, controllers.ThreatLeft)
	}

	c2 := controllers.LidarClearances{FrontM: 2.0, LeftM: 2.0, RightM: 2.0, BackM: 0.0}
	if got := c2.MostConstrainedSide(); got != controllers.ThreatBack {
		t.Errorf("MostConstrainedSide() = %v, want %v", got, controllers.ThreatBack)
	}
}

// TestClearancesFromScan_EmptyScanYieldsZeroedClearances ports
// TestClearancesFromScan.test_empty_scan_yields_zeroed_clearances.
func TestClearancesFromScan_EmptyScanYieldsZeroedClearances(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	scan := controllers.LidarScan{}
	c := controllers.ClearancesFromScan(
		scan,
		controller,
		controller.FrontHalfFovRad,
		controllers.AggregateMean,
	)

	want := controllers.LidarClearances{}
	if c != want {
		t.Errorf("ClearancesFromScan(empty) = %+v, want %+v", c, want)
	}
}

// TestClearancesFromScan_FrontWallReportsSmallFrontClearance ports
// TestClearancesFromScan.test_front_wall_reports_small_front_clearance.
func TestClearancesFromScan_FrontWallReportsSmallFrontClearance(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, 0.0)
	setSector(ranges, i, forwardSectorIndices, lidarCloseThreat)

	scan := controllers.LidarScan{RangesM: ranges, AnglesRad: angles}
	c := controllers.ClearancesFromScan(
		scan,
		controller,
		controller.FrontHalfFovRad,
		controllers.AggregateMean,
	)

	if got := c.MostConstrainedSide(); got != controllers.ThreatFront {
		t.Errorf("MostConstrainedSide() = %v, want %v", got, controllers.ThreatFront)
	}
	if c.FrontM >= c.LeftM {
		t.Errorf("FrontM (%v) >= LeftM (%v), want front strictly smaller", c.FrontM, c.LeftM)
	}
	if c.FrontM >= c.RightM {
		t.Errorf("FrontM (%v) >= RightM (%v), want front strictly smaller", c.FrontM, c.RightM)
	}
}

// TestClearancesFromScan_MinAggregationMatchesThreatSectors ports
// TestClearancesFromScan.test_min_aggregation_matches_threat_sectors: MIN
// aggregation over the threat FOV reproduces DetectThreatDirection's own
// sectors exactly.
func TestClearancesFromScan_MinAggregationMatchesThreatSectors(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	i := angleToIndex(angles, math.Pi/2) // +pi/2 = left
	setSector(ranges, i, forwardSectorIndices, lidarCloseThreat)

	scan := controllers.LidarScan{RangesM: ranges, AnglesRad: angles}
	c := controllers.ClearancesFromScan(
		scan,
		controller,
		controller.ThreatHalfFovRad,
		controllers.AggregateMin,
	)

	if got := c.MostConstrainedSide(); got != controllers.ThreatLeft {
		t.Errorf("MostConstrainedSide() = %v, want %v", got, controllers.ThreatLeft)
	}
	if got := controller.DetectThreatDirection(ranges, angles); got != controllers.ThreatLeft {
		t.Errorf("DetectThreatDirection() = %v, want %v", got, controllers.ThreatLeft)
	}
}

// TestThreatDirectionFrom_GatesOnNoDetectionRange ports
// TestClearancesFromScan.test_threat_direction_gates_on_no_detection_range.
func TestThreatDirectionFrom_GatesOnNoDetectionRange(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()

	// Open scan: nothing within the no-detection range -> NONE.
	openScan := controllers.LidarScan{RangesM: newScan(lidarDefaultFar), AnglesRad: angles}
	openClearances := controllers.ClearancesFromScan(
		openScan, controller, controller.ThreatHalfFovRad, controllers.AggregateMin,
	)
	got := controllers.ThreatDirectionFrom(openClearances, controller.ThreatNoDetectionRangeM)
	if got != controllers.ThreatNone {
		t.Errorf("ThreatDirectionFrom(open) = %v, want %v", got, controllers.ThreatNone)
	}

	// A close front wall enters the gate and names the constrained side.
	ranges := newScan(lidarDefaultFar)
	ranges[angleToIndex(angles, 0.0)] = lidarCloseThreat
	closeScan := controllers.LidarScan{RangesM: ranges, AnglesRad: angles}
	closeClearances := controllers.ClearancesFromScan(
		closeScan, controller, controller.ThreatHalfFovRad, controllers.AggregateMin,
	)
	closeThreatDir := controllers.ThreatDirectionFrom(
		closeClearances,
		controller.ThreatNoDetectionRangeM,
	)
	if closeThreatDir != controllers.ThreatFront {
		t.Errorf(
			"ThreatDirectionFrom(close front) = %v, want %v",
			closeThreatDir,
			controllers.ThreatFront,
		)
	}
}

// TestParkingGate_ReturnsForwardAndSweepFromOneCall ports
// TestParkingGate.test_returns_forward_and_sweep_from_one_call: the forward
// cone and the full 360deg sweep are two different LIDAR queries, and a
// closer wall outside the forward cone must only show up in the sweep.
func TestParkingGate_ReturnsForwardAndSweepFromOneCall(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	angles := anglesFullRotation()
	ranges := newScan(lidarDefaultFar)
	// Close wall dead ahead: catches the forward cone.
	i := angleToIndex(angles, 0.0)
	setSector(ranges, i, forwardSectorIndices, lidarCloseThreat)
	// A closer wall at 45deg (between the front cone and the left sector):
	// only the full-sweep min should see it.
	j := angleToIndex(angles, math.Pi/4)
	setSector(ranges, j, forwardSectorIndices, 0.1)

	gate := controller.ParkingClearances(ranges, angles)

	wantForward := controller.ComputeForwardClearance(ranges, angles)
	if gate.ForwardM != wantForward {
		t.Errorf(
			"ForwardM = %v, want %v (matching ComputeForwardClearance)",
			gate.ForwardM,
			wantForward,
		)
	}
	if gate.ForwardM <= 0.1 {
		t.Errorf("ForwardM = %v, want > 0.1 (must not see the 45deg wall)", gate.ForwardM)
	}
	wantSweep := controller.ComputeMinClearance(ranges, angles, 0.0, math.Pi)
	if gate.SweepM != wantSweep {
		t.Errorf("SweepM = %v, want %v (matching ComputeMinClearance)", gate.SweepM, wantSweep)
	}
	if gate.SweepM >= gate.ForwardM {
		t.Errorf(
			"SweepM (%v) >= ForwardM (%v), want the sweep to see the closer 45deg wall",
			gate.SweepM,
			gate.ForwardM,
		)
	}
}

// TestParkingGate_EmptyScanYieldsNoData ports
// TestParkingGate.test_empty_scan_yields_no_data.
func TestParkingGate_EmptyScanYieldsNoData(t *testing.T) {
	t.Parallel()

	controller := newDefaultCollisionAvoidanceController()
	gate := controller.ParkingClearances(nil, nil)

	if gate.ForwardM != controller.Geometry.NoDataRangeM {
		t.Errorf("ForwardM = %v, want %v", gate.ForwardM, controller.Geometry.NoDataRangeM)
	}
	if gate.SweepM != controller.Geometry.NoDataRangeM {
		t.Errorf("SweepM = %v, want %v", gate.SweepM, controller.Geometry.NoDataRangeM)
	}
}
