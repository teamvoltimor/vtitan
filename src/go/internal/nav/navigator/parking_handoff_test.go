package navigator_test

import (
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navigator"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/parking"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// TestParkingHandoff_EngagesNearStagingAndDrivesTheManeuver exercises
// handleFinish end-to-end (matching _handle_finish): a ParkController is
// attached, the single-waypoint lap completes, the handoff engages once
// close enough to the staging point in the right corridor, and
// ParkController.Update actually drives -- not a hold.
func TestParkingHandoff_EngagesNearStagingAndDrivesTheManeuver(t *testing.T) {
	t.Parallel()

	// A SOUTH lot (fins at x=1.00/1.45, y=0.10, matching the fixture in
	// internal/nav/parking's own tests): BuildZone's staging point sits at
	// (gap_cx, y_max+clearance) = (1.225, 0.20+0.45) = (1.225, 0.65).
	lot := parking.ParkingLot{
		Block1: parking.BlockPosition{X: 1.00, Y: 0.10},
		Block2: parking.BlockPosition{X: 1.45, Y: 0.10},
	}
	// The pursuit ships DEFERRED, so this test opts in explicitly: it is
	// about the handoff mechanism, which only runs when the bay is pursued.
	// See TestParkingHandoff_DeferredHoldsInsteadOfPursuing for the default.
	cfg := parking.DefaultConfig()
	cfg.AttemptAfterFinalLap = true
	pc := parking.NewParkController(lot, trackmodel.South, trackmodel.Counterclockwise, cfg, 0, 0)

	// A single-waypoint "lap" at (1.225, 0.5): CornerForPosition classifies
	// it South (default CornerMinM/MaxM 1.0/2.0), matching pc.Section(), and
	// it sits 0.15m from staging -- inside the default 0.45m ParkEngageDistM.
	const wpX, wpY = 1.225, 0.5
	nav, gateway := newNavigator(t, func(p *navigator.Params) {
		p.Waypoints = []trackmodel.Waypoint{{X: wpX, Y: wpY}}
		p.NumLaps = 1
		p.ParkController = pc
	})

	gateway.setPose(wpX, wpY, 0.0)

	// Tick 1: reaches the sole waypoint (index 0 -> 1), not yet a lap.
	nav.Step()
	if nav.LapsCompleted() != 0 {
		t.Fatalf("after reaching the waypoint: LapsCompleted() = %d, want 0", nav.LapsCompleted())
	}

	// Tick 2: index (1) >= len(waypoints) (1) wraps the lap.
	nav.Step()
	if nav.LapsCompleted() != 1 {
		t.Fatalf("after the wrap tick: LapsCompleted() = %d, want 1", nav.LapsCompleted())
	}

	// Tick 3: lapsCompleted >= numLaps now holds, so handleFinish runs:
	// shouldEngageParking should fire (right corridor, within range), and
	// ParkController.Update should drive rather than hold.
	nav.Step()

	if nav.ParkController() == nil {
		t.Fatal("ParkController() = nil, want the attached controller")
	}
	debug := nav.DebugSnapshot()
	if debug.Phase != navigator.PhaseParking {
		t.Fatalf("Phase = %v, want PhaseParking (engaged and driving)", debug.Phase)
	}
	if debug.ParkPhase == nil || *debug.ParkPhase != parking.PhaseStage {
		t.Errorf("ParkPhase = %v, want PhaseStage (not yet reached the staging point)", debug.ParkPhase)
	}
	cmd, ok := gateway.lastDrive()
	if !ok {
		t.Fatal("lastDrive() ok = false, want a published command")
	}
	if cmd.SpeedMPS == 0.0 {
		t.Error("SpeedMPS = 0.0, want a nonzero drive command -- parking should be actively maneuvering, not holding")
	}
}

// TestParkingHandoff_NilParkControllerHoldsAtFinish confirms the Open
// Challenge (or any scenario with no parking lot) path is unaffected:
// _handle_finish's `pc is None` branch always holds immediately.
func TestParkingHandoff_NilParkControllerHoldsAtFinish(t *testing.T) {
	t.Parallel()

	const wpX, wpY = 1.5, 0.5
	nav, gateway := newNavigator(t, func(p *navigator.Params) {
		p.Waypoints = []trackmodel.Waypoint{{X: wpX, Y: wpY}}
		p.NumLaps = 1
	})
	gateway.setPose(wpX, wpY, 0.0)

	nav.Step() // reach the waypoint
	nav.Step() // wrap the lap
	nav.Step() // finish: pc == nil, holds immediately

	debug := nav.DebugSnapshot()
	if debug.Phase != navigator.PhaseFinishedHold {
		t.Fatalf("Phase = %v, want PhaseFinishedHold", debug.Phase)
	}
	cmd, ok := gateway.lastDrive()
	if !ok || cmd.SpeedMPS != 0.0 || cmd.SteeringNorm != 0.0 {
		t.Errorf("lastDrive() = %+v, ok=%v, want a zero DriveCommand", cmd, ok)
	}
}

// TestParkingHandoff_DeferredHoldsInsteadOfPursuing pins the SHIPPED default:
// a ParkController is attached and the robot is parked right next to the
// staging point, yet the round ends holding in the finish section rather than
// engaging. This is the same geometry as
// TestParkingHandoff_EngagesNearStagingAndDrivesTheManeuver -- the ONLY
// difference is AttemptAfterFinalLap -- so if the default ever flips back,
// one of the two fails rather than both quietly agreeing.
func TestParkingHandoff_DeferredHoldsInsteadOfPursuing(t *testing.T) {
	t.Parallel()

	lot := parking.ParkingLot{
		Block1: parking.BlockPosition{X: 1.00, Y: 0.10},
		Block2: parking.BlockPosition{X: 1.45, Y: 0.10},
	}
	cfg := parking.DefaultConfig()
	if cfg.AttemptAfterFinalLap {
		t.Fatal("DefaultConfig().AttemptAfterFinalLap = true, want the shipped false")
	}
	pc := parking.NewParkController(lot, trackmodel.South, trackmodel.Counterclockwise, cfg, 0, 0)

	const wpX, wpY = 1.225, 0.5
	nav, gateway := newNavigator(t, func(p *navigator.Params) {
		p.Waypoints = []trackmodel.Waypoint{{X: wpX, Y: wpY}}
		p.NumLaps = 1
		p.ParkController = pc
	})
	gateway.setPose(wpX, wpY, 0.0)

	nav.Step() // reach the waypoint
	nav.Step() // wrap the lap
	nav.Step() // finish: pursuit deferred, so hold

	debug := nav.DebugSnapshot()
	if debug.Phase != navigator.PhaseFinishedHold {
		t.Fatalf("Phase = %v, want PhaseFinishedHold (pursuit deferred)", debug.Phase)
	}
	cmd, ok := gateway.lastDrive()
	if !ok || cmd.SpeedMPS != 0.0 || cmd.SteeringNorm != 0.0 {
		t.Errorf("lastDrive() = %+v, ok=%v, want a zero DriveCommand", cmd, ok)
	}
}
