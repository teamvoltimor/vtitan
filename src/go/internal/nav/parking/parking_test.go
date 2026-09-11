// Package parking_test is the black-box test suite for package parking,
// mirroring platform/robot/tests/unit/test_parking.py.
package parking_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/parking"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// Block positions matching tests/test_constants.py's PARKING_*_BLOCK*
// fixtures: spacing 0.45 m (= spacing_factor 1.5 * chassis length 0.30),
// fins 0.10 m off the wall, far edge at 2.90 m.
const (
	parkA    = 1.00
	parkB    = 1.45
	parkNear = 0.10
	parkFar  = 2.90
)

var (
	southCfg = parking.ParkingLot{
		Block1: parking.BlockPosition{X: parkA, Y: parkNear},
		Block2: parking.BlockPosition{X: parkB, Y: parkNear},
	}
	northCfg = parking.ParkingLot{
		Block1: parking.BlockPosition{X: parkA, Y: parkFar},
		Block2: parking.BlockPosition{X: parkB, Y: parkFar},
	}
	eastCfg = parking.ParkingLot{
		Block1: parking.BlockPosition{X: parkFar, Y: parkA},
		Block2: parking.BlockPosition{X: parkFar, Y: parkB},
	}
	westCfg = parking.ParkingLot{
		Block1: parking.BlockPosition{X: parkNear, Y: parkA},
		Block2: parking.BlockPosition{X: parkNear, Y: parkB},
	}
)

// TestBuildZone_Baycenter checks the lot center and the fins' inner faces.
func TestBuildZone_Baycenter(t *testing.T) {
	t.Parallel()

	z := parking.BuildZone(
		southCfg.Block1,
		southCfg.Block2,
		trackmodel.South,
		trackmodel.Counterclockwise,
		parking.DefaultParkingLotSpecs,
		parking.DefaultTrackDimensions,
	)

	if math.Abs(z.GapCX-(parkA+parkB)/2) > 1e-9 {
		t.Errorf("GapCX = %v, want %v", z.GapCX, (parkA+parkB)/2)
	}
	if math.Abs(z.GapCY-0.10) > 1e-9 {
		t.Errorf("GapCY = %v, want 0.10 (half fin thickness off the wall)", z.GapCY)
	}
	// Fins are 0.02 m thick, so the inner faces are 0.01 m in from each block.
	if math.Abs(z.XMin-parkA-0.01) > 1e-9 || math.Abs(z.XMax-parkB+0.01) > 1e-9 {
		t.Errorf("x bounds = [%v, %v], want fins' inner faces [%v, %v]",
			z.XMin, z.XMax, parkA+0.01, parkB-0.01)
	}
	// Depth equals the marker length (0.20 m), spanning from the wall.
	if math.Abs(z.YMin-0.0) > 1e-9 || math.Abs(z.YMax-0.20) > 1e-9 {
		t.Errorf("y bounds = [%v, %v], want [0, 0.20]", z.YMin, z.YMax)
	}
}

func TestBuildZone_TargetYaw(t *testing.T) {
	t.Parallel()

	cases := []struct {
		section   trackmodel.Section
		lot       parking.ParkingLot
		direction trackmodel.Direction
		want      float64
	}{
		{trackmodel.South, southCfg, trackmodel.Clockwise, math.Pi},
		{trackmodel.South, southCfg, trackmodel.Counterclockwise, 0.0},
		{trackmodel.North, northCfg, trackmodel.Clockwise, 0.0},
		{trackmodel.North, northCfg, trackmodel.Counterclockwise, math.Pi},
		{trackmodel.East, eastCfg, trackmodel.Clockwise, math.Pi / 2},
		{trackmodel.East, eastCfg, trackmodel.Counterclockwise, -math.Pi / 2},
		{trackmodel.West, westCfg, trackmodel.Clockwise, -math.Pi / 2},
		{trackmodel.West, westCfg, trackmodel.Counterclockwise, math.Pi / 2},
	}
	for _, c := range cases {
		z := parking.BuildZone(c.lot.Block1, c.lot.Block2, c.section, c.direction,
			parking.DefaultParkingLotSpecs, parking.DefaultTrackDimensions)
		if err := math.Abs(parking.NormaliseAngle(z.TargetYaw - c.want)); err > 1e-9 {
			t.Errorf("%v/%v: TargetYaw = %v, want %v", c.section, c.direction, z.TargetYaw, c.want)
		}
	}
}

// TestBuildZone_TargetYawNeverPerpendicular guards the pre-2026-07-25 nose-in
// geometry: the target heading must be parallel to the wall, not across it.
func TestBuildZone_TargetYawNeverPerpendicular(t *testing.T) {
	t.Parallel()

	cfgs := map[trackmodel.Section]parking.ParkingLot{
		trackmodel.South: southCfg,
		trackmodel.North: northCfg,
		trackmodel.East:  eastCfg,
		trackmodel.West:  westCfg,
	}
	for section, lot := range cfgs {
		for _, dir := range []trackmodel.Direction{trackmodel.Clockwise, trackmodel.Counterclockwise} {
			z := parking.BuildZone(lot.Block1, lot.Block2, section, dir,
				parking.DefaultParkingLotSpecs, parking.DefaultTrackDimensions)
			wallNormal := math.Pi / 2.0
			if section == trackmodel.East || section == trackmodel.West {
				wallNormal = 0.0
			}
			err := math.Abs(parking.NormaliseAngle(z.TargetYaw - wallNormal))
			minErr := math.Min(err, math.Abs(math.Pi-err))
			if minErr <= 45*math.Pi/180 {
				t.Errorf(
					"%v/%v: TargetYaw %v is within 45 deg of perpendicular to wall",
					section,
					dir,
					z.TargetYaw,
				)
			}
		}
	}
}

// TestInsideZone_PerfectlyPlacedFootprintIsParked exercises the WRO rule,
// not the center-in-box approximation.
func TestInsideZone_PerfectlyPlacedFootprintIsParked(t *testing.T) {
	t.Parallel()

	cfg := parking.DefaultConfig()
	z := parking.BuildZone(
		southCfg.Block1,
		southCfg.Block2,
		trackmodel.South,
		trackmodel.Counterclockwise,
		parking.DefaultParkingLotSpecs,
		parking.DefaultTrackDimensions,
	)

	posOK, yawOK := parking.InsideZone(z.GapCX, z.GapCY, z.TargetYaw, z, cfg)
	if !posOK || !yawOK {
		t.Errorf("center + target yaw: posOK=%v yawOK=%v, want both true", posOK, yawOK)
	}
}

// TestInsideZone_centerInsideButFootprintProtrudingIsNotParked is the exact
// failure the old center-in-box test could not see: a robot across the bay has
// its center well inside the rectangle while most of the chassis sits out in
// the corridor.
func TestInsideZone_centerInsideButFootprintProtrudingIsNotParked(t *testing.T) {
	t.Parallel()

	cfg := parking.DefaultConfig()
	z := parking.BuildZone(
		southCfg.Block1,
		southCfg.Block2,
		trackmodel.South,
		trackmodel.Counterclockwise,
		parking.DefaultParkingLotSpecs,
		parking.DefaultTrackDimensions,
	)

	posOK, _ := parking.InsideZone(z.GapCX, z.GapCY, z.TargetYaw+math.Pi/2, z, cfg)
	if posOK {
		t.Error("footprint across the bay reported parked; containment must fail")
	}
}

// TestInsideZone_NoseThroughWallIsNotParked checks a robot with its nose
// through the outer wall is not classified as parked.
func TestInsideZone_NoseThroughWallIsNotParked(t *testing.T) {
	t.Parallel()

	cfg := parking.DefaultConfig()
	z := parking.BuildZone(
		southCfg.Block1,
		southCfg.Block2,
		trackmodel.South,
		trackmodel.Counterclockwise,
		parking.DefaultParkingLotSpecs,
		parking.DefaultTrackDimensions,
	)

	posOK, _ := parking.InsideZone(z.GapCX, 0.02, z.TargetYaw+math.Pi/2, z, cfg)
	if posOK {
		t.Error("nose through the wall reported parked; containment must fail")
	}
}

// TestInsideZone_OutsideAlongWallIsNotParked checks a robot outside the lot
// along the wall is not classified as parked.
func TestInsideZone_OutsideAlongWallIsNotParked(t *testing.T) {
	t.Parallel()

	cfg := parking.DefaultConfig()
	z := parking.BuildZone(
		southCfg.Block1,
		southCfg.Block2,
		trackmodel.South,
		trackmodel.Counterclockwise,
		parking.DefaultParkingLotSpecs,
		parking.DefaultTrackDimensions,
	)

	posOK, _ := parking.InsideZone(z.XMin-0.30, z.GapCY, z.TargetYaw, z, cfg)
	if posOK {
		t.Error("outside along the wall reported parked; containment must fail")
	}
}

// TestInsideZone_ParallelToleranceFollowsTwoWheelRule checks the +-2 cm
// wheel-to-wall rule (atan(0.02 / wheelbase) ~ 6 deg).
func TestInsideZone_ParallelToleranceFollowsTwoWheelRule(t *testing.T) {
	t.Parallel()

	cfg := parking.DefaultConfig()
	z := parking.BuildZone(
		southCfg.Block1,
		southCfg.Block2,
		trackmodel.South,
		trackmodel.Counterclockwise,
		parking.DefaultParkingLotSpecs,
		parking.DefaultTrackDimensions,
	)
	limit := math.Atan2(0.02, cfg.WheelbaseM)

	_, justInside := parking.InsideZone(z.GapCX, z.GapCY, z.TargetYaw+limit*0.9, z, cfg)
	_, justOutside := parking.InsideZone(z.GapCX, z.GapCY, z.TargetYaw+limit*1.1, z, cfg)
	if !justInside {
		t.Error("within the 2 cm wheel rule should be parallel")
	}
	if justOutside {
		t.Error("beyond the 2 cm wheel rule should not be parallel")
	}
}

// TestController_NotDoneInitially checks a fresh controller has not finished.
func TestController_NotDoneInitially(t *testing.T) {
	t.Parallel()

	ctrl := parking.NewParkController(
		southCfg,
		trackmodel.South,
		trackmodel.Counterclockwise,
		parking.DefaultConfig(),
		0,
		0,
	)
	if ctrl.IsDone() {
		t.Error("fresh controller should not be done")
	}
}

// TestController_DoneReturnsZeroSpeed holds the robot at a genuinely parked
// pose: the lot center, wall-parallel.
func TestController_DoneReturnsZeroSpeed(t *testing.T) {
	t.Parallel()

	cfg := parking.DefaultConfig()
	ctrl := parking.NewParkController(
		southCfg,
		trackmodel.South,
		trackmodel.Counterclockwise,
		cfg,
		0,
		0,
	)
	pose := trackmodel.Pose{X: ctrl.Zone().GapCX, Y: ctrl.Zone().GapCY, Yaw: ctrl.Zone().TargetYaw}

	var cmd parking.ParkCommand
	for range 800 {
		cmd = ctrl.Update(pose)
		if cmd.Done {
			break
		}
	}
	if !ctrl.IsDone() {
		t.Fatal("controller did not reach done at a parked pose")
	}
	if cmd.LinearMPS != 0.0 || !cmd.Done {
		t.Errorf("done command: LinearMPS=%v Done=%v, want 0.0/true", cmd.LinearMPS, cmd.Done)
	}
}

// TestController_FarRobotDrivesNonzeroSpeed checks a far robot gets a nonzero
// drive command and is not done.
func TestController_FarRobotDrivesNonzeroSpeed(t *testing.T) {
	t.Parallel()

	ctrl := parking.NewParkController(
		southCfg,
		trackmodel.South,
		trackmodel.Counterclockwise,
		parking.DefaultConfig(),
		0,
		0,
	)
	cmd := ctrl.Update(trackmodel.Pose{X: 1.15, Y: 2.5, Yaw: 0.0})
	if cmd.LinearMPS <= 0 {
		t.Errorf("LinearMPS = %v, want > 0 while driving to the lot", cmd.LinearMPS)
	}
	if cmd.Done {
		t.Error("must not be done on the first, far tick")
	}
}

// TestController_TargetBehindTriggersReverseReposition checks the 2026-07-11
// non-convergent-orbit guard at the unit level: a target behind the robot makes
// the controller reverse-and-reorient rather than drive away from it. The full
// closed-loop Ackermann sim lives in the Python suite; here we assert the
// recovery branch fires on the degenerate geometry directly.
func TestController_TargetBehindTriggersReverseReposition(t *testing.T) {
	t.Parallel()

	ctrl := parking.NewParkController(
		southCfg,
		trackmodel.South,
		trackmodel.Counterclockwise,
		parking.DefaultConfig(),
		0,
		0,
	)
	// Staging point is in front of the bay (north of it). Put the robot just
	// south of staging, facing south (away from it) so the staging target is
	// behind.
	staging := ctrl.Staging()
	pose := trackmodel.Pose{X: staging.X, Y: staging.Y - 0.05, Yaw: -math.Pi / 2}
	cmd := ctrl.Update(pose)

	if cmd.LinearMPS >= 0 {
		t.Errorf("LinearMPS = %v, want negative (reverse) when target is behind", cmd.LinearMPS)
	}
	if !ctrl.IsRepositioning() {
		t.Error("expected IsRepositioning true while reversing out of the degenerate pose")
	}
}

// TestController_UnreachableTargetTimesOutInsteadOfRunningForever covers the
// give-up branch: a robot that can never reach the position+yaw stop condition
// must hold, not chase the gap center for the whole match.
func TestController_UnreachableTargetTimesOut(t *testing.T) {
	t.Parallel()

	ctrl := parking.NewParkController(
		southCfg,
		trackmodel.South,
		trackmodel.Counterclockwise,
		parking.DefaultConfig(),
		0,
		50,
	)

	var cmd parking.ParkCommand
	for range 51 {
		cmd = ctrl.Update(trackmodel.Pose{X: 2.9, Y: 2.9, Yaw: 0.0})
	}
	if !cmd.Done || !ctrl.IsDone() || !ctrl.IsTimedOut() {
		t.Errorf(
			"done=%v isDone=%v timedOut=%v, want all true",
			cmd.Done,
			ctrl.IsDone(),
			ctrl.IsTimedOut(),
		)
	}
	if cmd.LinearMPS != 0.0 || cmd.SteeringNorm != 0.0 {
		t.Errorf(
			"give-up command: LinearMPS=%v SteeringNorm=%v, want 0/0",
			cmd.LinearMPS,
			cmd.SteeringNorm,
		)
	}
}

// TestController_ParkedPoseIsNotFlaggedTimedOut checks a genuinely parked
// pose is reported done, never timed out.
func TestController_ParkedPoseIsNotFlaggedTimedOut(t *testing.T) {
	t.Parallel()

	ctrl := parking.NewParkController(
		southCfg,
		trackmodel.South,
		trackmodel.Counterclockwise,
		parking.DefaultConfig(),
		0,
		400,
	)
	z := ctrl.Zone()
	pose := trackmodel.Pose{X: z.GapCX, Y: z.GapCY, Yaw: z.TargetYaw}

	var cmd parking.ParkCommand
	for range 400 {
		cmd = ctrl.Update(pose)
		if cmd.Done {
			break
		}
	}
	if !ctrl.IsDone() || ctrl.IsTimedOut() {
		t.Errorf("isDone=%v timedOut=%v, want true/false", ctrl.IsDone(), ctrl.IsTimedOut())
	}
}

// TestFootprintBreachesWall gives up before driving into the field wall. The
// lot is only 0.20 m deep and the chassis is 0.194 m wide, so a parallel pose
// at the lot center sits within the 0.05 m wall standoff -- which is exactly
// why ParkController checks InsideZone first and short-circuits to DONE before
// ever reaching this predicate at the goal. The predicate's job is the STAGE
// corridor and the near-wall approach: it must fire at the wall and stay quiet
// out in the open track.
func TestFootprintBreachesWall(t *testing.T) {
	t.Parallel()

	cfg := parking.DefaultConfig()
	z := parking.BuildZone(
		southCfg.Block1,
		southCfg.Block2,
		trackmodel.South,
		trackmodel.Counterclockwise,
		parking.DefaultParkingLotSpecs,
		parking.DefaultTrackDimensions,
	)
	// A robot with a corner driven into the wall, beyond the lot center.
	if !parking.FootprintBreachesWall(z.GapCX, 0.08, 0.0, z, cfg) {
		t.Error("footprint at the wall should breach it")
	}
	// Out in the open corridor, well clear of the wall: must stay quiet.
	if parking.FootprintBreachesWall(z.GapCX, 0.70, 0.0, z, cfg) {
		t.Error("footprint in the open corridor should not breach the wall")
	}
}

// TestFootprintBreachesMarkers gives up before clipping a fin.
func TestFootprintBreachesMarkers(t *testing.T) {
	t.Parallel()

	cfg := parking.DefaultConfig()
	z := parking.BuildZone(
		southCfg.Block1,
		southCfg.Block2,
		trackmodel.South,
		trackmodel.Counterclockwise,
		parking.DefaultParkingLotSpecs,
		parking.DefaultTrackDimensions,
	)
	// A corner swung out to a fin's inner face, within the lot depth.
	atFinX := z.XMin - cfg.MarkerStandoffM*0.5
	if !parking.FootprintBreachesMarkers(atFinX, z.GapCY, 0.0, z, cfg) {
		t.Error("footprint at a fin inner face should breach it")
	}
	// A robot parked cleanly in the lot center should not.
	if parking.FootprintBreachesMarkers(z.GapCX, z.GapCY, z.TargetYaw, z, cfg) {
		t.Error("cleanly-parked footprint should not breach a marker")
	}
}

// TestParkControllerFromMetadata_NilLotReturnsNil mirrors the Python
// open-challenge path: no lot, no maneuver.
func TestParkControllerFromMetadata_NilLotReturnsNil(t *testing.T) {
	t.Parallel()

	ctrl := parking.ParkControllerFromMetadata(
		nil, trackmodel.South, trackmodel.Counterclockwise, parking.DefaultConfig(),
	)
	if ctrl != nil {
		t.Error("nil lot should yield a nil controller")
	}
}

// TestConfigFor_DefaultsWithoutRoot returns literal defaults when no config
// root is supplied.
func TestConfigFor_DefaultsWithoutRoot(t *testing.T) {
	t.Parallel()

	cfg := parking.ConfigFor(nil, "", nil)
	def := parking.DefaultConfig()
	if cfg != def {
		t.Errorf("ConfigFor(nil, \"\", nil) = %+v, want defaults %+v", cfg, def)
	}
}
