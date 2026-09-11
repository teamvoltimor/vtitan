package collision_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/collision"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/kinematics"
)

// chassisLengthM/chassisWidthM stand in for RobotSpecs.LENGTH/WIDTH.
const (
	chassisLengthM = 0.30
	chassisWidthM  = 0.194
)

// symmetricTrackModel builds a 3x3m track with a 1.0m corridor on every
// side (inner block spanning [1,2]x[1,2]) and no obstacles, mirroring
// trackmodel/walls_test.go's symmetricWalls fixture -- there is no Python
// test oracle for track_model.py, so this is hand-derived from the
// documented geometry (see internal/sim/collision/doc.go).
func symmetricTrackModel(t *testing.T) *collision.TrackModel {
	t.Helper()
	widths := map[trackmodel.Section]float64{
		trackmodel.North: 1.0, trackmodel.South: 1.0, trackmodel.East: 1.0, trackmodel.West: 1.0,
	}
	geometry := trackmodel.CorridorGeometryFromWidths(widths, 3.0)
	return collision.NewTrackModel(collision.NewTrackModelParams{
		Geometry:  geometry,
		MinCoordM: 0.0,
		MaxCoordM: 3.0,
	})
}

func TestTrackModel_ContactSurfaceAt_NoneInTheOpenCorridor(t *testing.T) {
	t.Parallel()

	tm := symmetricTrackModel(t)
	got := tm.ContactSurfaceAt(0.5, 0.5, 0.0, chassisLengthM, chassisWidthM)
	if got != collision.SurfaceNone {
		t.Errorf("ContactSurfaceAt() in the open corridor = %v, want SurfaceNone", got)
	}
}

func TestTrackModel_ContactSurfaceAt_OuterWall(t *testing.T) {
	t.Parallel()

	tm := symmetricTrackModel(t)
	// Chassis centered at x=0.1 with a 0.30m length pokes 0.05m past x=0.
	got := tm.ContactSurfaceAt(0.1, 0.5, 0.0, chassisLengthM, chassisWidthM)
	if got != collision.SurfaceOuterWall {
		t.Errorf("ContactSurfaceAt() against the outer wall = %v, want SurfaceOuterWall", got)
	}
}

func TestTrackModel_ContactSurfaceAt_InnerWall(t *testing.T) {
	t.Parallel()

	tm := symmetricTrackModel(t)
	// Inner block spans [1,2]x[1,2]; a chassis centered at x=1.1 (facing
	// +x, so its length runs along X) pokes into it.
	got := tm.ContactSurfaceAt(1.1, 1.5, 0.0, chassisLengthM, chassisWidthM)
	if got != collision.SurfaceInnerWall {
		t.Errorf("ContactSurfaceAt() against the inner block = %v, want SurfaceInnerWall", got)
	}
}

func TestTrackModel_ContactSurfaceAt_SignVsParkingLotFin(t *testing.T) {
	t.Parallel()

	widths := map[trackmodel.Section]float64{
		trackmodel.North: 1.0, trackmodel.South: 1.0, trackmodel.East: 1.0, trackmodel.West: 1.0,
	}
	geometry := trackmodel.CorridorGeometryFromWidths(widths, 3.0)
	sign := collision.NewObstacleBoxFromPose(0.5, 0.5, 0.05, 0.05, 0.0, 1e-6, false)
	fin := collision.NewObstacleBoxFromPose(2.5, 0.5, 0.20, 0.02, 0.0, 1e-6, true)
	tm := collision.NewTrackModel(collision.NewTrackModelParams{
		Geometry:  geometry,
		MinCoordM: 0.0,
		MaxCoordM: 3.0,
		Obstacles: []collision.ObstacleBox{sign, fin},
	})

	if got := tm.ContactSurfaceAt(0.5, 0.5, 0.0, chassisLengthM, chassisWidthM); got != collision.SurfaceObstacle {
		t.Errorf("ContactSurfaceAt() touching a traffic sign = %v, want SurfaceObstacle (nudgeable, 9.20)", got)
	}
	if got := tm.ContactSurfaceAt(2.5, 0.5, 0.0, chassisLengthM, chassisWidthM); got != collision.SurfaceParkingLot {
		t.Errorf(
			"ContactSurfaceAt() touching a parking-lot fin = %v, want SurfaceParkingLot (unforgivable, 9.24.7)",
			got,
		)
	}
}

func TestTrackModel_FootprintCollides_MatchesContactSurfaceAt(t *testing.T) {
	t.Parallel()

	tm := symmetricTrackModel(t)
	if tm.FootprintCollides(0.5, 0.5, 0.0, chassisLengthM, chassisWidthM) {
		t.Error("FootprintCollides() in the open corridor = true, want false")
	}
	if !tm.FootprintCollides(0.1, 0.5, 0.0, chassisLengthM, chassisWidthM) {
		t.Error("FootprintCollides() against the outer wall = false, want true")
	}
}

func TestTrackModel_ObstacleDisplacements_MeasuresIntrusionDepth(t *testing.T) {
	t.Parallel()

	widths := map[trackmodel.Section]float64{
		trackmodel.North: 1.0, trackmodel.South: 1.0, trackmodel.East: 1.0, trackmodel.West: 1.0,
	}
	geometry := trackmodel.CorridorGeometryFromWidths(widths, 3.0)
	obstacle := collision.NewObstacleBoxFromPose(0.5, 0.5, 0.10, 0.05, 0.0, 1e-6, false)
	tm := collision.NewTrackModel(collision.NewTrackModelParams{
		Geometry:  geometry,
		MinCoordM: 0.0,
		MaxCoordM: 3.0,
		Obstacles: []collision.ObstacleBox{obstacle},
	})

	// Obstacle: length=0.10 (its x-extent, unrotated), width=0.05 (its
	// y-extent) -- spans x in [0.45, 0.55], y in [0.475, 0.525]. A chassis
	// centered at (0.5, 0.5) with length=0.30/width=0.194 fully swallows it
	// on both axes, so the SAT penetration is the smaller of the two full
	// extents: min(0.10, 0.05) = 0.05.
	depths := tm.ObstacleDisplacements(0.5, 0.5, 0.0, chassisLengthM, chassisWidthM)
	depth, touched := depths[0]
	if !touched {
		t.Fatal("ObstacleDisplacements() reports no contact, want obstacle 0 touched")
	}
	if math.Abs(depth-0.05) > 1e-9 {
		t.Errorf("ObstacleDisplacements()[0] = %v, want 0.05", depth)
	}

	center, ok := tm.ObstacleCenter(0)
	if !ok || center.X != 0.5 || center.Y != 0.5 {
		t.Errorf("ObstacleCenter(0) = (%v, %v, %v), want (0.5, 0.5, true)", center.X, center.Y, ok)
	}
}

func TestTrackModel_RaycastScan_ObstacleShortensTheWallRange(t *testing.T) {
	t.Parallel()

	widths := map[trackmodel.Section]float64{
		trackmodel.North: 1.0, trackmodel.South: 1.0, trackmodel.East: 1.0, trackmodel.West: 1.0,
	}
	geometry := trackmodel.CorridorGeometryFromWidths(widths, 3.0)
	// A 0.1x0.1 obstacle centered at (0.7, 0.5), between the sensor at
	// (0.1, 0.5) and the inner block's west face at x=1.0.
	obstacle := collision.NewObstacleBoxFromPose(0.7, 0.5, 0.1, 0.1, 0.0, 1e-6, false)
	tm := collision.NewTrackModel(collision.NewTrackModelParams{
		Geometry:           geometry,
		MinCoordM:          0.0,
		MaxCoordM:          3.0,
		Obstacles:          []collision.ObstacleBox{obstacle},
		LidarSeesObstacles: true,
	})

	got := tm.RaycastScan(0.1, 0.5, 0.0, []float64{0.0}, 0.05, 12.0)
	// Without the obstacle the ray would travel 0.9m to the inner block's
	// west face; with it, it stops at the obstacle's near face, x=0.65.
	want := 0.65 - 0.1
	if math.Abs(got[0]-want) > 1e-6 {
		t.Errorf("RaycastScan()[0] = %v, want %v (obstacle's near face, not the 0.9m wall)", got[0], want)
	}
}

func TestTrackModel_RaycastScan_ObstaclesIgnoredWhenLidarCannotSeeThem(t *testing.T) {
	t.Parallel()

	widths := map[trackmodel.Section]float64{
		trackmodel.North: 1.0, trackmodel.South: 1.0, trackmodel.East: 1.0, trackmodel.West: 1.0,
	}
	geometry := trackmodel.CorridorGeometryFromWidths(widths, 3.0)
	// Sensor at y=1.5, inside the inner block's y-range of [1,2], so the
	// ray actually crosses its west face at x=1.0 once the obstacle is
	// ignored -- unlike the sibling test above, whose y=0.5 ray never
	// enters the inner block's y-range at all.
	obstacle := collision.NewObstacleBoxFromPose(0.7, 1.5, 0.1, 0.1, 0.0, 1e-6, false)
	tm := collision.NewTrackModel(collision.NewTrackModelParams{
		Geometry:           geometry,
		MinCoordM:          0.0,
		MaxCoordM:          3.0,
		Obstacles:          []collision.ObstacleBox{obstacle},
		LidarSeesObstacles: false,
	})

	got := tm.RaycastScan(0.1, 1.5, 0.0, []float64{0.0}, 0.05, 12.0)
	want := 0.9 // straight through to the inner block's west face at x=1.0
	if math.Abs(got[0]-want) > 1e-6 {
		t.Errorf("RaycastScan()[0] = %v, want %v (obstacle ignored)", got[0], want)
	}
}

// TestAllowedStep_ClearMoveIsUnrestricted is the trivial case: a step that
// lands nowhere solid returns unchanged.
func TestAllowedStep_ClearMoveIsUnrestricted(t *testing.T) {
	t.Parallel()

	tm := symmetricTrackModel(t)
	solid := collision.NewSurfaceSet(collision.SurfaceOuterWall, collision.SurfaceInnerWall)
	state := kinematics.AckermannState{X: 0.5, Y: 0.5, Yaw: 0.0}
	candidate := kinematics.AckermannState{X: 0.55, Y: 0.5, Yaw: 0.0, V: 0.1}

	got := collision.AllowedStep(tm, solid, chassisLengthM, chassisWidthM, state, candidate)
	if got == nil {
		t.Fatal("AllowedStep() = nil for a clear move, want the candidate unchanged")
	}
	if *got != candidate {
		t.Errorf("AllowedStep() = %+v, want unchanged candidate %+v", *got, candidate)
	}
}

// TestAllowedStep_HeadOnIntoAWallDoesNotMove is the "cut back to nothing"
// branch: a step driving squarely into a wall the chassis is already
// against makes no progress.
func TestAllowedStep_HeadOnIntoAWallDoesNotMove(t *testing.T) {
	t.Parallel()

	tm := symmetricTrackModel(t)
	solid := collision.NewSurfaceSet(collision.SurfaceOuterWall, collision.SurfaceInnerWall)
	// Chassis nose already at the outer wall (x=0); a further -x step can
	// only push deeper in.
	state := kinematics.AckermannState{X: chassisLengthM / 2, Y: 0.5, Yaw: 0.0}
	candidate := kinematics.AckermannState{X: state.X - 0.05, Y: 0.5, Yaw: 0.0, V: 0.1}

	got := collision.AllowedStep(tm, solid, chassisLengthM, chassisWidthM, state, candidate)
	if got != nil {
		t.Errorf("AllowedStep() = %+v, want nil (head-on contact makes no progress)", *got)
	}
}

// TestAllowedStep_GrazingTurnStillMovesForward is the regression test for
// the historical fix this file exists to preserve (Python commit
// 06b7fc1a): a chassis pinned at its maximum yaw against a wall must still
// be able to advance -- translation and rotation are bisected SEPARATELY,
// so a turn that would tunnel through the wall gets pulled back while the
// forward motion the wall does not block still goes through.
func TestAllowedStep_GrazingTurnStillMovesForward(t *testing.T) {
	t.Parallel()

	tm := symmetricTrackModel(t)
	solid := collision.NewSurfaceSet(collision.SurfaceOuterWall, collision.SurfaceInnerWall)

	// Nose 0.12m from the south outer wall (y=0), driving forward (+x)
	// while also commanding a large yaw change. At the full candidate yaw
	// the rotated footprint's lateral extent exceeds that 0.12m clearance
	// and clips the wall -- but the translation alone (turn=0) does not,
	// since it does not touch yaw at all.
	state := kinematics.AckermannState{X: 1.5, Y: 0.12, Yaw: 0.0}
	candidate := kinematics.AckermannState{X: 1.55, Y: 0.12, Yaw: 1.0, V: 0.1}

	got := collision.AllowedStep(tm, solid, chassisLengthM, chassisWidthM, state, candidate)
	if got == nil {
		t.Fatal("AllowedStep() = nil, want a pose that still advances (translation was clear)")
	}
	if got.X != candidate.X || got.Y != candidate.Y {
		t.Errorf("AllowedStep() translation = (%v, %v), want the full candidate translation (%v, %v)",
			got.X, got.Y, candidate.X, candidate.Y)
	}
	if got.Yaw == candidate.Yaw {
		t.Error("AllowedStep() yaw = full candidate yaw, want it bisected down to what still fits")
	}
}
