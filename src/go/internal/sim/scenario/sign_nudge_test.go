package scenario

import (
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/collision"
)

// signCenterX/signCenterY place the test sign inside the free corridor band
// (the inner block spans [1,2]x[1,2]; this sits well clear of it, matching
// TestTrackModel_ObstacleDisplacements_MeasuresIntrusionDepth's fixture).
const (
	signCenterX = 0.5
	signCenterY = 0.5
)

// chassisLengthTest/chassisWidthTest stand in for RobotSpecs.LENGTH/WIDTH,
// matching internal/sim/collision's own test fixtures.
const (
	chassisLengthTest = 0.30
	chassisWidthTest  = 0.194
)

// signNudgeTestTrack builds a 3x3m track with one sign-sized obstacle
// centered at (signCenterX, signCenterY).
func signNudgeTestTrack(t *testing.T) *collision.TrackModel {
	t.Helper()
	widths := map[trackmodel.Section]float64{
		trackmodel.North: 1.0, trackmodel.South: 1.0, trackmodel.East: 1.0, trackmodel.West: 1.0,
	}
	geometry := trackmodel.CorridorGeometryFromWidths(widths, 3.0)
	obstacle := collision.NewObstacleBoxFromPose(
		signCenterX, signCenterY, signObstacleWidthM, signObstacleDepthM, 0.0, 1e-6, false,
	)
	return collision.NewTrackModel(collision.NewTrackModelParams{
		Geometry:  geometry,
		MinCoordM: 0.0,
		MaxCoordM: 3.0,
		Obstacles: []collision.ObstacleBox{obstacle},
	})
}

// TestSignNudgeState_TouchWithinToleranceIsNotACollision mirrors
// test_parking_scoring.py-style pinning for scoring.py's
// _score_obstacle_contact: a graze that never pushes the sign past
// maxLegalSignDisplacementM must not read as terminal.
func TestSignNudgeState_TouchWithinToleranceIsNotACollision(t *testing.T) {
	t.Parallel()
	tm := signNudgeTestTrack(t)
	nudge := newSignNudgeState(signCenterX, signCenterY-0.08)

	// A single tick's worth of approach toward the sign, well short of
	// maxLegalSignDisplacementM (~0.059m).
	x, y, yaw := signCenterX, signCenterY-0.05, 0.0
	surface := tm.ContactSurfaceAt(x, y, yaw, chassisLengthTest, chassisWidthTest)
	if surface != collision.SurfaceObstacle {
		t.Fatalf("setup: ContactSurfaceAt = %v, want SurfaceObstacle (chassis must overlap the sign)", surface)
	}

	got := nudge.score(tm, surface, x, y, yaw, chassisLengthTest, chassisWidthTest)
	if got != collision.SurfaceNone {
		t.Errorf("score() = %v, want SurfaceNone (a small graze is legal under WRO 9.20)", got)
	}
}

// TestSignNudgeState_SustainedPushPastToleranceIsACollision mirrors the
// veto half of _score_obstacle_contact: once accumulated push exceeds
// maxLegalSignDisplacementM, the surface reports terminal again.
func TestSignNudgeState_SustainedPushPastToleranceIsACollision(t *testing.T) {
	t.Parallel()
	tm := signNudgeTestTrack(t)
	// Start well short of the sign so repeated forward ticks accumulate
	// push past the tolerance before the loop ends.
	nudge := newSignNudgeState(signCenterX, signCenterY-0.20)

	yaw := 0.0
	y := signCenterY - 0.20
	var surface collision.ContactSurface
	for range 20 {
		y += 0.01 // 1cm/tick toward the sign
		x := signCenterX
		raw := tm.ContactSurfaceAt(x, y, yaw, chassisLengthTest, chassisWidthTest)
		surface = nudge.score(tm, raw, x, y, yaw, chassisLengthTest, chassisWidthTest)
		if surface != collision.SurfaceNone {
			break
		}
	}

	if surface != collision.SurfaceObstacle {
		t.Errorf("score() after sustained approach = %v, want SurfaceObstacle (push exceeds maxLegalSignDisplacementM)", surface)
	}
}

// TestSignNudgeState_ReceedingTravelDoesNotAccumulate mirrors the
// dot-product gate directly (push < 0 is dropped, not subtracted): moving
// AWAY from the sign every tick must never accumulate any push, however far
// the chassis travels, matching _score_obstacle_contact's `if push > 0.0`.
func TestSignNudgeState_ReceedingTravelDoesNotAccumulate(t *testing.T) {
	t.Parallel()
	tm := signNudgeTestTrack(t)
	// Start already overlapping the sign, then back straight away from it
	// every tick -- a large SurfaceObstacle overlap that keeps receding.
	nudge := newSignNudgeState(signCenterX, signCenterY)

	yaw := 0.0
	y := signCenterY
	var surface collision.ContactSurface
	for range 50 {
		y -= 0.002 // receding from the sign every tick
		x := signCenterX
		raw := tm.ContactSurfaceAt(x, y, yaw, chassisLengthTest, chassisWidthTest)
		surface = nudge.score(tm, raw, x, y, yaw, chassisLengthTest, chassisWidthTest)
	}

	if surface != collision.SurfaceNone {
		t.Errorf("score() after 50 receding ticks = %v, want SurfaceNone (no tick ever pushed toward the sign)", surface)
	}
}
