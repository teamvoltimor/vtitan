package collision

import (
	"math"
	"testing"
)

// testLengthM/testWidthM stand in for RobotSpecs.LENGTH/WIDTH -- the
// formulas under test hold for any rectangle, so literal values are
// chosen for numerical convenience, matching the rest of this port's test
// literals (see kinematics_test.go's identical note).
const (
	testLengthM = 0.30
	testWidthM  = 0.194
)

const geomTolerance = 1e-9

// TestRectCornersIsCentredOnThePose matches the Python
// TestRotationIsAboutTheChassisCentre.test_the_collision_rectangle_is_centred_on_the_pose test.
//
//nolint:misspell // literal Python identifier, not prose
func TestRectCornersIsCentredOnThePose(t *testing.T) {
	t.Parallel()

	corners := rectCorners(0.0, 0.0, 0.0, testLengthM, testWidthM)

	minX, maxX := math.Inf(1), math.Inf(-1)
	minY, maxY := math.Inf(1), math.Inf(-1)
	for _, c := range corners {
		minX, maxX = math.Min(minX, c.X), math.Max(maxX, c.X)
		minY, maxY = math.Min(minY, c.Y), math.Max(maxY, c.Y)
	}

	if math.Abs(minX-(-testLengthM/2)) > geomTolerance || math.Abs(maxX-testLengthM/2) > geomTolerance {
		t.Errorf("x-extent = [%v, %v], want [%v, %v]", minX, maxX, -testLengthM/2, testLengthM/2)
	}
	if math.Abs(minY-(-testWidthM/2)) > geomTolerance || math.Abs(maxY-testWidthM/2) > geomTolerance {
		t.Errorf("y-extent = [%v, %v], want [%v, %v]", minY, maxY, -testWidthM/2, testWidthM/2)
	}
}

// TestTheTailSwingsOutAsFarAsTheNoseSwingsIn matches
// TestRotationIsAboutTheChassisCentre.test_the_tail_swings_out_as_far_as_the_nose_swings_in.
//
// The property that constrains "pass the sign square, then turn". With a
// center-referenced body, rotating in place sweeps the rear corner outward
// by exactly what the front corner sweeps inward.
func TestTheTailSwingsOutAsFarAsTheNoseSwingsIn(t *testing.T) {
	t.Parallel()

	yaw := math.Pi / 6 // 30 degrees
	corners := rectCorners(0.0, 0.0, yaw, testLengthM, testWidthM)

	minY, maxY := math.Inf(1), math.Inf(-1)
	for _, c := range corners {
		minY, maxY = math.Min(minY, c.Y), math.Max(maxY, c.Y)
	}
	if math.Abs(minY-(-maxY)) > geomTolerance {
		t.Errorf("footprint is not symmetric about the pose: minY=%v, maxY=%v", minY, maxY)
	}
}

// TestLateralHalfExtentMatchesTheClearanceFormula matches
// TestRotationIsAboutTheChassisCentre.test_lateral_half_extent_matches_the_clearance_formula:
// (L/2)|sin th| + (W/2)|cos th|.
func TestLateralHalfExtentMatchesTheClearanceFormula(t *testing.T) {
	t.Parallel()

	for _, degrees := range []float64{0, 20, 28, 40, 60, 84} {
		yaw := degrees * math.Pi / 180
		corners := rectCorners(0.0, 0.0, yaw, testLengthM, testWidthM)

		measured := math.Inf(-1)
		for _, c := range corners {
			measured = math.Max(measured, c.Y)
		}
		predicted := (testLengthM/2)*math.Abs(math.Sin(yaw)) + (testWidthM/2)*math.Abs(math.Cos(yaw))

		if math.Abs(measured-predicted) > geomTolerance {
			t.Errorf("at %v deg: measured = %v, predicted = %v", degrees, measured, predicted)
		}
	}
}

// TestConvexOverlap_SeparatedBoxesDoNotOverlap and its sibling cover the
// separating-axis test used by ContactSurfaceAt/ObstacleDisplacements --
// no Python test oracle exists for track_model.py, so these are
// hand-derived from the documented geometry (see doc.go).
func TestConvexOverlap_SeparatedBoxesDoNotOverlap(t *testing.T) {
	t.Parallel()

	rect := rectCorners(0.0, 0.0, 0.0, 0.2, 0.2) // spans [-0.1, 0.1] on both axes
	far := box{5.0, 5.0, 5.1, 5.1}.corners()

	if convexOverlap(rect, far, 0.0) {
		t.Error("convexOverlap() = true for boxes 5m apart, want false")
	}
}

func TestConvexOverlap_OverlappingBoxesOverlap(t *testing.T) {
	t.Parallel()

	rect := rectCorners(0.0, 0.0, 0.0, 0.2, 0.2) // spans [-0.1, 0.1]
	overlapping := box{0.05, 0.05, 0.5, 0.5}.corners()

	if !convexOverlap(rect, overlapping, 0.0) {
		t.Error("convexOverlap() = false for boxes overlapping by 0.05m, want true")
	}
}

func TestConvexPenetration_MeasuresTheIntrusionDepth(t *testing.T) {
	t.Parallel()

	// A 0.2x0.2 rect centered at origin overlaps a box starting at x=0.05:
	// the intrusion along X is 0.1 (rect's right edge) - 0.05 = 0.05m.
	rect := rectCorners(0.0, 0.0, 0.0, 0.2, 0.2)
	obstacle := box{0.05, -0.5, 0.5, 0.5}.corners()

	got := convexPenetration(rect, obstacle, 0.0)
	want := 0.05
	if math.Abs(got-want) > geomTolerance {
		t.Errorf("convexPenetration() = %v, want %v", got, want)
	}
}

func TestConvexPenetration_ZeroWhenNotOverlapping(t *testing.T) {
	t.Parallel()

	rect := rectCorners(0.0, 0.0, 0.0, 0.2, 0.2)
	far := box{5.0, 5.0, 5.1, 5.1}.corners()

	if got := convexPenetration(rect, far, 0.0); got != 0.0 {
		t.Errorf("convexPenetration() = %v, want 0 for non-overlapping boxes", got)
	}
}

// TestNewObstacleBoxFromPose_SwapsExtentsOnAQuarterTurn matches
// ObstacleBox.from_pose's documented behavior: a near-90deg yaw swaps
// length/width.
func TestNewObstacleBoxFromPose_SwapsExtentsOnAQuarterTurn(t *testing.T) {
	t.Parallel()

	const tolerance = 1e-6
	straight := NewObstacleBoxFromPose(0, 0, 0.20, 0.05, 0.0, tolerance, false)
	if straight.SizeX != 0.20 || straight.SizeY != 0.05 {
		t.Errorf("yaw=0: got (%v, %v), want (0.20, 0.05)", straight.SizeX, straight.SizeY)
	}

	turned := NewObstacleBoxFromPose(0, 0, 0.20, 0.05, math.Pi/2, tolerance, false)
	if turned.SizeX != 0.05 || turned.SizeY != 0.20 {
		t.Errorf("yaw=pi/2: got (%v, %v), want (0.05, 0.20)", turned.SizeX, turned.SizeY)
	}
}

// TestRaycastBox_HitsAndMisses matches _raycast_box's slab method directly.
func TestRaycastBox_HitsAndMisses(t *testing.T) {
	t.Parallel()

	b := box{xMin: 1.0, yMin: -0.5, xMax: 1.2, yMax: 0.5}

	// Straight ahead (+x) from the origin hits the box's near face at x=1.0.
	if got := raycastBox(0, 0, 1, 0, b, 12.0); math.Abs(got-1.0) > geomTolerance {
		t.Errorf("raycastBox() forward = %v, want 1.0", got)
	}

	// Straight up (+y) from the origin never reaches the box (it's centered
	// on y=0 with the ray running parallel to x): miss, so max range.
	if got := raycastBox(0, 0, 0, 1, b, 12.0); got != 12.0 {
		t.Errorf("raycastBox() perpendicular miss = %v, want 12.0 (max range)", got)
	}
}
