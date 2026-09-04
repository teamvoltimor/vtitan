package bayexit

import (
	"math"
	"testing"
)

// There is no Python oracle test file for bay_exit.py (none exists in the
// source tree), so these tests pin the module's own documented geometric
// contracts directly rather than porting fixture-for-fixture.

func TestRectCorners_AxisAlignedSquareAtOrigin(t *testing.T) {
	t.Parallel()

	corners := rectCorners(0, 0, 0, 2.0, 1.0)
	want := []point{{1, 0.5}, {1, -0.5}, {-1, -0.5}, {-1, 0.5}}
	for i, c := range corners {
		if math.Abs(c.along-want[i].along) > 1e-9 || math.Abs(c.out-want[i].out) > 1e-9 {
			t.Errorf("corner %d = %+v, want %+v", i, c, want[i])
		}
	}
}

func TestRectCorners_TranslatesByAlongOut(t *testing.T) {
	t.Parallel()

	corners := rectCorners(5.0, 3.0, 0, 2.0, 1.0)
	for _, c := range corners {
		if math.Abs(c.along-5.0) > 1.0+1e-9 || math.Abs(c.out-3.0) > 0.5+1e-9 {
			t.Errorf("corner %+v not within the translated rectangle", c)
		}
	}
}

func TestGap_SeparatedRectanglesReturnPositive(t *testing.T) {
	t.Parallel()

	a := rectCorners(0, 0, 0, 1.0, 1.0)
	b := rectCorners(5, 0, 0, 1.0, 1.0)
	if g := gap(a, b); g <= 0 {
		t.Errorf("gap() = %v, want > 0 for well-separated rectangles", g)
	}
}

func TestGap_OverlappingRectanglesReturnNegative(t *testing.T) {
	t.Parallel()

	a := rectCorners(0, 0, 0, 2.0, 2.0)
	b := rectCorners(0.5, 0, 0, 2.0, 2.0)
	if g := gap(a, b); g >= 0 {
		t.Errorf("gap() = %v, want < 0 for overlapping rectangles", g)
	}
}

func TestGap_ExactSeparationIsApproximatelyZero(t *testing.T) {
	t.Parallel()

	// Two 1x1 squares placed edge-to-edge along "along": touching, gap ~ 0.
	a := rectCorners(0, 0, 0, 1.0, 1.0)
	b := rectCorners(1.0, 0, 0, 1.0, 1.0)
	if g := gap(a, b); math.Abs(g) > 1e-9 {
		t.Errorf("gap() = %v, want ~0 for edge-touching rectangles", g)
	}
}

func TestGap_IsSymmetric(t *testing.T) {
	t.Parallel()

	a := rectCorners(0, 0, 0.3, 1.2, 0.6)
	b := rectCorners(3, 1, -0.2, 0.8, 0.4)
	if g1, g2 := gap(a, b), gap(b, a); math.Abs(g1-g2) > 1e-9 {
		t.Errorf("gap(a, b) = %v, gap(b, a) = %v, want equal", g1, g2)
	}
}

// TestFinRects_MirroredAboutTheAlongAxis matches _fin_rects' documented
// symmetry: the two fins are the same rectangle mirrored across along=0.
func TestFinRects_MirroredAboutTheAlongAxis(t *testing.T) {
	t.Parallel()

	cfg := DefaultConfig()
	fins := finRects(cfg)
	// Corner ORDER differs between the two fins (each is wound the same
	// way around its own rectangle, which starts from opposite corners
	// once mirrored), so match by (out, |along|) instead of by index.
	for _, left := range fins[0] {
		found := false
		for _, right := range fins[1] {
			if math.Abs(left.along+right.along) < 1e-9 && math.Abs(left.out-right.out) < 1e-9 {
				found = true
				break
			}
		}
		if !found {
			t.Errorf("left corner %+v has no mirrored counterpart in the right fin", left)
		}
	}
}

// TestFinRects_SpanTheLotDepthFromTheWall matches _fin_rects' documented
// depth: the wall side sits at -WallOffsetM and the tip at
// -WallOffsetM+Length.
func TestFinRects_SpanTheLotDepthFromTheWall(t *testing.T) {
	t.Parallel()

	cfg := DefaultConfig()
	fins := finRects(cfg)
	wall := -cfg.ParkingLot.WallOffsetM
	tip := wall + cfg.ParkingLot.Length
	for _, fin := range fins {
		outs := map[float64]bool{}
		for _, c := range fin {
			outs[c.out] = true
		}
		if !outs[wall] || !outs[tip] {
			t.Errorf("fin outs = %v, want wall=%v and tip=%v present", outs, wall, tip)
		}
	}
}

// TestFinRects_ChassisAtOriginFitsBetweenThem: the chassis, centred at the
// pocket origin, must not already overlap either fin -- otherwise the
// clearance guard would refuse to move on tick 0.
func TestFinRects_ChassisAtOriginFitsBetweenThem(t *testing.T) {
	t.Parallel()

	cfg := DefaultConfig()
	chassis := rectCorners(0, 0, 0, cfg.ChassisLengthM, cfg.ChassisWidthM)
	fins := finRects(cfg)
	for i, fin := range fins {
		if g := gap(chassis, fin); g <= 0 {
			t.Errorf("gap(chassis, fin[%d]) = %v, want > 0 at the pocket origin", i, g)
		}
	}
}
