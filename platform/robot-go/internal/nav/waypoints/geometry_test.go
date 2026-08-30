package waypoints_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/waypoints"
)

const (
	narrowWidthM = 0.6 // CorridorDimensions.NARROW
	wideWidthM   = 1.0 // CorridorDimensions.WIDE
	arcCapM      = 0.45
	arcBiasM     = 0.05
)

func TestStraightWaypoints_X(t *testing.T) {
	t.Parallel()

	got := waypoints.StraightWaypoints(1.5, true, 0.0, 1.0, 3)
	want := []trackmodel.Waypoint{{X: 1.5, Y: 0.0}, {X: 1.5, Y: 0.5}, {X: 1.5, Y: 1.0}}
	if len(got) != len(want) {
		t.Fatalf("len = %d, want %d", len(got), len(want))
	}
	for i := range want {
		if got[i] != want[i] {
			t.Errorf("[%d] = %v, want %v", i, got[i], want[i])
		}
	}
}

func TestStraightWaypoints_Y(t *testing.T) {
	t.Parallel()

	got := waypoints.StraightWaypoints(0.5, false, 1.0, 2.0, 2)
	want := []trackmodel.Waypoint{{X: 1.0, Y: 0.5}, {X: 2.0, Y: 0.5}}
	if len(got) != len(want) {
		t.Fatalf("len = %d, want %d", len(got), len(want))
	}
	for i := range want {
		if got[i] != want[i] {
			t.Errorf("[%d] = %v, want %v", i, got[i], want[i])
		}
	}
}

func TestArcWithEndpoints_PointsStayOnRadius(t *testing.T) {
	t.Parallel()

	arc := waypoints.ArcWithEndpoints(trackmodel.Waypoint{X: 1.0, Y: 1.0}, 0.45, 0.0, math.Pi/2, 3)

	if len(arc) != 5 {
		t.Fatalf("len(arc) = %d, want 5", len(arc))
	}
	for _, wp := range arc {
		dist := math.Hypot(wp.X-1.0, wp.Y-1.0)
		if math.Abs(dist-0.45) > 0.01 {
			t.Errorf("point %v is %.4f from center, want ~0.45", wp, dist)
		}
	}
}

func TestArcWithEndpoints_Endpoints(t *testing.T) {
	t.Parallel()

	arc := waypoints.ArcWithEndpoints(
		trackmodel.Waypoint{X: 1.5, Y: 1.5},
		0.45,
		math.Pi,
		1.5*math.Pi,
		1,
	)

	if len(arc) != 3 {
		t.Fatalf("len(arc) = %d, want 3", len(arc))
	}
	if math.Abs(arc[0].X-1.05) > 0.01 || math.Abs(arc[0].Y-1.5) > 0.01 {
		t.Errorf("arc[0] = %v, want ~(1.05, 1.5)", arc[0])
	}
	last := arc[len(arc)-1]
	if math.Abs(last.X-1.5) > 0.01 || math.Abs(last.Y-1.05) > 0.01 {
		t.Errorf("last point = %v, want ~(1.5, 1.05)", last)
	}
}

func TestCornerArcRadius_OnlyNarrowToNarrowTightens(t *testing.T) {
	t.Parallel()

	wantNarrow := narrowWidthM/2 - arcBiasM
	got := waypoints.CornerArcRadius(narrowWidthM, narrowWidthM, arcBiasM, arcCapM)
	if math.Abs(got-wantNarrow) > 1e-9 {
		t.Errorf("narrow-to-narrow = %v, want %v", got, wantNarrow)
	}

	wantWide := wideWidthM/2 - arcBiasM
	combos := [][2]float64{
		{narrowWidthM, wideWidthM},
		{wideWidthM, narrowWidthM},
		{wideWidthM, wideWidthM},
	}
	for _, c := range combos {
		if gotWide := waypoints.CornerArcRadius(c[0], c[1], arcBiasM, arcCapM); math.Abs(
			gotWide-wantWide,
		) > 1e-9 {
			t.Errorf("CornerArcRadius(%v, %v) = %v, want %v", c[0], c[1], gotWide, wantWide)
		}
	}
}

func TestCornerArcRadius_SymmetricInEntryAndExit(t *testing.T) {
	t.Parallel()

	combos := [][2]float64{
		{narrowWidthM, wideWidthM},
		{wideWidthM, narrowWidthM},
		{narrowWidthM, narrowWidthM},
	}
	for _, c := range combos {
		fwd := waypoints.CornerArcRadius(c[0], c[1], arcBiasM, arcCapM)
		rev := waypoints.CornerArcRadius(c[1], c[0], arcBiasM, arcCapM)
		if fwd != rev {
			t.Errorf(
				"CornerArcRadius(%v,%v)=%v != CornerArcRadius(%v,%v)=%v",
				c[0],
				c[1],
				fwd,
				c[1],
				c[0],
				rev,
			)
		}
	}
}

func TestCornerArcRadius_NeverExceedsTheCap(t *testing.T) {
	t.Parallel()

	oversized := wideWidthM * 4
	if got := waypoints.CornerArcRadius(oversized, oversized, arcBiasM, arcCapM); got != arcCapM {
		t.Errorf("CornerArcRadius() = %v, want capped at %v", got, arcCapM)
	}
}

func TestCornerArcRadius_OutwardBiasWidensTheArc(t *testing.T) {
	t.Parallel()

	inward := waypoints.CornerArcRadius(narrowWidthM, narrowWidthM, arcBiasM, arcCapM)
	outward := waypoints.CornerArcRadius(narrowWidthM, narrowWidthM, -arcBiasM, arcCapM)
	if outward <= inward {
		t.Errorf("outward bias radius %v <= inward %v, want outward larger", outward, inward)
	}
}
