package waypoints

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// benchInput builds a symmetric 2.0 m-wide layout on a 4.0 m track, matching
// cmd/track-navigator's default bench geometry, with a resolved CCW direction.
func benchInput(maxCoordM, chassisWidthM float64) PlannerInput {
	geom := trackmodel.CorridorGeometryFromWidths(map[trackmodel.Section]float64{
		trackmodel.North: 2.0,
		trackmodel.South: 2.0,
		trackmodel.East:  2.0,
		trackmodel.West:  2.0,
	}, maxCoordM)
	dir := trackmodel.Counterclockwise
	return PlannerInput{
		Geometry:      geom,
		Starting:      StartingConditions{Direction: &dir, Section: trackmodel.South, Position: trackmodel.Waypoint{X: -maxCoordM + 1, Y: -maxCoordM + 1}},
		MaxCoordM:     maxCoordM,
		ChassisWidthM: chassisWidthM,
	}
}

func TestCalculateWaypoints_CCWLoopStartsNearSpawn(t *testing.T) {
	cfg := DefaultConfig()
	cfg.CornerArcAssumeWide = false
	input := benchInput(4.0, 0.30)
	dir := trackmodel.Counterclockwise
	input.Starting.Direction = &dir

	wps, err := CalculateWaypoints(input, 1, cfg, nil, AllConfirmed())
	if err != nil {
		t.Fatalf("CalculateWaypoints() error = %v", err)
	}
	if len(wps) < 20 {
		t.Fatalf("len(waypoints) = %d, want > 20 (matching calculate_waypoints' own sanity bound)", len(wps))
	}

	// Loop order with a South start, CCW, anchored at East:
	// absolute CCW = [East, North, West, South] rotated to start at South
	// => [South, East, North, West].
	wantOrder := []trackmodel.Section{
		trackmodel.South, trackmodel.East, trackmodel.North, trackmodel.West,
	}
	got := trackmodel.LoopOrder(trackmodel.South, trackmodel.Counterclockwise)
	if len(got) != len(wantOrder) {
		t.Fatalf("LoopOrder len = %d, want %d", len(got), len(wantOrder))
	}
	for i := range wantOrder {
		if got[i] != wantOrder[i] {
			t.Errorf("LoopOrder[%d] = %v, want %v", i, got[i], wantOrder[i])
		}
	}

	// The first waypoint should sit on the starting (South) corridor's
	// centerline, since BuildWaypointSequence begins at the nearest point of
	// the first segment to the spawn. South centerline y = south_width/2 +
	// south_bias = 1.0 + 0.10 = 1.10.
	if math.Abs(wps[0].Y-1.10) > 0.05 {
		t.Errorf("first waypoint y = %.3f, want ~1.10 (South corridor centerline)", wps[0].Y)
	}
}

func TestCalculateWaypoints_RejectsUnresolvedDirection(t *testing.T) {
	cfg := DefaultConfig()
	input := benchInput(4.0, 0.30)
	input.Starting.Direction = nil

	if _, err := CalculateWaypoints(input, 1, cfg, nil, AllConfirmed()); err == nil {
		t.Fatal("CalculateWaypoints() with nil Direction = nil error, want error")
	}
}

func TestCalculateWaypoints_RejectsTooNarrowCorridor(t *testing.T) {
	cfg := DefaultConfig()
	cfg.NarrowWidthThresholdM = 0.8
	// 0.5 m corridors with a 0.10 m wide bias need 0.30+2*0.10 = 0.50 m; add a
	// wide bias of 0.10 and the chassis (0.30) requires 0.50 m exactly -- nudge
	// the chassis up so it fails.
	geom := trackmodel.CorridorGeometryFromWidths(map[trackmodel.Section]float64{
		trackmodel.North: 0.5,
		trackmodel.South: 0.5,
		trackmodel.East:  0.5,
		trackmodel.West:  0.5,
	}, 3.0)
	dir := trackmodel.Clockwise
	input := PlannerInput{
		Geometry:      geom,
		Starting:      StartingConditions{Section: trackmodel.South, Position: trackmodel.Waypoint{X: 0.25, Y: 0.25}},
		MaxCoordM:     3.0,
		ChassisWidthM: 0.6,
	}
	input.Starting.Direction = &dir
	if _, err := CalculateWaypoints(input, 1, cfg, nil, AllConfirmed()); err == nil {
		t.Fatal("CalculateWaypoints() with infeasible corridor = nil error, want error")
	}
}

func TestPlanBelievedPath_ReplansFromBelievedGeometry(t *testing.T) {
	cfg := DefaultConfig()
	cfg.CornerArcAssumeWide = false
	base := benchInput(4.0, 0.30)
	dir := trackmodel.Counterclockwise
	base.Starting.Direction = &dir
	believed := trackmodel.CorridorGeometryFromWidths(map[trackmodel.Section]float64{
		trackmodel.North: 1.0,
		trackmodel.South: 1.0,
		trackmodel.East:  1.0,
		trackmodel.West:  1.0,
	}, 4.0)

	wps, err := PlanBelievedPath(
		base, believed,
		StartingConditions{
			Direction: &dir,
			Section:   trackmodel.East,
			Position:  trackmodel.Waypoint{X: 2.0, Y: 0.5},
			Yaw:       0.0,
		},
		cfg, nil, AllConfirmed(),
	)
	if err != nil {
		t.Fatalf("PlanBelievedPath() error = %v", err)
	}
	if len(wps) < 20 {
		t.Fatalf("len(waypoints) = %d, want > 20", len(wps))
	}
}

func TestLoopOrder_AnchoredAtEast(t *testing.T) {
	// CW absolute = [East, South, West, North]; CCW = [East, North, West, South].
	cw := trackmodel.LoopOrder(trackmodel.East, trackmodel.Clockwise)
	if cw[0] != trackmodel.East || cw[1] != trackmodel.South || cw[2] != trackmodel.West || cw[3] != trackmodel.North {
		t.Errorf("CW loop order = %v, want [East South West North]", cw)
	}
	ccw := trackmodel.LoopOrder(trackmodel.East, trackmodel.Counterclockwise)
	if ccw[0] != trackmodel.East || ccw[1] != trackmodel.North || ccw[2] != trackmodel.West || ccw[3] != trackmodel.South {
		t.Errorf("CCW loop order = %v, want [East North West South]", ccw)
	}
}
