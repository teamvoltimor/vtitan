package memory

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/apps/backend/domain/navigation"
)

const epsilon = 1e-9

func TestCreateWaypointAssignsSequentialIndex(t *testing.T) {
	m := NewMemory()
	ctx := context.Background()

	a, err := m.CreateWaypoint(ctx, navigation.CreateWaypointRequest{X: 1, Y: 1})
	if err != nil {
		t.Fatalf("CreateWaypoint a: %v", err)
	}
	b, err := m.CreateWaypoint(ctx, navigation.CreateWaypointRequest{X: 2, Y: 2})
	if err != nil {
		t.Fatalf("CreateWaypoint b: %v", err)
	}
	if a.Index != 0 || b.Index != 1 {
		t.Fatalf("indices = %d, %d, want 0, 1", a.Index, b.Index)
	}
}

func TestDeleteWaypointReindexes(t *testing.T) {
	m := NewMemory()
	ctx := context.Background()

	a, _ := m.CreateWaypoint(ctx, navigation.CreateWaypointRequest{X: 0, Y: 0})
	_, _ = m.CreateWaypoint(ctx, navigation.CreateWaypointRequest{X: 1, Y: 1})
	c, _ := m.CreateWaypoint(ctx, navigation.CreateWaypointRequest{X: 2, Y: 2})

	if err := m.DeleteWaypoint(ctx, a.ID); err != nil {
		t.Fatalf("DeleteWaypoint: %v", err)
	}

	wps, err := m.ListWaypoints(ctx)
	if err != nil {
		t.Fatalf("ListWaypoints: %v", err)
	}
	if len(wps) != 2 {
		t.Fatalf("len(wps) = %d, want 2", len(wps))
	}
	// c was the third waypoint (index 2); after deleting the first, it must
	// be reindexed to 1, not left with a stale index.
	for _, wp := range wps {
		if wp.ID == c.ID && wp.Index != 1 {
			t.Fatalf("surviving waypoint index = %d, want 1", wp.Index)
		}
	}
}

func TestDeleteWaypointNotFound(t *testing.T) {
	m := NewMemory()
	if err := m.DeleteWaypoint(context.Background(), "missing"); !errors.Is(err, navigation.ErrNotFound) {
		t.Fatalf("want ErrNotFound, got %v", err)
	}
}

func TestPlanRouteComputesRealDistance(t *testing.T) {
	m := NewMemory()
	ctx := context.Background()

	if _, err := m.CreateWaypoint(ctx, navigation.CreateWaypointRequest{X: 3, Y: 4}); err != nil {
		t.Fatalf("CreateWaypoint: %v", err)
	}

	route, err := m.PlanRoute(ctx, navigation.PlanRouteRequest{StartX: 0, StartY: 0})
	if err != nil {
		t.Fatalf("PlanRoute: %v", err)
	}
	// (0,0) -> (3,4) is a 3-4-5 triangle: distance 5.
	if math.Abs(route.TotalDistance-5.0) > epsilon {
		t.Fatalf("TotalDistance = %v, want 5.0", route.TotalDistance)
	}
	if math.Abs(route.EstimatedDurationS-route.TotalDistance/assumedAvgSpeedMPS) > epsilon {
		t.Fatalf("EstimatedDurationS = %v, inconsistent with TotalDistance %v", route.EstimatedDurationS, route.TotalDistance)
	}
}

func TestPlanRouteMultipliesByLaps(t *testing.T) {
	m := NewMemory()
	ctx := context.Background()

	if _, err := m.CreateWaypoint(ctx, navigation.CreateWaypointRequest{X: 3, Y: 4}); err != nil {
		t.Fatalf("CreateWaypoint: %v", err)
	}

	laps := 3
	route, err := m.PlanRoute(ctx, navigation.PlanRouteRequest{StartX: 0, StartY: 0, Laps: &laps})
	if err != nil {
		t.Fatalf("PlanRoute: %v", err)
	}
	want := 5.0 * float64(laps)
	if math.Abs(route.TotalDistance-want) > epsilon {
		t.Fatalf("TotalDistance = %v, want %v", route.TotalDistance, want)
	}
	if route.Laps == nil || *route.Laps != laps {
		t.Fatalf("Laps = %v, want %d", route.Laps, laps)
	}
}

func TestUpdateTuningRoundTrips(t *testing.T) {
	m := NewMemory()
	ctx := context.Background()

	kp := 1.5
	got, err := m.UpdateTuning(ctx, navigation.Tuning{SteerKp: &kp})
	if err != nil {
		t.Fatalf("UpdateTuning: %v", err)
	}
	if got.SteerKp == nil || *got.SteerKp != kp {
		t.Fatalf("SteerKp = %v, want %v", got.SteerKp, kp)
	}

	fetched, err := m.Tuning(ctx)
	if err != nil {
		t.Fatalf("Tuning: %v", err)
	}
	if fetched.SteerKp == nil || *fetched.SteerKp != kp {
		t.Fatalf("Tuning() SteerKp = %v, want %v", fetched.SteerKp, kp)
	}
}

func TestStatusReflectsWaypointCount(t *testing.T) {
	m := NewMemory()
	ctx := context.Background()

	if _, err := m.CreateWaypoint(ctx, navigation.CreateWaypointRequest{X: 0, Y: 0}); err != nil {
		t.Fatalf("CreateWaypoint: %v", err)
	}
	if _, err := m.CreateWaypoint(ctx, navigation.CreateWaypointRequest{X: 1, Y: 1}); err != nil {
		t.Fatalf("CreateWaypoint: %v", err)
	}

	st, err := m.Status(ctx)
	if err != nil {
		t.Fatalf("Status: %v", err)
	}
	if st.TotalWaypoints != 2 {
		t.Fatalf("TotalWaypoints = %d, want 2", st.TotalWaypoints)
	}
}
