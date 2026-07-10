// Package memory is an in-memory Store for the Navigation bounded context.
// Waypoint/route state is real (kept in memory, real geometry for route
// distance/duration); live status and clearance are zero-valued since no
// navigation-node feed is wired into this backend yet.
package memory

import (
	"context"
	"math"
	"sync"
	"time"

	"github.com/google/uuid"

	"github.com/teamvoldemor/voldemorbot/platform/backend/domain/navigation"
)

// assumedAvgSpeedMPS is a conservative average-speed estimate used to derive
// Route.EstimatedDurationS from planned distance until a real planner/executor
// feed is wired in.
const assumedAvgSpeedMPS = 1.0

// Memory is a thread-safe in-memory Store for the Navigation bounded context.
type Memory struct {
	mu        sync.RWMutex
	waypoints []navigation.Waypoint
	route     navigation.Route
	tuning    navigation.Tuning
}

// NewMemory returns an empty Memory store.
func NewMemory() *Memory {
	return &Memory{}
}

func (m *Memory) ListWaypoints(_ context.Context) ([]navigation.Waypoint, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	out := make([]navigation.Waypoint, len(m.waypoints))
	copy(out, m.waypoints)
	return out, nil
}

func (m *Memory) CreateWaypoint(_ context.Context, req navigation.CreateWaypointRequest) (navigation.Waypoint, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	wp := navigation.Waypoint{
		ID:        uuid.NewString(),
		X:         req.X,
		Y:         req.Y,
		Index:     len(m.waypoints),
		Tolerance: req.Tolerance,
		Section:   req.Section,
	}
	m.waypoints = append(m.waypoints, wp)
	return wp, nil
}

func (m *Memory) DeleteWaypoint(_ context.Context, id string) error {
	m.mu.Lock()
	defer m.mu.Unlock()
	for i, wp := range m.waypoints {
		if wp.ID == id {
			m.waypoints = append(m.waypoints[:i], m.waypoints[i+1:]...)
			reindex(m.waypoints)
			return nil
		}
	}
	return navigation.ErrNotFound
}

func (m *Memory) Route(_ context.Context) (navigation.Route, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.route, nil
}

// PlanRoute builds a route from the currently stored waypoints. Distance is
// the real polyline length through those waypoints; duration is derived from
// an assumed average speed until a real planner/executor feed exists.
func (m *Memory) PlanRoute(_ context.Context, req navigation.PlanRouteRequest) (navigation.Route, error) {
	m.mu.Lock()
	defer m.mu.Unlock()

	laps := 1
	if req.Laps != nil {
		laps = *req.Laps
	}

	dist := polylineLength(req.StartX, req.StartY, m.waypoints) * float64(laps)
	now := time.Now().UTC()
	m.route = navigation.Route{
		ID:                 uuid.NewString(),
		Waypoints:          append([]navigation.Waypoint(nil), m.waypoints...),
		TotalDistance:      dist,
		EstimatedDurationS: dist / assumedAvgSpeedMPS,
		Laps:               &laps,
		CreatedAt:          &now,
	}
	return m.route, nil
}

func (m *Memory) Status(_ context.Context) (navigation.Status, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return navigation.Status{
		TotalWaypoints: len(m.waypoints),
	}, nil
}

func (m *Memory) Clearance(_ context.Context) (navigation.Clearance, error) {
	return navigation.Clearance{Timestamp: time.Now().UTC()}, nil
}

func (m *Memory) Tuning(_ context.Context) (navigation.Tuning, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.tuning, nil
}

func (m *Memory) UpdateTuning(_ context.Context, t navigation.Tuning) (navigation.Tuning, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.tuning = t
	return m.tuning, nil
}

func reindex(wps []navigation.Waypoint) {
	for i := range wps {
		wps[i].Index = i
	}
}

func polylineLength(startX, startY float64, wps []navigation.Waypoint) float64 {
	if len(wps) == 0 {
		return 0
	}
	total := math.Hypot(wps[0].X-startX, wps[0].Y-startY)
	for i := 1; i < len(wps); i++ {
		total += math.Hypot(wps[i].X-wps[i-1].X, wps[i].Y-wps[i-1].Y)
	}
	return total
}
