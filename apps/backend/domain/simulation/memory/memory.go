// Package memory is an in-memory Store for the Simulation bounded context.
// Scenario generation produces real, request-driven random geometry (not a
// bridge to the Python/Gazebo generator under platform/robot/src/simulation --
// no such bridge exists yet); run lifecycle/control is real state-machine
// logic with no actual Gazebo process behind it until one is wired in.
package memory

import (
	"context"
	"math/rand/v2"
	"sync"
	"time"

	"github.com/google/uuid"

	"github.com/teamvoltimor/vtitan/platform/backend/domain/simulation"
)

const (
	defaultListLimit  = 20
	minTrackWidthM    = 1.2
	maxTrackWidthMAdd = 0.6
	minNumSigns       = 6
	maxNumSignsSpan   = 8
)

// Memory is a thread-safe in-memory Store for the Simulation bounded context.
type Memory struct {
	mu        sync.RWMutex
	scenarios map[string]simulation.Scenario
	runs      map[string]simulation.Run
}

// NewMemory returns a Memory store seeded with the two known WRO environments.
func NewMemory() *Memory {
	return &Memory{
		scenarios: make(map[string]simulation.Scenario),
		runs:      make(map[string]simulation.Run),
	}
}

// ListScenarios returns up to limit stored scenarios, optionally filtered by challenge.
func (m *Memory) ListScenarios(_ context.Context, challenge simulation.Challenge, limit int) ([]simulation.Scenario, error) {
	if limit <= 0 {
		limit = defaultListLimit
	}
	m.mu.RLock()
	defer m.mu.RUnlock()

	out := make([]simulation.Scenario, 0, len(m.scenarios))
	for _, sc := range m.scenarios {
		if challenge != "" && sc.Challenge != challenge {
			continue
		}
		out = append(out, sc)
		if len(out) >= limit {
			break
		}
	}
	return out, nil
}

// GenerateScenario produces a scenario with real, request-driven randomized
// geometry. It does not call into the Python/Gazebo scenario generator under
// platform/robot/src/simulation -- no such bridge exists from this Go service.
func (m *Memory) GenerateScenario(_ context.Context, req simulation.GenerateScenarioRequest) (simulation.Scenario, error) {
	numSigns := minNumSigns + rand.IntN(maxNumSignsSpan)
	if req.NumSigns != nil {
		numSigns = *req.NumSigns
	}

	trackWidth := minTrackWidthM + rand.Float64()*maxTrackWidthMAdd
	sc := simulation.Scenario{
		ID:        uuid.NewString(),
		Name:      string(req.Challenge) + "-" + uuid.NewString()[:8],
		Challenge: req.Challenge,
		CreatedAt: time.Now().UTC(),
		TrackConfig: &simulation.TrackConfig{
			TrackWidth: &trackWidth,
			Sections:   []string{"start", "corridor_1", "corridor_2", "finish"},
		},
	}
	if req.Challenge == simulation.ChallengeObstacles {
		n := numSigns
		sc.DetectionCount = &n
	}
	if req.Lighting != nil {
		lighting := string(*req.Lighting)
		sc.Lighting = &simulation.LightingConfig{Scenario: &lighting}
	}

	m.mu.Lock()
	m.scenarios[sc.ID] = sc
	m.mu.Unlock()
	return sc, nil
}

// GetScenario returns the scenario with the given id.
func (m *Memory) GetScenario(_ context.Context, id string) (simulation.Scenario, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	sc, ok := m.scenarios[id]
	if !ok {
		return simulation.Scenario{}, simulation.ErrNotFound
	}
	return sc, nil
}

// DeleteScenario removes the scenario with the given id.
func (m *Memory) DeleteScenario(_ context.Context, id string) error {
	m.mu.Lock()
	defer m.mu.Unlock()
	if _, ok := m.scenarios[id]; !ok {
		return simulation.ErrNotFound
	}
	delete(m.scenarios, id)
	return nil
}

// ListRuns returns all stored simulation runs.
func (m *Memory) ListRuns(_ context.Context) ([]simulation.Run, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	out := make([]simulation.Run, 0, len(m.runs))
	for _, r := range m.runs {
		out = append(out, r)
	}
	return out, nil
}

// StartRun creates a new pending run for the given scenario.
func (m *Memory) StartRun(_ context.Context, req simulation.StartRunRequest) (simulation.Run, error) {
	m.mu.RLock()
	_, scenarioExists := m.scenarios[req.ScenarioID]
	m.mu.RUnlock()
	if !scenarioExists {
		return simulation.Run{}, simulation.ErrNotFound
	}

	laps := 1
	if req.Laps != nil {
		laps = *req.Laps
	}
	run := simulation.Run{
		ID:         uuid.NewString(),
		ScenarioID: req.ScenarioID,
		Status:     simulation.RunPending,
		StartedAt:  time.Now().UTC(),
		TotalLaps:  &laps,
	}
	m.mu.Lock()
	m.runs[run.ID] = run
	m.mu.Unlock()
	return run, nil
}

// GetRun returns the run with the given id.
func (m *Memory) GetRun(_ context.Context, id string) (simulation.Run, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	r, ok := m.runs[id]
	if !ok {
		return simulation.Run{}, simulation.ErrNotFound
	}
	return r, nil
}

// ControlRun applies a real (if Gazebo-free) run state transition.
func (m *Memory) ControlRun(_ context.Context, id string, action simulation.RunAction) (simulation.Run, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	r, ok := m.runs[id]
	if !ok {
		return simulation.Run{}, simulation.ErrNotFound
	}

	switch action {
	case simulation.RunActionPause:
		r.Status = simulation.RunPaused
	case simulation.RunActionResume:
		r.Status = simulation.RunRunning
	case simulation.RunActionStop:
		r.Status = simulation.RunCancelled
		now := time.Now().UTC()
		r.CompletedAt = &now
	case simulation.RunActionRestart:
		r.Status = simulation.RunPending
		r.CompletedAt = nil
		r.StartedAt = time.Now().UTC()
	}
	m.runs[id] = r
	return r, nil
}

// Fixed, stable IDs for the two seeded environments below (must be valid
// UUIDs -- the wire schema requires format: uuid -- so they stay distinct
// and reproducible across restarts instead of colliding on uuid.Nil).
const (
	openEnvironmentID      = "5b7f6a6e-1b0e-4c6e-9a0a-6f1b7b9e0a01"
	obstaclesEnvironmentID = "5b7f6a6e-1b0e-4c6e-9a0a-6f1b7b9e0a02"
)

// ListEnvironments returns the environments known to this service. There is
// no real Gazebo world catalog wired in yet, so this reflects the two known
// WRO challenge types with placeholder world files.
func (m *Memory) ListEnvironments(_ context.Context) ([]simulation.Environment, error) {
	return []simulation.Environment{
		{ID: openEnvironmentID, Name: "Open Challenge", WorldFile: "open_challenge.world"},
		{ID: obstaclesEnvironmentID, Name: "Obstacles Challenge", WorldFile: "obstacles_challenge.world"},
	}, nil
}
