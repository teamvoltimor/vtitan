// Package memory is an in-memory Store for the Robot bounded context. There is
// no persistent robot registry or live hardware feed yet — this keeps the REST
// contract fully functional (real CRUD, real validation) while being honest
// that state does not survive a restart and status/telemetry fields are
// zero-valued until a real data source is wired in.
package memory

import (
	"context"
	"sync"
	"time"

	"github.com/google/uuid"

	"github.com/teamvoltimor/vtitan/platform/backend/domain/robot"
)

// Memory is a thread-safe in-memory Store for the Robot bounded context.
type Memory struct {
	mu      sync.RWMutex
	robots  map[string]robot.Robot
	configs map[string]robot.Config
}

// NewMemory returns an empty Memory store.
func NewMemory() *Memory {
	return &Memory{
		robots:  make(map[string]robot.Robot),
		configs: make(map[string]robot.Config),
	}
}

// List returns all robots, optionally filtered by fleet ID and/or state.
func (m *Memory) List(_ context.Context, fleetID string, state robot.State) ([]robot.Robot, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()

	out := make([]robot.Robot, 0, len(m.robots))
	for _, r := range m.robots {
		if fleetID != "" && r.FleetID != fleetID {
			continue
		}
		if state != "" && r.State != state {
			continue
		}
		out = append(out, r)
	}
	return out, nil
}

// Create registers a new robot in state BOOT_CHECK, with an empty config.
func (m *Memory) Create(_ context.Context, req robot.CreateRequest) (robot.Robot, error) {
	now := time.Now().UTC()
	r := robot.Robot{
		ID:        uuid.NewString(),
		Name:      req.Name,
		FleetID:   req.FleetID,
		State:     robot.StateBootCheck,
		CreatedAt: now,
		UpdatedAt: now,
	}

	m.mu.Lock()
	m.robots[r.ID] = r
	m.configs[r.ID] = robot.Config{}
	m.mu.Unlock()

	return r, nil
}

// Get returns the robot with the given id.
func (m *Memory) Get(_ context.Context, id string) (robot.Robot, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	r, ok := m.robots[id]
	if !ok {
		return robot.Robot{}, robot.ErrNotFound
	}
	return r, nil
}

// Update applies non-nil fields from req to the robot with the given id.
func (m *Memory) Update(_ context.Context, id string, req robot.UpdateRequest) (robot.Robot, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	r, ok := m.robots[id]
	if !ok {
		return robot.Robot{}, robot.ErrNotFound
	}
	if req.Name != nil {
		r.Name = *req.Name
	}
	r.UpdatedAt = time.Now().UTC()
	m.robots[id] = r
	return r, nil
}

// Delete removes the robot and its config.
func (m *Memory) Delete(_ context.Context, id string) error {
	m.mu.Lock()
	defer m.mu.Unlock()
	if _, ok := m.robots[id]; !ok {
		return robot.ErrNotFound
	}
	delete(m.robots, id)
	delete(m.configs, id)
	return nil
}

// Status returns a live-shaped status derived from the stored robot record.
// Pose/velocity/system fields are zero-valued since no telemetry feed is
// wired to individual fleet robots yet.
func (m *Memory) Status(_ context.Context, id string) (robot.Status, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	r, ok := m.robots[id]
	if !ok {
		return robot.Status{}, robot.ErrNotFound
	}
	return robot.Status{
		RobotID:   r.ID,
		State:     r.State,
		Timestamp: time.Now().UTC(),
	}, nil
}

// Config returns the stored tunable config for the robot with the given id.
func (m *Memory) Config(_ context.Context, id string) (robot.Config, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	if _, ok := m.robots[id]; !ok {
		return robot.Config{}, robot.ErrNotFound
	}
	return m.configs[id], nil
}

// UpdateConfig applies non-nil fields from req to the robot's stored config.
func (m *Memory) UpdateConfig(_ context.Context, id string, req robot.UpdateConfigRequest) (robot.Config, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if _, ok := m.robots[id]; !ok {
		return robot.Config{}, robot.ErrNotFound
	}
	cfg := m.configs[id]
	if req.MaxLinearSpeed != nil {
		cfg.MaxLinearSpeed = req.MaxLinearSpeed
	}
	if req.MaxAngularSpeed != nil {
		cfg.MaxAngularSpeed = req.MaxAngularSpeed
	}
	if req.SteeringKp != nil {
		cfg.SteeringKp = req.SteeringKp
	}
	if req.LookaheadDistance != nil {
		cfg.LookaheadDistance = req.LookaheadDistance
	}
	if req.SpeedProfile != nil {
		cfg.SpeedProfiles = append(cfg.SpeedProfiles, *req.SpeedProfile)
	}
	m.configs[id] = cfg
	return cfg, nil
}

// Command records that a command was received for id. No hardware/ROS path is
// wired up yet (matching the rest of this service's current integration
// state), so every command for a known robot is accepted but not physically
// executed.
func (m *Memory) Command(_ context.Context, id string, cmd robot.Command) (robot.CommandResult, error) {
	m.mu.RLock()
	_, ok := m.robots[id]
	m.mu.RUnlock()
	if !ok {
		return robot.CommandResult{}, robot.ErrNotFound
	}
	return robot.CommandResult{
		CommandID: uuid.NewString(),
		Status:    robot.CommandAccepted,
		Message:   "accepted; not yet forwarded to hardware (no ROS command path wired up)",
	}, nil
}
