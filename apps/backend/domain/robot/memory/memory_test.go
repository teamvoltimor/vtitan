package memory

import (
	"context"
	"errors"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/backend/domain/robot"
)

func TestCreateAndGet(t *testing.T) {
	m := NewMemory()
	ctx := context.Background()

	r, err := m.Create(ctx, robot.CreateRequest{Name: "vtitan", FleetID: "fleet-1"})
	if err != nil {
		t.Fatalf("Create: %v", err)
	}
	if r.State != robot.StateBootCheck {
		t.Fatalf("want initial state %q, got %q", robot.StateBootCheck, r.State)
	}

	got, err := m.Get(ctx, r.ID)
	if err != nil {
		t.Fatalf("Get: %v", err)
	}
	if got != r {
		t.Fatalf("Get returned %+v, want %+v", got, r)
	}
}

func TestGetNotFound(t *testing.T) {
	m := NewMemory()
	if _, err := m.Get(context.Background(), "missing"); !errors.Is(err, robot.ErrNotFound) {
		t.Fatalf("want ErrNotFound, got %v", err)
	}
}

func TestListFiltersByFleetAndState(t *testing.T) {
	m := NewMemory()
	ctx := context.Background()

	a, err := m.Create(ctx, robot.CreateRequest{Name: "a", FleetID: "fleet-1"})
	if err != nil {
		t.Fatalf("Create a: %v", err)
	}
	if _, err := m.Create(ctx, robot.CreateRequest{Name: "b", FleetID: "fleet-2"}); err != nil {
		t.Fatalf("Create b: %v", err)
	}

	byFleet, err := m.List(ctx, "fleet-1", "")
	if err != nil {
		t.Fatalf("List by fleet: %v", err)
	}
	if len(byFleet) != 1 || byFleet[0].ID != a.ID {
		t.Fatalf("List by fleet-1 = %+v, want only %q", byFleet, a.ID)
	}

	byState, err := m.List(ctx, "", robot.StateReady)
	if err != nil {
		t.Fatalf("List by state: %v", err)
	}
	if len(byState) != 0 {
		t.Fatalf("List by READY state = %+v, want empty (both robots are BOOT_CHECK)", byState)
	}
}

func TestUpdateAndDelete(t *testing.T) {
	m := NewMemory()
	ctx := context.Background()

	r, err := m.Create(ctx, robot.CreateRequest{Name: "old-name", FleetID: "fleet-1"})
	if err != nil {
		t.Fatalf("Create: %v", err)
	}

	newName := "new-name"
	updated, err := m.Update(ctx, r.ID, robot.UpdateRequest{Name: &newName})
	if err != nil {
		t.Fatalf("Update: %v", err)
	}
	if updated.Name != newName {
		t.Fatalf("Update name = %q, want %q", updated.Name, newName)
	}
	if !updated.UpdatedAt.After(r.UpdatedAt) && updated.UpdatedAt != r.UpdatedAt {
		t.Fatalf("UpdatedAt did not advance: before=%v after=%v", r.UpdatedAt, updated.UpdatedAt)
	}

	if err := m.Delete(ctx, r.ID); err != nil {
		t.Fatalf("Delete: %v", err)
	}
	if _, err := m.Get(ctx, r.ID); !errors.Is(err, robot.ErrNotFound) {
		t.Fatalf("Get after Delete: want ErrNotFound, got %v", err)
	}
	if err := m.Delete(ctx, r.ID); !errors.Is(err, robot.ErrNotFound) {
		t.Fatalf("double Delete: want ErrNotFound, got %v", err)
	}
}

func TestUpdateConfigMergesFields(t *testing.T) {
	m := NewMemory()
	ctx := context.Background()

	r, err := m.Create(ctx, robot.CreateRequest{Name: "r", FleetID: "f"})
	if err != nil {
		t.Fatalf("Create: %v", err)
	}

	speed := 2.5
	cfg, err := m.UpdateConfig(ctx, r.ID, robot.UpdateConfigRequest{MaxLinearSpeed: &speed})
	if err != nil {
		t.Fatalf("UpdateConfig (speed): %v", err)
	}
	if cfg.MaxLinearSpeed == nil || *cfg.MaxLinearSpeed != speed {
		t.Fatalf("MaxLinearSpeed = %v, want %v", cfg.MaxLinearSpeed, speed)
	}

	kp := 1.2
	cfg, err = m.UpdateConfig(ctx, r.ID, robot.UpdateConfigRequest{SteeringKp: &kp})
	if err != nil {
		t.Fatalf("UpdateConfig (kp): %v", err)
	}
	// The earlier field must survive an update that only touches a different field.
	if cfg.MaxLinearSpeed == nil || *cfg.MaxLinearSpeed != speed {
		t.Fatalf("MaxLinearSpeed was clobbered: got %v, want %v", cfg.MaxLinearSpeed, speed)
	}
	if cfg.SteeringKp == nil || *cfg.SteeringKp != kp {
		t.Fatalf("SteeringKp = %v, want %v", cfg.SteeringKp, kp)
	}

	profile := "aggressive"
	cfg, err = m.UpdateConfig(ctx, r.ID, robot.UpdateConfigRequest{SpeedProfile: &profile})
	if err != nil {
		t.Fatalf("UpdateConfig (profile): %v", err)
	}
	if len(cfg.SpeedProfiles) != 1 || cfg.SpeedProfiles[0] != profile {
		t.Fatalf("SpeedProfiles = %v, want [%q]", cfg.SpeedProfiles, profile)
	}
}

func TestUpdateConfigNotFound(t *testing.T) {
	m := NewMemory()
	if _, err := m.UpdateConfig(context.Background(), "missing", robot.UpdateConfigRequest{}); !errors.Is(err, robot.ErrNotFound) {
		t.Fatalf("want ErrNotFound, got %v", err)
	}
}

func TestCommandAcceptedForKnownRobot(t *testing.T) {
	m := NewMemory()
	ctx := context.Background()

	r, err := m.Create(ctx, robot.CreateRequest{Name: "r", FleetID: "f"})
	if err != nil {
		t.Fatalf("Create: %v", err)
	}

	res, err := m.Command(ctx, r.ID, robot.Command{Type: robot.CommandEmergencyStop})
	if err != nil {
		t.Fatalf("Command: %v", err)
	}
	if res.Status != robot.CommandAccepted {
		t.Fatalf("Status = %q, want %q", res.Status, robot.CommandAccepted)
	}
	if res.CommandID == "" {
		t.Fatal("CommandID is empty")
	}
}

func TestCommandNotFound(t *testing.T) {
	m := NewMemory()
	if _, err := m.Command(context.Background(), "missing", robot.Command{Type: robot.CommandPause}); !errors.Is(err, robot.ErrNotFound) {
		t.Fatalf("want ErrNotFound, got %v", err)
	}
}
