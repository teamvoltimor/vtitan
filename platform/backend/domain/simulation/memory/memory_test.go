package memory

import (
	"context"
	"errors"
	"testing"

	"github.com/teamvoldemor/voldemorbot/platform/backend/domain/simulation"
)

func TestGenerateScenarioSetsDetectionCountOnlyForObstacles(t *testing.T) {
	m := NewMemory()
	ctx := context.Background()

	n := 8
	obstacles, err := m.GenerateScenario(ctx, simulation.GenerateScenarioRequest{Challenge: simulation.ChallengeObstacles, NumSigns: &n})
	if err != nil {
		t.Fatalf("GenerateScenario (obstacles): %v", err)
	}
	if obstacles.DetectionCount == nil || *obstacles.DetectionCount != n {
		t.Fatalf("DetectionCount = %v, want %d", obstacles.DetectionCount, n)
	}

	open, err := m.GenerateScenario(ctx, simulation.GenerateScenarioRequest{Challenge: simulation.ChallengeOpen})
	if err != nil {
		t.Fatalf("GenerateScenario (open): %v", err)
	}
	if open.DetectionCount != nil {
		t.Fatalf("DetectionCount = %v, want nil for an open-challenge scenario", open.DetectionCount)
	}
}

func TestListScenariosFiltersByChallengeAndLimit(t *testing.T) {
	m := NewMemory()
	ctx := context.Background()

	for range 3 {
		if _, err := m.GenerateScenario(ctx, simulation.GenerateScenarioRequest{Challenge: simulation.ChallengeObstacles}); err != nil {
			t.Fatalf("GenerateScenario: %v", err)
		}
	}
	if _, err := m.GenerateScenario(ctx, simulation.GenerateScenarioRequest{Challenge: simulation.ChallengeOpen}); err != nil {
		t.Fatalf("GenerateScenario: %v", err)
	}

	obstaclesOnly, err := m.ListScenarios(ctx, simulation.ChallengeObstacles, 0)
	if err != nil {
		t.Fatalf("ListScenarios: %v", err)
	}
	if len(obstaclesOnly) != 3 {
		t.Fatalf("len(obstaclesOnly) = %d, want 3", len(obstaclesOnly))
	}

	limited, err := m.ListScenarios(ctx, "", 2)
	if err != nil {
		t.Fatalf("ListScenarios (limited): %v", err)
	}
	if len(limited) != 2 {
		t.Fatalf("len(limited) = %d, want 2", len(limited))
	}
}

func TestGetAndDeleteScenario(t *testing.T) {
	m := NewMemory()
	ctx := context.Background()

	sc, err := m.GenerateScenario(ctx, simulation.GenerateScenarioRequest{Challenge: simulation.ChallengeOpen})
	if err != nil {
		t.Fatalf("GenerateScenario: %v", err)
	}

	if _, err := m.GetScenario(ctx, sc.ID); err != nil {
		t.Fatalf("GetScenario: %v", err)
	}
	if err := m.DeleteScenario(ctx, sc.ID); err != nil {
		t.Fatalf("DeleteScenario: %v", err)
	}
	if _, err := m.GetScenario(ctx, sc.ID); !errors.Is(err, simulation.ErrNotFound) {
		t.Fatalf("GetScenario after delete: want ErrNotFound, got %v", err)
	}
}

func TestStartRunRequiresExistingScenario(t *testing.T) {
	m := NewMemory()
	if _, err := m.StartRun(context.Background(), simulation.StartRunRequest{ScenarioID: "missing"}); !errors.Is(err, simulation.ErrNotFound) {
		t.Fatalf("want ErrNotFound, got %v", err)
	}
}

func TestControlRunStateTransitions(t *testing.T) {
	m := NewMemory()
	ctx := context.Background()

	sc, err := m.GenerateScenario(ctx, simulation.GenerateScenarioRequest{Challenge: simulation.ChallengeOpen})
	if err != nil {
		t.Fatalf("GenerateScenario: %v", err)
	}
	run, err := m.StartRun(ctx, simulation.StartRunRequest{ScenarioID: sc.ID})
	if err != nil {
		t.Fatalf("StartRun: %v", err)
	}
	if run.Status != simulation.RunPending {
		t.Fatalf("initial Status = %q, want %q", run.Status, simulation.RunPending)
	}

	cases := []struct {
		action     simulation.RunAction
		wantStatus simulation.RunStatus
	}{
		{simulation.RunActionResume, simulation.RunRunning},
		{simulation.RunActionPause, simulation.RunPaused},
		{simulation.RunActionResume, simulation.RunRunning},
		{simulation.RunActionStop, simulation.RunCancelled},
	}
	for _, tc := range cases {
		run, err = m.ControlRun(ctx, run.ID, tc.action)
		if err != nil {
			t.Fatalf("ControlRun(%s): %v", tc.action, err)
		}
		if run.Status != tc.wantStatus {
			t.Fatalf("ControlRun(%s) status = %q, want %q", tc.action, run.Status, tc.wantStatus)
		}
	}
	if run.CompletedAt == nil {
		t.Fatal("CompletedAt not set after stop")
	}
}

func TestControlRunNotFound(t *testing.T) {
	m := NewMemory()
	if _, err := m.ControlRun(context.Background(), "missing", simulation.RunActionPause); !errors.Is(err, simulation.ErrNotFound) {
		t.Fatalf("want ErrNotFound, got %v", err)
	}
}

func TestListEnvironmentsReturnsDistinctIDs(t *testing.T) {
	m := NewMemory()
	envs, err := m.ListEnvironments(context.Background())
	if err != nil {
		t.Fatalf("ListEnvironments: %v", err)
	}
	if len(envs) != 2 {
		t.Fatalf("len(envs) = %d, want 2", len(envs))
	}
	if envs[0].ID == envs[1].ID {
		t.Fatalf("environment IDs collide: both are %q", envs[0].ID)
	}
}
