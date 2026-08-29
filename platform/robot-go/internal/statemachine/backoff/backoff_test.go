package backoff_test

import (
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/statemachine/backoff"
)

func TestBackoff_NextDoublesUpToMax(t *testing.T) {
	t.Parallel()

	cfg := backoff.Config{Initial: 1 * time.Second, Max: 8 * time.Second}
	b := backoff.New(cfg)

	want := []time.Duration{
		1 * time.Second,
		2 * time.Second,
		4 * time.Second,
		8 * time.Second,
		8 * time.Second, // capped
		8 * time.Second, // still capped
	}

	for i, w := range want {
		if got := b.Next(); got != w {
			t.Fatalf("Next() call %d = %v, want %v", i, got, w)
		}
	}
}

func TestBackoff_ResetRestoresInitial(t *testing.T) {
	t.Parallel()

	cfg := backoff.Config{Initial: 1 * time.Second, Max: 60 * time.Second}
	b := backoff.New(cfg)

	b.Next()
	b.Next()
	b.Next() // current is now 8s

	b.Reset()

	if got := b.Next(); got != cfg.Initial {
		t.Fatalf("Next() after Reset() = %v, want %v", got, cfg.Initial)
	}
}

func TestDefaultConfig_MatchesGRPCBackoffConstants(t *testing.T) {
	t.Parallel()

	cfg := backoff.DefaultConfig()

	if cfg.Initial != 1*time.Second {
		t.Errorf("DefaultConfig().Initial = %v, want 1s", cfg.Initial)
	}
	if cfg.Max != 60*time.Second {
		t.Errorf("DefaultConfig().Max = %v, want 60s", cfg.Max)
	}
}
