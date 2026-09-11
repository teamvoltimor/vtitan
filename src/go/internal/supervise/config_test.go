package supervise_test

import (
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/internal/statemachine/backoff"
	"github.com/teamvoltimor/vtitan/src/go/internal/supervise"
)

func TestDefaultConfig(t *testing.T) {
	t.Parallel()

	cfg := supervise.DefaultConfig()

	if want := backoff.DefaultConfig(); cfg.Backoff != want {
		t.Errorf("DefaultConfig().Backoff = %+v, want %+v", cfg.Backoff, want)
	}
	if cfg.HealthyDuration != supervise.DefaultHealthyDuration {
		t.Errorf(
			"DefaultConfig().HealthyDuration = %v, want %v",
			cfg.HealthyDuration, supervise.DefaultHealthyDuration,
		)
	}
	if cfg.HealthyDuration <= 0 {
		t.Error("DefaultConfig().HealthyDuration must be positive")
	}
}

func TestDefaultHealthyDuration(t *testing.T) {
	t.Parallel()

	if supervise.DefaultHealthyDuration != 5*time.Minute {
		t.Errorf("DefaultHealthyDuration = %v, want 5m", supervise.DefaultHealthyDuration)
	}
}
