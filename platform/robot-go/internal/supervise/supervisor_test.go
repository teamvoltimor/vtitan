package supervise_test

import (
	"context"
	"errors"
	"log/slog"
	"sync/atomic"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/statemachine/backoff"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/supervise"
)

const testTimeout = 2 * time.Second

func discardLogger() *slog.Logger {
	return slog.New(slog.DiscardHandler)
}

func fastTestConfig() supervise.Config {
	return supervise.Config{
		Backoff:         backoff.Config{Initial: time.Millisecond, Max: 4 * time.Millisecond},
		HealthyDuration: time.Hour,
	}
}

func TestNew_InvalidConfigReturnsError(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name string
		cfg  supervise.Config
	}{
		{
			name: "zero backoff",
			cfg:  supervise.Config{HealthyDuration: time.Second},
		},
		{
			name: "zero healthy duration",
			cfg:  supervise.Config{Backoff: backoff.DefaultConfig()},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			if _, err := supervise.New(tt.cfg, discardLogger()); err == nil {
				t.Fatalf("New(%+v) error = nil, want non-nil", tt.cfg)
			}
		})
	}
}

func TestNew_ValidConfigReturnsSupervisor(t *testing.T) {
	t.Parallel()

	sup, err := supervise.New(supervise.DefaultConfig(), discardLogger())
	if err != nil {
		t.Fatalf("New() error = %v, want nil", err)
	}
	if sup == nil {
		t.Fatal("New() supervisor = nil, want non-nil")
	}
}

func TestSupervisor_Run_CleanReturnStopsRestarting(t *testing.T) {
	t.Parallel()

	sup, err := supervise.New(fastTestConfig(), discardLogger())
	if err != nil {
		t.Fatalf("New() error = %v", err)
	}

	var calls atomic.Int32
	ctx, cancel := context.WithTimeout(context.Background(), testTimeout)
	defer cancel()

	err = sup.Run(ctx, "clean-exit", func(context.Context) error {
		calls.Add(1)
		return nil
	})
	if err != nil {
		t.Fatalf("Run() error = %v, want nil", err)
	}
	if got := calls.Load(); got != 1 {
		t.Errorf("calls = %d, want 1 (fn returning nil must not be restarted)", got)
	}
}

func TestSupervisor_Run_RestartsAfterError(t *testing.T) {
	t.Parallel()

	sup, err := supervise.New(fastTestConfig(), discardLogger())
	if err != nil {
		t.Fatalf("New() error = %v", err)
	}

	var calls atomic.Int32
	ctx, cancel := context.WithTimeout(context.Background(), testTimeout)
	defer cancel()

	errFn := errors.New("boom")
	runErr := sup.Run(ctx, "flaky", func(context.Context) error {
		n := calls.Add(1)
		if n >= 3 {
			return nil
		}
		return errFn
	})
	if runErr != nil {
		t.Fatalf("Run() error = %v, want nil", runErr)
	}
	if got := calls.Load(); got != 3 {
		t.Errorf("calls = %d, want 3", got)
	}
}

func TestSupervisor_Run_RestartsAfterPanic(t *testing.T) {
	t.Parallel()

	sup, err := supervise.New(fastTestConfig(), discardLogger())
	if err != nil {
		t.Fatalf("New() error = %v", err)
	}

	var calls atomic.Int32
	ctx, cancel := context.WithTimeout(context.Background(), testTimeout)
	defer cancel()

	runErr := sup.Run(ctx, "panicky", func(context.Context) error {
		n := calls.Add(1)
		if n >= 2 {
			return nil
		}
		panic("kaboom")
	})
	if runErr != nil {
		t.Fatalf("Run() error = %v, want nil", runErr)
	}
	if got := calls.Load(); got != 2 {
		t.Errorf("calls = %d, want 2 (panic must be recovered and restarted)", got)
	}
}

func TestSupervisor_Run_ContextCancelStopsPromptly(t *testing.T) {
	t.Parallel()

	cfg := supervise.Config{
		Backoff:         backoff.Config{Initial: time.Hour, Max: 2 * time.Hour},
		HealthyDuration: time.Hour,
	}
	sup, err := supervise.New(cfg, discardLogger())
	if err != nil {
		t.Fatalf("New() error = %v", err)
	}

	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() {
		done <- sup.Run(ctx, "cancel-during-backoff", func(context.Context) error {
			return errors.New("always fails")
		})
	}()

	// Let fn run (and fail) at least once so Run is asleep in its backoff
	// delay -- which is an hour long, per cfg above -- before canceling,
	// proving cancellation isn't just short-circuiting the first call.
	time.Sleep(20 * time.Millisecond)
	cancel()

	select {
	case runErr := <-done:
		if runErr != nil {
			t.Errorf("Run() error = %v, want nil", runErr)
		}
	case <-time.After(testTimeout):
		t.Fatal("Run() did not return promptly after ctx cancellation during backoff sleep")
	}
}

func TestSupervisor_Run_ResetsBackoffAfterHealthyRun(t *testing.T) {
	t.Parallel()

	cfg := supervise.Config{
		Backoff:         backoff.Config{Initial: time.Millisecond, Max: time.Hour},
		HealthyDuration: 30 * time.Millisecond,
	}
	sup, err := supervise.New(cfg, discardLogger())
	if err != nil {
		t.Fatalf("New() error = %v", err)
	}

	var calls atomic.Int32
	ctx, cancel := context.WithTimeout(context.Background(), testTimeout)
	defer cancel()

	// First call fails immediately (delay stays at Initial, no health
	// window elapsed). Second call runs past HealthyDuration before
	// failing, so its failure must reset the delay back to Initial rather
	// than leaving it doubled -- if it didn't, the third call would only
	// start after Backoff.Max (an hour), and this test would time out.
	runErr := sup.Run(ctx, "recovers", func(context.Context) error {
		n := calls.Add(1)
		switch n {
		case 1:
			return errors.New("first failure")
		case 2:
			time.Sleep(cfg.HealthyDuration + 10*time.Millisecond)
			return errors.New("second failure, after healthy run")
		default:
			return nil
		}
	})
	if runErr != nil {
		t.Fatalf("Run() error = %v, want nil", runErr)
	}
	if got := calls.Load(); got != 3 {
		t.Errorf("calls = %d, want 3", got)
	}
}

func TestSupervisor_RunAll_CancelStopsEveryTarget(t *testing.T) {
	t.Parallel()

	sup, err := supervise.New(fastTestConfig(), discardLogger())
	if err != nil {
		t.Fatalf("New() error = %v", err)
	}

	ctx, cancel := context.WithCancel(context.Background())

	var flakyStarted, longRunnerStarted, longRunnerCanceled atomic.Bool
	targets := []supervise.Target{
		{
			// fastTestConfig's Backoff.Max is 4ms, so this target keeps
			// erroring and restarting for the whole test, exercising that
			// RunAll's cancellation reaches a target mid-restart-loop, not
			// just a target that is blocked in a single fn call.
			Name: "flaky",
			Fn: func(context.Context) error {
				flakyStarted.Store(true)
				return errors.New("always fails")
			},
		},
		{
			Name: "long-runner",
			Fn: func(fnCtx context.Context) error {
				longRunnerStarted.Store(true)
				<-fnCtx.Done()
				longRunnerCanceled.Store(true)
				return nil
			},
		},
	}

	done := make(chan error, 1)
	go func() {
		done <- sup.RunAll(ctx, targets...)
	}()

	time.Sleep(20 * time.Millisecond)
	cancel()

	select {
	case runErr := <-done:
		if runErr != nil {
			t.Errorf("RunAll() error = %v, want nil", runErr)
		}
	case <-time.After(testTimeout):
		t.Fatal("RunAll() did not return promptly after ctx cancellation")
	}

	if !flakyStarted.Load() {
		t.Error("flaky target never started")
	}
	if !longRunnerStarted.Load() {
		t.Error("long-runner target never started")
	}
	if !longRunnerCanceled.Load() {
		t.Error("long-runner target was not cancelled")
	}
}
