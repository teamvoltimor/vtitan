package supervise

import (
	"context"
	"fmt"
	"log/slog"
	"time"

	"github.com/go-playground/validator/v10"
	"golang.org/x/sync/errgroup"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/statemachine/backoff"
)

// Target names one goroutine handed to Supervisor.RunAll: Name identifies
// it in restart log lines, Fn is the supervised function itself (same
// shape Supervisor.Run takes directly).
type Target struct {
	Name string
	Fn   func(ctx context.Context) error
}

// Supervisor runs one or more long-lived goroutines (a motor control loop,
// an IMU/LIDAR publish loop, an OLED render loop, ...) inside a single
// process, restarting each one with backoff if it panics or returns an
// error, and leaving a clean return (nil error, or ctx cancellation) alone.
type Supervisor struct {
	cfg    Config
	logger *slog.Logger
}

// New builds a Supervisor from cfg, validating it up front so a bad
// deployment config (e.g. HealthyDuration <= 0) fails at process startup
// rather than silently misbehaving the first time a supervised goroutine
// crashes, matching nats.Connect's validate-at-the-call-site convention.
func New(cfg Config, logger *slog.Logger) (*Supervisor, error) {
	if err := validator.New().Struct(cfg); err != nil {
		return nil, fmt.Errorf("supervise: invalid config: %w", err)
	}
	return &Supervisor{cfg: cfg, logger: logger}, nil
}

// Run calls fn repeatedly under name until fn returns nil, ctx is done, or
// the process exits. A panic inside fn is recovered and treated exactly
// like a returned error -- one supervised goroutine crashing must never
// take the rest of the process down with it. Restarts are delayed by
// s.cfg.Backoff, which is reset to Backoff.Initial once fn has run for at
// least s.cfg.HealthyDuration before its next failure.
func (s *Supervisor) Run(ctx context.Context, name string, fn func(ctx context.Context) error) error {
	restartDelay := backoff.New(s.cfg.Backoff)

	for {
		select {
		case <-ctx.Done():
			return nil
		default:
		}

		startedAt := time.Now()
		runErr := runRecovered(ctx, fn)

		select {
		case <-ctx.Done():
			// A cancellation racing with fn's own return is shutdown, not
			// a failure to restart from -- don't backoff-sleep or log an
			// error for it even if fn happened to return one on its way
			// out.
			return nil
		default:
		}
		if runErr == nil {
			return nil
		}

		if time.Since(startedAt) >= s.cfg.HealthyDuration {
			restartDelay.Reset()
		}

		delay := restartDelay.Next()
		s.logger.Error(
			"supervise: goroutine failed, restarting",
			"name", name,
			"error", runErr,
			"delay", delay,
		)

		if !sleep(ctx, delay) {
			return nil
		}
	}
}

// RunAll runs every target concurrently via Run and waits for all of them
// to stop -- which, per Run's own contract, only happens on ctx
// cancellation or a target's fn returning nil, never on a supervised
// failure (Run absorbs those into a restart, by design). errgroup.WithContext
// is used purely for its fan-out/wait-for-all shape here, not for its
// first-error-cancels-the-group behavior, since Run itself never surfaces
// an error to cancel on -- see go.mod, errgroup is already a dependency of
// this module.
func (s *Supervisor) RunAll(ctx context.Context, targets ...Target) error {
	group, groupCtx := errgroup.WithContext(ctx)
	for _, target := range targets {
		group.Go(func() error {
			return s.Run(groupCtx, target.Name, target.Fn)
		})
	}
	return group.Wait() //nolint:wrapcheck // Run never returns a non-nil error by contract; nothing here to wrap
}

// runRecovered calls fn and converts a panic into an error, so Run's
// restart loop has exactly one failure shape to handle regardless of
// whether fn returned an error or panicked.
func runRecovered(ctx context.Context, fn func(ctx context.Context) error) (err error) {
	defer func() {
		if r := recover(); r != nil {
			err = fmt.Errorf("supervise: recovered panic: %v", r)
		}
	}()
	return fn(ctx)
}

// sleep waits for delay or ctx cancellation, whichever comes first,
// reporting which one happened -- so Run never blocks shutdown on a long
// backoff delay.
func sleep(ctx context.Context, delay time.Duration) bool {
	timer := time.NewTimer(delay)
	defer timer.Stop()

	select {
	case <-ctx.Done():
		return false
	case <-timer.C:
		return true
	}
}
