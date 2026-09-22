//go:build linux

package motor_test

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"math"
	"sync"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/internal/node/motor"
	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
)

// These tests pin the motor half of the failsafe contract (go-future.md
// §2.5) in software. What they cannot reach is the physical half: how long
// the H-bridge takes to follow a zero duty, and what the pins do while the
// process is gone. That still needs the bench.

// recordingDrive records every duty the loop applies.
type recordingDrive struct {
	mu     sync.Mutex
	duties []float64
}

type discardStatus struct{}

// chanReader feeds commands from a channel, and fails with err once the
// channel is closed.
type chanReader struct {
	cmds chan *actuationv1.AckermannCmd
	err  error
}

// failsafeTimeout is short so the tests are fast; the contract is about the
// loop honouring whatever timeout it is given, not about 500 ms itself.
const failsafeTimeout = 100 * time.Millisecond

// failsafeWait bounds every wait for the loop to react. Generous, because a
// loaded CI runner is not a real-time system either.
const failsafeWait = 2 * time.Second

func (d *recordingDrive) SetSpeed(_ context.Context, duty float64) error {
	d.mu.Lock()
	defer d.mu.Unlock()
	d.duties = append(d.duties, duty)
	return nil
}

func (d *recordingDrive) all() []float64 {
	d.mu.Lock()
	defer d.mu.Unlock()
	return append([]float64(nil), d.duties...)
}

func (d *recordingDrive) last() (float64, bool) {
	d.mu.Lock()
	defer d.mu.Unlock()
	if len(d.duties) == 0 {
		return 0, false
	}
	return d.duties[len(d.duties)-1], true
}

// waitForDuty polls until the last applied duty equals want.
func (d *recordingDrive) waitForDuty(t *testing.T, want float64, why string) {
	t.Helper()
	deadline := time.Now().Add(failsafeWait)
	for time.Now().Before(deadline) {
		if got, ok := d.last(); ok && got == want {
			return
		}
		time.Sleep(5 * time.Millisecond)
	}
	got, _ := d.last()
	t.Fatalf("%s: last duty %v, want %v within %v", why, got, want, failsafeWait)
}

func (discardStatus) Publish(*actuationv1.MotorStatus) error { return nil }

func (r *chanReader) Read(ctx context.Context) (*actuationv1.AckermannCmd, error) {
	select {
	case cmd, ok := <-r.cmds:
		if !ok {
			return nil, r.err
		}
		return cmd, nil
	case <-ctx.Done():
		return nil, fmt.Errorf("reading command: %w", ctx.Err())
	}
}

func newFailsafeLoop(drv *recordingDrive) *motor.Loop {
	logger := slog.New(slog.DiscardHandler)
	return motor.NewLoop(logger, drv, discardStatus{}, motor.DefaultSpeedScalePercentPerMPS)
}

// runLoop starts l.Run in the background and returns a channel with its
// result.
func runLoop(ctx context.Context, l *motor.Loop, r motor.CommandReader) <-chan error {
	done := make(chan error, 1)
	go func() { done <- l.Run(ctx, r, failsafeTimeout) }()
	return done
}

// Contract row 1: the brain stops publishing. The drive must reach zero,
// and must not resume on its own.
func TestFailsafe_SilenceStopsTheDrive(t *testing.T) {
	t.Parallel()

	drv := &recordingDrive{}
	reader := &chanReader{cmds: make(chan *actuationv1.AckermannCmd)}
	ctx, cancel := context.WithCancel(t.Context())
	defer cancel()
	done := runLoop(ctx, newFailsafeLoop(drv), reader)

	reader.cmds <- &actuationv1.AckermannCmd{Speed: 1.0}
	drv.waitForDuty(t, motor.SpeedToNormalized(1.0, motor.DefaultSpeedScalePercentPerMPS), "command applied")

	drv.waitForDuty(t, 0, "silence past the timeout")

	cancel()
	<-done
}

// Contract row 3: an invalid command is treated as missing, not acted on.
// Each non-finite value is sent repeatedly and faster than the timeout: if a
// rejected command refreshed the watchdog, the drive would never stop.
func TestFailsafe_NonFiniteSpeedIsTreatedAsMissing(t *testing.T) {
	t.Parallel()

	for _, speed := range []float32{
		float32(math.NaN()),
		float32(math.Inf(1)),
		float32(math.Inf(-1)),
	} {
		drv := &recordingDrive{}
		reader := &chanReader{cmds: make(chan *actuationv1.AckermannCmd)}
		ctx, cancel := context.WithCancel(t.Context())
		done := runLoop(ctx, newFailsafeLoop(drv), reader)

		reader.cmds <- &actuationv1.AckermannCmd{Speed: 0.5}
		drv.waitForDuty(t, motor.SpeedToNormalized(0.5, motor.DefaultSpeedScalePercentPerMPS), "command applied")

		deadline := time.Now().Add(3 * failsafeTimeout)
		for time.Now().Before(deadline) {
			reader.cmds <- &actuationv1.AckermannCmd{Speed: speed}
			time.Sleep(failsafeTimeout / 4)
		}
		got, _ := drv.last()
		if got != 0 {
			t.Errorf("speed %v: last duty %v after a stream of them, want 0", speed, got)
		}
		for _, duty := range drv.all() {
			if math.IsNaN(duty) || math.Abs(duty) == 1 {
				t.Errorf("speed %v: the drive was handed duty %v", speed, duty)
			}
		}

		cancel()
		<-done
	}
}

// Run failing is not the end of the story under supervise: the restart
// waits a backoff, and the watchdog is not running during it. The drive has
// to be stopped on the way out, not after the next Run's timeout.
func TestFailsafe_RunErrorStopsTheDriveBeforeReturning(t *testing.T) {
	t.Parallel()

	drv := &recordingDrive{}
	reader := &chanReader{
		cmds: make(chan *actuationv1.AckermannCmd),
		err:  errors.New("subscription lost"),
	}
	// A timeout far beyond the test, so only the exit path can stop it.
	ctx, cancel := context.WithCancel(t.Context())
	defer cancel()
	l := newFailsafeLoop(drv)
	done := make(chan error, 1)
	go func() { done <- l.Run(ctx, reader, time.Hour) }()

	reader.cmds <- &actuationv1.AckermannCmd{Speed: 1.0}
	drv.waitForDuty(t, motor.SpeedToNormalized(1.0, motor.DefaultSpeedScalePercentPerMPS), "command applied")

	close(reader.cmds)
	select {
	case err := <-done:
		if err == nil {
			t.Fatal("Run returned nil on a subscription error")
		}
	case <-time.After(failsafeWait):
		t.Fatal("Run did not return after its reader failed")
	}
	if got, _ := drv.last(); got != 0 {
		t.Fatalf("Run returned with the drive at duty %v, want 0", got)
	}
}
