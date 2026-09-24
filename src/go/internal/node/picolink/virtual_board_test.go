package picolink_test

import (
	"context"
	"errors"
	"math"
	"net"
	"path/filepath"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/internal/hwconfig"
	"github.com/teamvoltimor/vtitan/src/go/internal/node/picolink"
	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
	uiv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/ui/v1"
	"github.com/teamvoltimor/vtitan/src/go/pkg/boardsim"
	driverbutton "github.com/teamvoltimor/vtitan/src/go/pkg/driver/button"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/actuation"
)

// These tests run the host Session against boardsim, the real
// boardloop.Loop on in-memory hardware, instead of the hand-scripted
// fakeBoard: the Config, the command conversion, the failsafe and the
// odometry all come from the code the Pico firmware runs.

// virtualHarness is a Session wired to a running virtual board.
type virtualHarness struct {
	*harness

	board *boardsim.Board
}

// virtualCountsPerS is the virtual wheel's count rate at full duty.
const virtualCountsPerS = 2000

// startVirtual runs a Session and a boardsim.Board over a net.Pipe until
// the test ends.
func startVirtual(t *testing.T, cfg picolink.SessionConfig, opts boardsim.Options) *virtualHarness {
	t.Helper()

	hostEnd, boardEnd := net.Pipe()
	board, err := boardsim.New(boardEnd, opts)
	if err != nil {
		t.Fatalf("boardsim.New: %v", err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() { done <- board.Run(ctx) }()
	// Registered before the session's cleanup so it runs after it.
	t.Cleanup(func() {
		cancel()
		_ = boardEnd.Close()
		select {
		case runErr := <-done:
			if runErr != nil && !errors.Is(runErr, boardsim.ErrLinkClosed) {
				t.Errorf("Board.Run: %v", runErr)
			}
		case <-time.After(waitTimeout):
			t.Error("Board.Run did not return after cancellation")
		}
	})

	return &virtualHarness{harness: startSessionOn(t, cfg, hostEnd), board: board}
}

// waitFor polls cond until it holds or waitTimeout passes.
func waitFor(t *testing.T, what string, cond func() bool) {
	t.Helper()

	deadline := time.Now().Add(waitTimeout)
	for !cond() {
		if time.Now().After(deadline) {
			t.Fatalf("timed out waiting for %s", what)
		}
		time.Sleep(time.Millisecond)
	}
}

// waitState waits for a MotorStatus in state want.
func (v *virtualHarness) waitState(t *testing.T, want actuationv1.MotorStatus_State) {
	t.Helper()

	deadline := time.After(waitTimeout)
	for {
		select {
		case st := <-v.status:
			if st.GetState() == want {
				return
			}
		case <-deadline:
			t.Fatalf("no MotorStatus in %s within %s", want, waitTimeout)
		}
	}
}

// keepCommanding sends cmd every 50 ms, well inside the command timeout,
// until the test ends.
func (v *virtualHarness) keepCommanding(t *testing.T, cmd *actuationv1.AckermannCmd) {
	t.Helper()

	ctx, cancel := context.WithCancel(context.Background())
	t.Cleanup(cancel)
	go func() {
		ticker := time.NewTicker(50 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case v.cmds <- cmd:
			case <-ctx.Done():
				return
			}
			select {
			case <-ticker.C:
			case <-ctx.Done():
				return
			}
		}
	}()
}

// The session's Config configures the real Loop: the drive connects, the
// servo gets its first (centering) pulse, and the board reports Idle.
func TestVirtualBoard_ConfiguresAndReportsIdle(t *testing.T) {
	t.Parallel()

	v := startVirtual(t, picolink.SessionConfig{Board: sampleBoard}, boardsim.Options{BootID: 7})
	v.waitState(t, actuationv1.MotorStatus_STATE_IDLE)

	got, ok := v.board.Configured()
	if !ok || got != sampleBoard {
		t.Fatalf("board Config = %+v (configured %v), want %+v", got, ok, sampleBoard)
	}
	if !v.board.Drive.Connected() {
		t.Error("drive not connected after a valid Config")
	}
	if _, written := v.board.Servo.PulseUS(); !written {
		t.Error("servo never written after a valid Config")
	}
}

// A command reaches the H-bridge with the board's own conversion and the
// Config's inversion applied once; when the stream stops, the board's
// failsafe zeroes the drive on its own.
func TestVirtualBoard_CommandDrivesThenWatchdogStops(t *testing.T) {
	t.Parallel()

	v := startVirtual(t, picolink.SessionConfig{Board: sampleBoard}, boardsim.Options{})
	v.waitState(t, actuationv1.MotorStatus_STATE_IDLE)
	center, _ := v.board.Servo.PulseUS()

	const speedMPS = 0.5
	v.cmds <- &actuationv1.AckermannCmd{Speed: speedMPS, SteeringAngle: 0.2}

	want := actuation.SpeedToNormalized(speedMPS, float64(sampleBoard.SpeedScalePctPerMPS))
	if sampleBoard.InvertDrive {
		want = -want
	}
	waitFor(t, "the commanded duty on the drive", func() bool {
		return math.Abs(v.board.Drive.Duty()-want) < 1e-9
	})
	if pulse, _ := v.board.Servo.PulseUS(); pulse == center {
		t.Errorf("servo pulse stayed at center %v under a 0.2 rad steering command", pulse)
	}

	// No further commands: the board stops itself after CommandTimeoutMS.
	waitFor(t, "the board's command watchdog to stop the drive", func() bool {
		return v.board.Drive.Duty() == 0 && v.board.Counters().WatchdogStops > 0
	})
	if pulse, _ := v.board.Servo.PulseUS(); pulse != center {
		t.Errorf("servo pulse after watchdog stop = %v, want center %v", pulse, center)
	}
}

// A forward command turns the virtual wheel, and the counts come back
// through the board's Odometry into a positive JointStates velocity.
func TestVirtualBoard_ForwardCommandYieldsOdometry(t *testing.T) {
	t.Parallel()

	board := sampleBoard
	board.InvertDrive = false
	v := startVirtual(t,
		picolink.SessionConfig{Board: board, Encoder: &picolink.EncoderParams{CountsPerRev: 60}},
		boardsim.Options{CountsPerSecondAtFullDuty: virtualCountsPerS},
	)
	v.waitState(t, actuationv1.MotorStatus_STATE_IDLE)
	v.keepCommanding(t, &actuationv1.AckermannCmd{Speed: 0.5})

	deadline := time.After(waitTimeout)
	for {
		select {
		case js := <-v.joints:
			if vel := js.GetVelocity(); len(vel) > 0 && vel[0] > 0 {
				return
			}
		case <-deadline:
			t.Fatalf("no positive JointStates velocity within %s (encoder at %d)",
				waitTimeout, v.board.Encoder.Counts())
		}
	}
}

// Pressing the virtual board's button goes through the board's raw edge
// report and the host's evaluator to a short press.
func TestVirtualBoard_ButtonPressBecomesShortPress(t *testing.T) {
	t.Parallel()

	v := startVirtual(t, picolink.SessionConfig{
		Board:  sampleBoard,
		Button: &picolink.ButtonParams{Thresholds: driverbutton.DefaultThresholds()},
	}, boardsim.Options{})
	v.waitState(t, actuationv1.MotorStatus_STATE_IDLE)

	v.board.Button.SetPressed(true)
	time.Sleep(150 * time.Millisecond)
	v.board.Button.SetPressed(false)

	deadline := time.After(waitTimeout)
	for {
		select {
		case ev := <-v.button:
			if ev.GetKind() == uiv1.ButtonEvent_KIND_SHORT_PRESS {
				return
			}
		case <-deadline:
			t.Fatal("no KIND_SHORT_PRESS ButtonEvent within the timeout")
		}
	}
}

// A drive write that fails on the board surfaces on the host as a Fault
// MotorStatus.
func TestVirtualBoard_DriveFailureReportsFault(t *testing.T) {
	t.Parallel()

	v := startVirtual(t, picolink.SessionConfig{Board: sampleBoard}, boardsim.Options{})
	v.waitState(t, actuationv1.MotorStatus_STATE_IDLE)

	v.board.Drive.SetError(errors.New("bridge overcurrent"))
	v.keepCommanding(t, &actuationv1.AckermannCmd{Speed: 0.5})
	v.waitState(t, actuationv1.MotorStatus_STATE_FAULT)
	v.waitLog(t, "actuator write failure")
}

// The session configures the board and drives it over the link emulation
// shipped in board_sim.toml, loaded the way a simulation would load it.
func TestVirtualBoard_ShippedLinkEmulation(t *testing.T) {
	t.Setenv(profile.EnvVar, "")

	opts, err := hwconfig.BoardSim(filepath.Join("..", "..", "..", "..", ".."))
	if err != nil {
		t.Fatalf("hwconfig.BoardSim: %v", err)
	}
	v := startVirtual(t, picolink.SessionConfig{Board: sampleBoard}, opts)
	v.waitState(t, actuationv1.MotorStatus_STATE_IDLE)

	v.keepCommanding(t, &actuationv1.AckermannCmd{Speed: 0.5})
	waitFor(t, "a nonzero duty over the shipped link", func() bool {
		return v.board.Drive.Duty() != 0
	})
}

// A slow, jittery link that corrupts bytes in both directions still
// converges: the board drops the damaged frames, resynchronizes on the
// frame delimiter, answers Hello until a Config survives, and a clean
// Command eventually lands. At 2% per byte roughly half of all frames are
// damaged, and the test insists some were, so it cannot pass vacuously.
func TestVirtualBoard_SurvivesACorruptLink(t *testing.T) {
	t.Parallel()

	noisy := boardsim.Direction{Latency: 5 * time.Millisecond, Jitter: 5 * time.Millisecond, CorruptRate: 0.02}
	v := startVirtual(t, picolink.SessionConfig{Board: sampleBoard},
		boardsim.Options{Link: boardsim.LinkConfig{ToBoard: noisy, ToHost: noisy, Seed: 11}})
	v.waitState(t, actuationv1.MotorStatus_STATE_IDLE)

	const speedMPS = 0.5
	want := -actuation.SpeedToNormalized(speedMPS, float64(sampleBoard.SpeedScalePctPerMPS)) // InvertDrive
	v.keepCommanding(t, &actuationv1.AckermannCmd{Speed: speedMPS})
	waitFor(t, "the board to drop a corrupted frame", func() bool {
		return v.board.Counters().FramesDropped > 0
	})
	waitFor(t, "the commanded duty through a corrupting link", func() bool {
		return math.Abs(v.board.Drive.Duty()-want) < 1e-9
	})
}
