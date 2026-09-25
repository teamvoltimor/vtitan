package picolink_test

import (
	"context"
	"errors"
	"math"
	"net"
	"path/filepath"
	"strings"
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
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
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

const (
	// virtualCountsPerS is the virtual wheel's count rate at full duty.
	virtualCountsPerS = 2000
	// drivingSpeedMPS is the speed startDriving commands.
	drivingSpeedMPS = 0.5
)

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

// drivingDuty is the H-bridge duty sampleBoard produces for
// drivingSpeedMPS.
func drivingDuty() float64 {
	d := actuation.SpeedToNormalized(drivingSpeedMPS, float64(sampleBoard.SpeedScalePctPerMPS))
	if sampleBoard.InvertDrive {
		d = -d
	}
	return d
}

// startDriving configures a virtual board and keeps commanding
// drivingSpeedMPS until the drive carries it.
func startDriving(t *testing.T, opts boardsim.Options) *virtualHarness {
	t.Helper()

	v := startVirtual(t, picolink.SessionConfig{Board: sampleBoard}, opts)
	v.waitState(t, actuationv1.MotorStatus_STATE_IDLE)
	v.keepCommanding(t, &actuationv1.AckermannCmd{Speed: drivingSpeedMPS})
	want := drivingDuty()
	waitFor(t, "the commanded duty", func() bool { return math.Abs(v.board.Drive.Duty()-want) < 1e-9 })
	return v
}

// A host-to-board stall shorter than the board's command timeout, with
// commands queued behind it, does not stop the car.
func TestFailsafe_ShortStallKeepsDriving(t *testing.T) {
	t.Parallel()

	v := startDriving(t, boardsim.Options{})
	// Commands every 50 ms: the longest gap is the stall plus one period,
	// 300 ms against a 500 ms timeout.
	v.board.StallToBoard(250 * time.Millisecond)
	time.Sleep(450 * time.Millisecond)

	if stops := v.board.Counters().WatchdogStops; stops != 0 {
		t.Errorf("WatchdogStops = %d after a 250 ms stall, want 0", stops)
	}
	if d := v.board.Drive.Duty(); math.Abs(d-drivingDuty()) > 1e-9 {
		t.Errorf("duty after a short stall = %v, want %v", d, drivingDuty())
	}
}

// A stall longer than the board's command timeout stops the car on the
// board's own authority, around CommandTimeoutMS after the last command
// got through, and driving resumes when the link does. The backlog the
// stall released is not replayed: the board applies only the newest
// command of each Step and supersedes the rest.
func TestFailsafe_LongStallStopsThenRecovers(t *testing.T) {
	t.Parallel()

	v := startDriving(t, boardsim.Options{})
	applied := v.board.Counters().CommandsApplied

	stalled := time.Now()
	v.board.StallToBoard(1200 * time.Millisecond)
	waitFor(t, "the board's watchdog to stop the drive", func() bool {
		return v.board.Drive.Duty() == 0
	})
	stoppedAfter := time.Since(stalled)
	timeout := time.Duration(sampleBoard.CommandTimeoutMS) * time.Millisecond
	// The last command crossed up to one 50 ms period before the stall.
	if stoppedAfter < timeout-100*time.Millisecond || stoppedAfter > timeout+300*time.Millisecond {
		t.Errorf("drive stopped %v after the stall began, want about the %v command timeout", stoppedAfter, timeout)
	}

	waitFor(t, "driving to resume after the stall", func() bool {
		return math.Abs(v.board.Drive.Duty()-drivingDuty()) < 1e-9
	})
	c := v.board.Counters()
	// About 24 commands queue behind a 1.2 s stall at one per 50 ms.
	if c.CommandsSuperseded < 10 {
		t.Errorf("CommandsSuperseded = %d, want the released backlog (about 24) collapsed", c.CommandsSuperseded)
	}
	t.Logf("stopped %v into the stall; applied %d, superseded %d since it began",
		stoppedAfter.Round(time.Millisecond), c.CommandsApplied-applied, c.CommandsSuperseded)
}

// When the board goes silent past the host's link timeout, the host
// publishes a FAULT status saying so, and reports again once frames return.
func TestFailsafe_BoardSilenceIsReportedAsLinkLost(t *testing.T) {
	t.Parallel()

	v := startVirtual(t, picolink.SessionConfig{Board: sampleBoard}, boardsim.Options{})
	v.waitState(t, actuationv1.MotorStatus_STATE_IDLE)
	v.board.StallToHost(1600 * time.Millisecond)

	deadline := time.After(waitTimeout)
	for lost := false; !lost; {
		select {
		case st := <-v.status:
			lost = st.GetState() == actuationv1.MotorStatus_STATE_FAULT &&
				strings.Contains(st.GetDetail(), "link lost")
		case <-deadline:
			t.Fatal("no link-lost MotorStatus while the board was silent")
		}
	}
	v.waitLog(t, "no frame from the board")
	v.waitState(t, actuationv1.MotorStatus_STATE_IDLE)
}

// A board that resets mid-run (watchdog or brownout) is noticed by its new
// boot ID, reconfigured by the host, and drives again on the next command.
func TestFailsafe_BoardResetMidRunIsReconfigured(t *testing.T) {
	t.Parallel()

	v := startDriving(t, boardsim.Options{BootID: 1})

	if err := v.board.Reboot(2, boardlink.FaultWatchdogReset); err != nil {
		t.Fatalf("Reboot: %v", err)
	}
	v.waitLog(t, "board reset mid-run, reconfiguring")
	waitFor(t, "the rebooted board to drive again", func() bool {
		cfg, ok := v.board.Configured()
		return ok && cfg == sampleBoard && math.Abs(v.board.Drive.Duty()-drivingDuty()) < 1e-9
	})
}

// Bursty loss in both directions (a third of all chunks, three at a time)
// still converges to a configured board carrying the command. The test
// keeps driving until the emulation has provably lost traffic both ways,
// then checks the drive still carries the command.
func TestFailsafe_BurstyLossStillConverges(t *testing.T) {
	t.Parallel()

	lossy := boardsim.Direction{LossRate: 0.3, LossBurst: 3}
	v := startDriving(t, boardsim.Options{Link: boardsim.LinkConfig{ToBoard: lossy, ToHost: lossy, Seed: 21}})

	waitFor(t, "chunks lost in both directions", func() bool {
		s := v.board.LinkStats()
		return s.ToBoard.ChunksLost > 0 && s.ToHost.ChunksLost > 0
	})
	waitFor(t, "the commanded duty after losses", func() bool {
		return math.Abs(v.board.Drive.Duty()-drivingDuty()) < 1e-9
	})
}
