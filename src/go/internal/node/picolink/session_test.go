package picolink_test

import (
	"math"
	"strings"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/internal/node/picolink"
	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
)

const floatTol = 1e-9

// sampleBoard is a Config with every field distinct, so a field swapped on
// the way out would show.
var sampleBoard = boardlink.Config{
	CommandTimeoutMS:    500,
	SpeedScalePctPerMPS: 30,
	InvertDrive:         true,
	LinkageRatio:        0.63,
	ServoMaxAngleDeg:    135,
	SteeringOffsetDeg:   -1.5,
	ServoMinPulseUS:     500,
	ServoMaxPulseUS:     2500,
	ServoCenterPulseUS:  1500,
	ServoRangeDeg:       270,
	ServoReversed:       true,
	HardwareWatchdogMS:  picolink.HardwareWatchdogMS,
	StatusIntervalMS:    picolink.StatusIntervalMS,
	OdometryIntervalMS:  picolink.OdometryIntervalMS,
}

// A configured board sends no Hello, so the session opens with Config
// unprompted, as its first frame, preceded by a delimiter that flushes a
// half-received frame. The board's Status is logged as the configuration
// taking effect.
func TestRun_SendsConfigBeforeAnyHello(t *testing.T) {
	t.Parallel()

	h := startSessionRaw(t, picolink.SessionConfig{Board: sampleBoard})
	first, ok := <-h.board.pkts
	if !ok || first.Type != boardlink.TypeConfig || first.Config != sampleBoard {
		t.Fatalf("first frame = %+v, want the Config", first)
	}
	if b, seen := h.board.firstByte(); !seen || b != 0 {
		t.Errorf("first byte on the wire = %#x (seen %v), want the 0x00 flush", b, seen)
	}
	h.board.send(t, boardlink.Packet{Type: boardlink.TypeStatus, Status: boardlink.Status{State: boardlink.StateIdle}})
	h.waitLog(t, "level=INFO", "board configured")
}

// A Config the board would refuse is never sent; the reason is logged.
func TestRun_InvalidConfigIsNotSent(t *testing.T) {
	t.Parallel()

	bad := sampleBoard
	bad.ServoCenterPulseUS = 5000
	h := startSessionRaw(t, picolink.SessionConfig{Board: bad, PingInterval: 50 * time.Millisecond})
	h.waitLog(t, "level=ERROR", "invalid config", "center pulse")
	h.board.send(t, boardlink.Packet{Type: boardlink.TypeHello, Hello: boardlink.Hello{
		ProtocolVersion: boardlink.Version, BootID: 4,
	}})

	// Everything the host sends up to its second Ping, which it sends well
	// after handling the Hello, must be Ping.
	for pings := 0; pings < 2; {
		select {
		case p := <-h.board.pkts:
			if p.Type != boardlink.TypePing {
				t.Fatalf("host sent %s with an invalid config: %+v", p.Type, p.Config)
			}
			pings++
		case <-time.After(waitTimeout):
			t.Fatal("no Ping from the host")
		}
	}
}

// A Hello from the same boot while a Config is pending is the board
// refusing it: a WARN, and no Config again until the retry interval.
func TestHello_AfterConfigIsARefusal(t *testing.T) {
	t.Parallel()

	h := startSession(t, picolink.SessionConfig{Board: sampleBoard})
	hello := boardlink.Packet{Type: boardlink.TypeHello, Hello: boardlink.Hello{
		ProtocolVersion: boardlink.Version, BootID: 9, Faults: boardlink.FaultActuator,
	}}

	// The first Hello may have crossed the initial Config: not a refusal.
	h.board.send(t, hello)
	h.board.expect(t, boardlink.TypeConfig)
	if strings.Contains(h.logs.String(), "refused") {
		t.Fatalf("the first hello was taken as a refusal:\n%s", h.logs.String())
	}

	// No Status came back, and the board says Hello again: refused.
	h.board.send(t, hello)
	h.waitLog(t, "level=WARN", "board refused config", "boot_id=9", "faults=actuator")

	// Its next regular Hello, after the retry interval, is answered.
	time.Sleep(250 * time.Millisecond)
	h.board.send(t, hello)
	h.board.expect(t, boardlink.TypeConfig)
}

// Every Hello is answered with the configured Config.
func TestHello_AnswersWithConfig(t *testing.T) {
	t.Parallel()

	h := startSession(t, picolink.SessionConfig{Board: sampleBoard})
	h.board.send(t, boardlink.Packet{Type: boardlink.TypeHello, Hello: boardlink.Hello{
		ProtocolVersion: boardlink.Version, BootID: 0xCAFE,
	}})

	got := h.board.expect(t, boardlink.TypeConfig)
	if got.Config != sampleBoard {
		t.Errorf("Config = %+v, want %+v", got.Config, sampleBoard)
	}
	h.waitLog(t, "level=INFO", "board hello", "boot_id=51966")
}

// A new BootID is a reset, logged at WARN with the fault bits, and the
// board is reconfigured; it is not a refusal, even with a Config pending.
func TestHello_BootIDChangeIsLoggedAsReset(t *testing.T) {
	t.Parallel()

	h := startSession(t, picolink.SessionConfig{Board: sampleBoard})
	hello := func(bootID uint32, faults boardlink.Faults) {
		h.board.send(t, boardlink.Packet{Type: boardlink.TypeHello, Hello: boardlink.Hello{
			ProtocolVersion: boardlink.Version, BootID: bootID, Faults: faults,
		}})
		if got := h.board.expect(t, boardlink.TypeConfig); got.Config != sampleBoard {
			t.Errorf("Config after hello %d = %+v", bootID, got.Config)
		}
	}

	hello(1, 0)
	h.board.send(t, boardlink.Packet{Type: boardlink.TypeStatus, Status: boardlink.Status{State: boardlink.StateIdle}})
	h.waitLog(t, "board configured")
	if strings.Contains(h.logs.String(), "reset") {
		t.Fatalf("a first hello was logged as a reset:\n%s", h.logs.String())
	}
	hello(2, boardlink.FaultWatchdogReset)
	h.waitLog(t, "level=WARN", "board reset", "previous_boot_id=1", "boot_id=2", "faults=watchdog_reset")
	hello(3, 0)
	h.waitLog(t, "level=WARN", "board reset", "previous_boot_id=2", "boot_id=3")
	if strings.Contains(h.logs.String(), "refused") {
		t.Fatalf("a reset was logged as a refusal:\n%s", h.logs.String())
	}
}

// Each AckermannCmd reaches the board as a Command carrying the same values,
// non-finite ones included: the board's rule rejects and reports those.
func TestCommand_ForwardedUnfiltered(t *testing.T) {
	t.Parallel()

	h := startSession(t, picolink.SessionConfig{Board: sampleBoard})

	h.cmds <- &actuationv1.AckermannCmd{Speed: 0.75, SteeringAngle: -0.25}
	got := h.board.expect(t, boardlink.TypeCommand).Command
	if got.SpeedMPS != 0.75 || got.SteeringAngleRad != -0.25 {
		t.Errorf("Command = %+v, want speed 0.75, steering -0.25", got)
	}

	h.cmds <- &actuationv1.AckermannCmd{Speed: float32(math.NaN()), SteeringAngle: float32(math.Inf(1))}
	got = h.board.expect(t, boardlink.TypeCommand).Command
	if !math.IsNaN(float64(got.SpeedMPS)) || !math.IsInf(float64(got.SteeringAngleRad), 1) {
		t.Errorf("non-finite Command = %+v, want NaN speed and +Inf steering forwarded", got)
	}
}

func TestStatus_PublishedAsMotorStatus(t *testing.T) {
	t.Parallel()

	h := startSession(t, picolink.SessionConfig{Board: sampleBoard})
	h.board.send(t, boardlink.Packet{Type: boardlink.TypeStatus, Status: boardlink.Status{
		BoardTimeUS: 1, State: boardlink.StateRunning, Duty: -0.4, CommandAgeMS: 17,
	}})

	st := h.nextStatus(t)
	if st.GetState() != actuationv1.MotorStatus_STATE_RUNNING || st.GetDutyCycle() != -0.4 ||
		st.GetCommandAgeMs() != 17 || st.GetDetail() != "" || st.GetFrameId() != picolink.FrameID ||
		st.GetStamp() == nil {
		t.Errorf("MotorStatus = %v", st)
	}
}

func TestMotorStatusFor(t *testing.T) {
	t.Parallel()

	fault, idle, running := actuationv1.MotorStatus_STATE_FAULT,
		actuationv1.MotorStatus_STATE_IDLE, actuationv1.MotorStatus_STATE_RUNNING
	for _, tc := range []struct {
		name       string
		state      boardlink.State
		faults     boardlink.Faults
		wantState  actuationv1.MotorStatus_State
		wantDetail string
	}{
		{"idle", boardlink.StateIdle, 0, idle, ""},
		{"running", boardlink.StateRunning, 0, running, ""},
		{"fault", boardlink.StateFault, 0, fault, ""},
		{"unconfigured is a fault", boardlink.StateUnconfigured, 0, fault, picolink.DetailUnconfigured},
		{"actuator fault overrides running", boardlink.StateRunning, boardlink.FaultActuator, fault, "actuator"},
		{"rejected command is not a fault", boardlink.StateRunning, boardlink.FaultRejectedCommand, running, ""},
		{"watchdog reset is not a fault", boardlink.StateIdle, boardlink.FaultWatchdogReset, idle, ""},
		{
			"fault lists every bit", boardlink.StateFault,
			boardlink.FaultWatchdogReset | boardlink.FaultRejectedCommand, fault, "watchdog_reset,rejected_command",
		},
		{"unconfigured with a bit", boardlink.StateUnconfigured, boardlink.FaultWatchdogReset, fault,
			"unconfigured; watchdog_reset"},
		{"unknown state", boardlink.State(9), 0, fault, "unknown board state 9"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()

			st := picolink.MotorStatusFor(boardlink.Status{State: tc.state, Faults: tc.faults})
			if st.GetState() != tc.wantState || st.GetDetail() != tc.wantDetail {
				t.Errorf("got %v %q, want %v %q", st.GetState(), st.GetDetail(), tc.wantState, tc.wantDetail)
			}
		})
	}
}

func TestFaultString_UnknownBitsAreKept(t *testing.T) {
	t.Parallel()

	if got := picolink.FaultString(boardlink.FaultActuator | 0x80); got != "actuator,unknown(0x80)" {
		t.Errorf("FaultString = %q", got)
	}
	if got := picolink.FaultString(0); got != "" {
		t.Errorf("FaultString(0) = %q, want empty", got)
	}
}

// Counts become the accumulated wheel angle and the smoothed wheel rate the
// Zero publishes: 60 counts per rev, one revolution in 100 ms of board time
// is 600 rpm raw, 180 rpm after the estimator's first 0.3 smoothing step.
func TestOdometry_PublishedAsJointStates(t *testing.T) {
	t.Parallel()

	for _, tc := range []struct {
		name   string
		invert bool
		sign   float64
	}{
		{"as counted", false, 1},
		{"inverted", true, -1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()

			h := startSession(t, picolink.SessionConfig{
				Board:   sampleBoard,
				Encoder: &picolink.EncoderParams{CountsPerRev: 60, Invert: tc.invert},
			})
			h.board.send(t, boardlink.Packet{Type: boardlink.TypeOdometry,
				Odometry: boardlink.Odometry{BoardTimeUS: 1_000_000, Counts: 0}})
			if js := h.nextJoints(t); js.GetPosition()[0] != 0 || js.GetVelocity()[0] != 0 {
				t.Errorf("seed sample = %v, want zero position and velocity", js)
			}

			h.board.send(t, boardlink.Packet{Type: boardlink.TypeOdometry,
				Odometry: boardlink.Odometry{BoardTimeUS: 1_100_000, Counts: 60}})
			js := h.nextJoints(t)
			if len(js.GetName()) != 1 || js.GetName()[0] != actuationv1.DriveJoint ||
				js.GetFrameId() != picolink.FrameID {
				t.Fatalf("JointStates names/frame = %v %q", js.GetName(), js.GetFrameId())
			}
			if got, want := js.GetPosition()[0], tc.sign*2*math.Pi; math.Abs(got-want) > floatTol {
				t.Errorf("position = %v rad, want %v", got, want)
			}
			if got, want := js.GetVelocity()[0], tc.sign*180*2*math.Pi/60; math.Abs(got-want) > floatTol {
				t.Errorf("velocity = %v rad/s, want %v", got, want)
			}
		})
	}
}

// Without an encoder profile Odometry is dropped, as the Zero publishes no
// joint_states without one, and the session keeps serving Status.
func TestOdometry_SkippedWithoutEncoder(t *testing.T) {
	t.Parallel()

	h := startSession(t, picolink.SessionConfig{Board: sampleBoard})
	h.board.send(t, boardlink.Packet{Type: boardlink.TypeOdometry, Odometry: boardlink.Odometry{Counts: 5}})
	h.board.send(t, boardlink.Packet{Type: boardlink.TypeStatus, Status: boardlink.Status{State: boardlink.StateIdle}})
	if st := h.nextStatus(t); st.GetState() != actuationv1.MotorStatus_STATE_IDLE {
		t.Errorf("state = %v", st.GetState())
	}
}

func TestClockFrom(t *testing.T) {
	t.Parallel()

	rtt, offset := picolink.ClockFrom(1_000, 3_000, 50_000)
	if rtt != 2*time.Millisecond {
		t.Errorf("rtt = %v, want 2ms", rtt)
	}
	// Midpoint 2_000: board minus host midpoint.
	if offset != 48_000 {
		t.Errorf("offset = %d us, want 48000", offset)
	}
	c := picolink.ClockSync{Epoch: time.Unix(100, 0), OffsetUS: offset}
	if got, want := c.HostTime(50_000), time.Unix(100, 0).Add(2*time.Millisecond); !got.Equal(want) {
		t.Errorf("HostTime = %v, want %v", got, want)
	}
}

// The host pings on its own, and a Pong yields the offset: a board running
// 5 s ahead reads as 5 s minus half the round trip. A Pong echoing a host
// time from the future is ignored.
func TestPing_PongSetsClock(t *testing.T) {
	t.Parallel()

	h := startSession(t, picolink.SessionConfig{Board: sampleBoard, PingInterval: 10 * time.Millisecond})
	const ahead = 5_000_000

	h.board.send(t, boardlink.Packet{Type: boardlink.TypePong, Pong: boardlink.Pong{
		HostTimeUS: math.MaxUint64 / 2, BoardTimeUS: 1,
	}})
	ping := h.board.expect(t, boardlink.TypePing).Ping
	if _, ok := h.session.Clock(); ok {
		t.Fatal("a pong echoing a future host time set the clock")
	}

	h.board.send(t, boardlink.Packet{Type: boardlink.TypePong, Pong: boardlink.Pong{
		HostTimeUS: ping.HostTimeUS, BoardTimeUS: ping.HostTimeUS + ahead,
	}})
	deadline := time.Now().Add(waitTimeout)
	var (
		c  picolink.ClockSync
		ok bool
	)
	for !ok && time.Now().Before(deadline) {
		c, ok = h.session.Clock()
		time.Sleep(time.Millisecond)
	}
	if !ok {
		t.Fatal("no clock estimate after a pong")
	}
	halfRTT := c.RTT.Microseconds() / 2
	if c.OffsetUS != ahead-halfRTT {
		t.Errorf("offset = %d us with rtt %v, want %d", c.OffsetUS, c.RTT, ahead-halfRTT)
	}
}

// Garbage on the line (noise without a delimiter, a run too long for any
// frame, a corrupted frame) is dropped and counted, and the next Hello is
// still answered.
func TestGarbage_DoesNotBreakTheLink(t *testing.T) {
	t.Parallel()

	h := startSession(t, picolink.SessionConfig{Board: sampleBoard})

	h.board.write(t, []byte{0x13, 0x37, 0xFF, 0x00, 0x02, 0x01, 0x00})
	h.board.write(t, append(bytesOf(0xAA, 3*boardlink.MaxEncodedLen), 0))
	corrupt, err := boardlink.Append(nil, &boardlink.Packet{Type: boardlink.TypeHello})
	if err != nil {
		t.Fatal(err)
	}
	corrupt[2] ^= 0x40
	h.board.write(t, corrupt)

	h.board.send(t, boardlink.Packet{Type: boardlink.TypeHello, Hello: boardlink.Hello{
		ProtocolVersion: boardlink.Version, BootID: 7,
	}})
	if got := h.board.expect(t, boardlink.TypeConfig); got.Config != sampleBoard {
		t.Errorf("Config after garbage = %+v", got.Config)
	}
	if h.session.Dropped() < 3 {
		t.Errorf("Dropped = %d, want at least 3", h.session.Dropped())
	}
}

// A silent link is reported once, as a WARN and a FAULT MotorStatus, and the
// next frame restores it.
func TestLinkLoss_ReportedOnceAndRestored(t *testing.T) {
	t.Parallel()

	h := startSession(t, picolink.SessionConfig{Board: sampleBoard, LinkTimeout: 40 * time.Millisecond})

	st := h.nextStatus(t)
	if st.GetState() != actuationv1.MotorStatus_STATE_FAULT || !strings.HasPrefix(st.GetDetail(), "link lost") {
		t.Fatalf("status on silence = %v %q, want FAULT link lost", st.GetState(), st.GetDetail())
	}
	h.waitLog(t, "level=WARN", "no frame from the board")

	time.Sleep(100 * time.Millisecond)
	select {
	case again := <-h.status:
		t.Fatalf("link loss reported twice in one silence: %v", again)
	default:
	}

	h.board.send(t, boardlink.Packet{Type: boardlink.TypeStatus, Status: boardlink.Status{State: boardlink.StateIdle}})
	h.waitLog(t, "level=INFO", "link restored")
}

// A closed link is an error, so the supervisor restarts the target and
// reopens the port.
func TestRun_LinkClosedIsAnError(t *testing.T) {
	t.Parallel()

	h := startSession(t, picolink.SessionConfig{Board: sampleBoard})
	_ = h.board.conn.Close()

	select {
	case err := <-h.done:
		if err == nil || !isClosed(err) {
			t.Errorf("Run = %v, want a closed-link error", err)
		}
		h.done <- err // for the cleanup's wait
	case <-time.After(waitTimeout):
		t.Fatal("Run did not return after the link closed")
	}
}

func bytesOf(b byte, n int) []byte {
	out := make([]byte, n)
	for i := range out {
		out[i] = b
	}
	return out
}
