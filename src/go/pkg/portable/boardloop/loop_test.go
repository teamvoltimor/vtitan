package boardloop_test

import (
	"errors"
	"math"
	"slices"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardloop"
)

// Expected conversions of the test calibration: 0.2 rad of wheel is
// 11.459 deg, 22.918 deg of servo through a 0.5 linkage, and
// 1500 + 22.918/270*2000 us of pulse; 0.5 m/s at 30 %/(m/s) is 0.15 duty.
const (
	testSteerRad  = 0.2
	testSpeedMPS  = 0.5
	wantServoDeg  = 22.918312
	wantPulseUS   = 1669.7653
	wantDuty      = 0.15
	wantCenterMsg = "servo 1500.0"
)

var errActuator = errors.New("actuator write failed")

func TestNew_RequiresLinkDriveAndServo(t *testing.T) {
	t.Parallel()

	_, err := boardloop.New(boardloop.Hardware{Link: nil, Drive: nil, Servo: nil, Encoder: nil},
		boardloop.Options{BootID: 0, BootFaults: 0})
	if !errors.Is(err, boardloop.ErrMissingHardware) {
		t.Fatalf("New with no hardware: err = %v, want ErrMissingHardware", err)
	}
}

func TestHello_RepeatsUntilConfigured(t *testing.T) {
	t.Parallel()

	h := newHarness(t, boardlink.FaultWatchdogReset, false)
	h.step()
	h.run(time.Second)

	hellos := h.sent(boardlink.TypeHello)
	// At 1, 251, 501, 751 and 1001 ms.
	if len(hellos) != 5 {
		t.Fatalf("Hello count over 1 s = %d, want 5 (every %v)", len(hellos), boardloop.HelloInterval)
	}
	for _, p := range hellos {
		want := boardlink.Hello{
			ProtocolVersion: boardlink.Version,
			BootID:          testBootID,
			Faults:          boardlink.FaultWatchdogReset,
		}
		if p.Hello != want {
			t.Fatalf("Hello = %+v, want %+v", p.Hello, want)
		}
	}

	h.configure()
	h.run(time.Second)
	if n := len(h.sent(boardlink.TypeHello)); n != 0 {
		t.Fatalf("configured board sent %d Hello, want 0", n)
	}
	if got := h.lastStatus().Faults; got&boardlink.FaultWatchdogReset == 0 {
		t.Errorf("Status.Faults = %#x, want FaultWatchdogReset kept for the boot", got)
	}
}

func TestBeforeConfig_NoActuation(t *testing.T) {
	t.Parallel()

	h := newHarness(t, 0, true)
	for range 10 {
		h.queueCommand(testSpeedMPS, testSteerRad)
		h.run(50 * time.Millisecond)
	}
	if len(h.ev.list) != 0 {
		t.Fatalf("unconfigured board touched hardware: %v", h.ev.list)
	}
	if got := h.loop.Counters().CommandsIgnored; got != 10 {
		t.Errorf("CommandsIgnored = %d, want 10", got)
	}
	if n := len(h.sent(boardlink.TypeStatus)) + len(h.sent(boardlink.TypeOdometry)); n != 0 {
		t.Errorf("unconfigured board sent %d Status/Odometry, want 0", n)
	}
}

func TestConfig_ConnectsThenStopsAndCenters(t *testing.T) {
	t.Parallel()

	h := newHarness(t, 0, false)
	h.queueConfig(validConfig())
	h.step()

	want := []string{"connect", "drive 0.000", wantCenterMsg}
	if !slices.Equal(h.ev.list, want) {
		t.Fatalf("writes on Config = %v, want %v", h.ev.list, want)
	}
	st := h.lastStatus()
	if st.State != boardlink.StateIdle {
		t.Errorf("Status.State after Config = %v, want Idle", st.State)
	}
}

func TestCommand_SteersThenDrives(t *testing.T) {
	t.Parallel()

	h := newHarness(t, 0, false)
	h.configure()
	h.queueCommand(testSpeedMPS, testSteerRad)
	h.step()

	if len(h.ev.list) != 2 || h.ev.list[0][:5] != "servo" || h.ev.list[1][:5] != "drive" {
		t.Fatalf("writes = %v, want servo then drive", h.ev.list)
	}
	if !near(h.servo.pulses[0], wantPulseUS) {
		t.Errorf("pulse = %v us, want %v", h.servo.pulses[0], wantPulseUS)
	}
	if !near(h.drive.speeds[0], wantDuty) {
		t.Errorf("duty = %v, want %v", h.drive.speeds[0], wantDuty)
	}

	h.run(testStatusMS * time.Millisecond)
	st := h.lastStatus()
	if st.State != boardlink.StateRunning || !near(float64(st.Duty), wantDuty) ||
		!near(float64(st.ServoAngleDeg), wantServoDeg) {
		t.Errorf("Status = %+v, want Running at duty %v, servo %v deg", st, wantDuty, wantServoDeg)
	}
}

func TestCommand_SamePulseIsNotRewritten(t *testing.T) {
	t.Parallel()

	h := newHarness(t, 0, false)
	h.configure()
	for range 3 {
		h.queueCommand(testSpeedMPS, testSteerRad)
		h.step()
	}
	if len(h.servo.pulses) != 1 {
		t.Errorf("servo writes for 3 identical commands = %d, want 1", len(h.servo.pulses))
	}
	if len(h.drive.speeds) != 3 {
		t.Errorf("drive writes for 3 commands = %d, want 3", len(h.drive.speeds))
	}
}

func TestCommand_NonFiniteRejectedWhole(t *testing.T) {
	t.Parallel()

	nan := float32(math.NaN())
	inf := float32(math.Inf(1))
	for _, tc := range []struct {
		name         string
		speed, steer float32
	}{
		{"NaN speed", nan, testSteerRad},
		{"+Inf speed", inf, testSteerRad},
		{"NaN steer", testSpeedMPS, nan},
		{"-Inf steer", testSpeedMPS, -inf},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()

			h := newHarness(t, 0, false)
			h.configure()
			h.queueCommand(tc.speed, tc.steer)
			h.step()
			if len(h.ev.list) != 0 {
				t.Fatalf("non-finite command wrote %v, want nothing", h.ev.list)
			}

			h.run(testStatusMS * time.Millisecond)
			if got := h.lastStatus().Faults; got&boardlink.FaultRejectedCommand == 0 {
				t.Errorf("Status.Faults = %#x, want FaultRejectedCommand", got)
			}
			h.run(testStatusMS * time.Millisecond)
			if got := h.lastStatus().Faults; got&boardlink.FaultRejectedCommand != 0 {
				t.Errorf("FaultRejectedCommand still set in the next Status (%#x)", got)
			}
		})
	}
}

// TestCommand_NonFiniteDoesNotRefreshWatchdog: a stream of rejected
// commands stops the drive exactly as silence would.
func TestCommand_NonFiniteDoesNotRefreshWatchdog(t *testing.T) {
	t.Parallel()

	h := newHarness(t, 0, false)
	h.configure()
	h.queueCommand(testSpeedMPS, testSteerRad)
	h.step()
	h.reset()

	for range 5 {
		h.run(100 * time.Millisecond)
		h.queueCommand(float32(math.NaN()), 0)
	}
	h.step()
	want := []string{"drive 0.000", wantCenterMsg}
	if !slices.Equal(h.ev.list, want) {
		t.Fatalf("writes %v after %v of non-finite commands = %v, want %v",
			h.now, testTimeoutMS*time.Millisecond, h.ev.list, want)
	}
}

func TestWatchdog_StopsAndCentersOncePerEpisode(t *testing.T) {
	t.Parallel()

	h := newHarness(t, 0, false)
	h.configure()
	h.queueCommand(testSpeedMPS, testSteerRad)
	h.step()
	h.reset()

	h.run(testTimeoutMS*time.Millisecond - stepTick)
	if len(h.ev.list) != 0 {
		t.Fatalf("watchdog acted before the timeout: %v", h.ev.list)
	}
	h.step()
	want := []string{"drive 0.000", wantCenterMsg}
	if !slices.Equal(h.ev.list, want) {
		t.Fatalf("writes at the timeout = %v, want %v", h.ev.list, want)
	}
	st := h.lastStatus()
	if st.State != boardlink.StateIdle || st.CommandAgeMS < testTimeoutMS || st.Duty != 0 {
		t.Errorf("Status at the timeout = %+v, want Idle, zero duty, age >= %d ms", st, testTimeoutMS)
	}

	h.run(3 * time.Second)
	if !slices.Equal(h.ev.list, want) {
		t.Fatalf("watchdog acted again in the same episode: %v", h.ev.list)
	}
	if got := h.loop.Counters().WatchdogStops; got != 1 {
		t.Errorf("WatchdogStops = %d, want 1", got)
	}

	// A fresh command re-arms it for a second episode.
	h.reset()
	h.queueCommand(testSpeedMPS, testSteerRad)
	h.step()
	h.run(testTimeoutMS * time.Millisecond)
	want = []string{"servo 1669.8", "drive 0.150", "drive 0.000", wantCenterMsg}
	if !slices.Equal(h.ev.list, want) {
		t.Fatalf("second episode writes = %v, want %v", h.ev.list, want)
	}
}

func TestWatchdog_QuietBeforeFirstCommand(t *testing.T) {
	t.Parallel()

	h := newHarness(t, 0, false)
	h.configure()
	h.run(5 * testTimeoutMS * time.Millisecond)
	if len(h.ev.list) != 0 {
		t.Fatalf("watchdog acted with no command ever accepted: %v", h.ev.list)
	}
}

func TestPing_AnsweredBeforeAndAfterConfig(t *testing.T) {
	t.Parallel()

	const hostTime = 1 << 40
	h := newHarness(t, 0, false)
	for i, configured := range []bool{false, true} {
		if configured {
			h.configure()
		}
		h.queue(boardlink.Packet{Type: boardlink.TypePing, Ping: boardlink.Ping{HostTimeUS: hostTime + uint64(i)}})
		h.step()
		pongs := h.sent(boardlink.TypePong)
		if len(pongs) == 0 {
			t.Fatalf("configured=%v: no Pong", configured)
		}
		got := pongs[len(pongs)-1].Pong
		want := boardlink.Pong{HostTimeUS: hostTime + uint64(i), BoardTimeUS: uint64(h.now.Microseconds())}
		if got != want {
			t.Errorf("configured=%v: Pong = %+v, want %+v", configured, got, want)
		}
	}
}

func TestStatusAndOdometry_Cadence(t *testing.T) {
	t.Parallel()

	h := newHarness(t, 0, true)
	h.configure()
	h.enc.counts = -4242
	h.run(time.Second)

	if n := len(h.sent(boardlink.TypeStatus)); n != 1000/testStatusMS {
		t.Errorf("Status over 1 s = %d, want %d", n, 1000/testStatusMS)
	}
	odo := h.sent(boardlink.TypeOdometry)
	if len(odo) != 1000/testOdometryMS {
		t.Fatalf("Odometry over 1 s = %d, want %d", len(odo), 1000/testOdometryMS)
	}
	last := odo[len(odo)-1].Odometry
	if last.Counts != -4242 || last.BoardTimeUS == 0 {
		t.Errorf("Odometry = %+v, want counts -4242 and a timestamp", last)
	}
}

func TestOdometry_NoneWithoutEncoderOrInterval(t *testing.T) {
	t.Parallel()

	h := newHarness(t, 0, false)
	h.configure()
	h.run(time.Second)
	if n := len(h.sent(boardlink.TypeOdometry)); n != 0 {
		t.Errorf("no encoder: %d Odometry, want 0", n)
	}

	h = newHarness(t, 0, true)
	cfg := validConfig()
	cfg.OdometryIntervalMS = 0
	h.queueConfig(cfg)
	h.run(time.Second)
	if n := len(h.sent(boardlink.TypeOdometry)); n != 0 {
		t.Errorf("interval 0: %d Odometry, want 0", n)
	}
}

func TestSequence_IncrementsPerFrame(t *testing.T) {
	t.Parallel()

	h := newHarness(t, 0, true)
	h.run(time.Second)
	h.queueConfig(validConfig())
	h.run(time.Second)
	for i, p := range h.link.out {
		if p.Seq != uint16(i) {
			t.Fatalf("frame %d (%v) has seq %d, want %d", i, p.Type, p.Seq, i)
		}
	}
}

func TestStatus_ActuatorFault(t *testing.T) {
	t.Parallel()

	h := newHarness(t, 0, false)
	h.configure()
	h.drive.setErr = errActuator
	h.queueCommand(testSpeedMPS, testSteerRad)
	h.run(testStatusMS * time.Millisecond)
	st := h.lastStatus()
	if st.State != boardlink.StateFault || st.Faults&boardlink.FaultActuator == 0 {
		t.Fatalf("Status after a failed write = %+v, want Fault with FaultActuator", st)
	}

	h.drive.setErr = nil
	h.queueCommand(testSpeedMPS, testSteerRad)
	h.run(testStatusMS * time.Millisecond)
	if st = h.lastStatus(); st.State != boardlink.StateRunning || st.Faults != 0 {
		t.Errorf("Status after a good write = %+v, want Running and no faults", st)
	}
}

func TestServo_FailedWriteIsRetried(t *testing.T) {
	t.Parallel()

	h := newHarness(t, 0, false)
	h.configure()
	h.servo.setErr = errActuator
	h.queueCommand(0, testSteerRad)
	h.step()
	h.servo.setErr = nil
	h.queueCommand(0, testSteerRad)
	h.step()
	if len(h.servo.pulses) != 2 {
		t.Errorf("servo writes = %d, want 2 (the failed pulse retried)", len(h.servo.pulses))
	}
}

func TestLink_ResyncsAfterGarbage(t *testing.T) {
	t.Parallel()

	h := newHarness(t, 0, false)
	// A burst longer than any frame with no delimiter, then short
	// delimited junk, then half of a real frame cut off by a delimiter.
	for i := range 3 * boardlink.MaxEncodedLen {
		h.link.in = append(h.link.in, byte(i%250+1))
	}
	h.link.in = append(h.link.in, 0, 0x55, 0x13, 0x00, 0xFF, 0x01, 0x00)
	var cut boardlink.Packet
	cut.Type = boardlink.TypeCommand
	half, err := boardlink.Append(nil, &cut)
	if err != nil {
		t.Fatal(err)
	}
	h.link.in = append(h.link.in, half[:len(half)/2]...)
	h.link.in = append(h.link.in, 0)
	h.link.readErr = errActuator

	h.queueConfig(validConfig())
	h.run(10 * time.Millisecond)
	if _, ok := h.loop.Config(); !ok {
		t.Fatal("Config after a garbage burst was not applied")
	}
	c := h.loop.Counters()
	if c.FramesDropped == 0 || c.LinkReadErrors != 1 || c.ConfigsApplied != 1 {
		t.Errorf("Counters = %+v, want drops, 1 read error, 1 config", c)
	}
}

func TestLink_UnexpectedTypeIsCounted(t *testing.T) {
	t.Parallel()

	h := newHarness(t, 0, false)
	h.queue(boardlink.Packet{Type: boardlink.TypeStatus})
	h.step()
	if got := h.loop.Counters().FramesUnexpected; got != 1 {
		t.Errorf("FramesUnexpected = %d, want 1", got)
	}
}

// TestStep_DoesNotAllocate: at 50 Hz on a microcontroller, per-frame
// garbage is a GC pause. (TinyGo's AllocsPerRun is a stub returning 0, so
// this binds only under Go.)
//
//nolint:paralleltest // testing.AllocsPerRun panics inside a parallel test.
func TestStep_DoesNotAllocate(t *testing.T) {
	cfg := boardlink.Packet{Type: boardlink.TypeConfig, Config: validConfig()}
	cfgFrame, err := boardlink.Append(nil, &cfg)
	if err != nil {
		t.Fatal(err)
	}
	cmd := boardlink.Packet{
		Type:    boardlink.TypeCommand,
		Command: boardlink.Command{SpeedMPS: testSpeedMPS, SteeringAngleRad: testSteerRad},
	}
	cmdFrame, err := boardlink.Append(nil, &cmd)
	if err != nil {
		t.Fatal(err)
	}

	link := &nopLink{frame: cfgFrame, feed: true}
	loop, err := boardloop.New(
		boardloop.Hardware{Link: link, Drive: nopDrive{}, Servo: nopServo{}, Encoder: &fakeEncoder{}},
		boardloop.Options{BootID: 1, BootFaults: 0},
	)
	if err != nil {
		t.Fatal(err)
	}
	now := time.Millisecond
	loop.Step(now)
	link.frame = cmdFrame

	allocs := testing.AllocsPerRun(200, func() {
		link.feed = true
		now += 10 * time.Millisecond
		loop.Step(now)
	})
	if allocs != 0 {
		t.Errorf("Step allocates %v times per call, want 0", allocs)
	}
}
