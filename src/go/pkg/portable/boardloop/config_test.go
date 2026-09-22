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

func invalidConfigs() map[string]boardlink.Config {
	mut := func(f func(c *boardlink.Config)) boardlink.Config {
		c := validConfig()
		f(&c)
		return c
	}
	nan := float32(math.NaN())
	return map[string]boardlink.Config{
		"zero command timeout": mut(func(c *boardlink.Config) { c.CommandTimeoutMS = 0 }),
		"zero hw watchdog":     mut(func(c *boardlink.Config) { c.HardwareWatchdogMS = 0 }),
		"zero status interval": mut(func(c *boardlink.Config) { c.StatusIntervalMS = 0 }),
		"NaN linkage":          mut(func(c *boardlink.Config) { c.LinkageRatio = nan }),
		"zero servo max":       mut(func(c *boardlink.Config) { c.ServoMaxAngleDeg = 0 }),
		"negative speed scale": mut(func(c *boardlink.Config) { c.SpeedScalePctPerMPS = -testSpeedScale }),
		"pulse in ms":          mut(func(c *boardlink.Config) { c.ServoMinPulseUS, c.ServoMaxPulseUS = 1, 2 }),
		"min above max":        mut(func(c *boardlink.Config) { c.ServoMinPulseUS = testMaxPulseUS }),
		"center outside":       mut(func(c *boardlink.Config) { c.ServoCenterPulseUS = testMaxPulseUS + 1 }),
		"NaN center":           mut(func(c *boardlink.Config) { c.ServoCenterPulseUS = nan }),
		"zero servo range":     mut(func(c *boardlink.Config) { c.ServoRangeDeg = 0 }),
	}
}

func TestConfig_InvalidLeavesBoardUnconfigured(t *testing.T) {
	t.Parallel()

	if err := boardloop.ValidateConfig(validConfig()); err != nil {
		t.Fatalf("ValidateConfig(valid) = %v", err)
	}
	for name, cfg := range invalidConfigs() {
		t.Run(name, func(t *testing.T) {
			t.Parallel()

			if err := boardloop.ValidateConfig(cfg); !errors.Is(err, boardloop.ErrInvalidConfig) {
				t.Errorf("ValidateConfig = %v, want ErrInvalidConfig", err)
			}
			h := newHarness(t, 0, false)
			h.queueConfig(cfg)
			h.queueCommand(testSpeedMPS, testSteerRad)
			h.run(time.Second)
			if _, ok := h.loop.Config(); ok {
				t.Fatal("invalid Config configured the board")
			}
			if len(h.ev.list) != 0 {
				t.Fatalf("invalid Config touched hardware: %v", h.ev.list)
			}
			if len(h.sent(boardlink.TypeHello)) < 4 {
				t.Error("board stopped repeating Hello after an invalid Config")
			}
		})
	}
}

// TestConfig_InvalidWhileConfiguredStopsAndUnconfigures: the board safety-
// stops on the calibration it had, then drops back to Hello and ignores
// commands.
func TestConfig_InvalidWhileConfiguredStopsAndUnconfigures(t *testing.T) {
	t.Parallel()

	h := newHarness(t, 0, false)
	h.configure()
	h.queueCommand(testSpeedMPS, testSteerRad)
	h.step()
	h.reset()

	bad := validConfig()
	bad.CommandTimeoutMS = 0
	h.queueConfig(bad)
	h.step()
	want := []string{"drive 0.000", wantCenterMsg}
	if !slices.Equal(h.ev.list, want) {
		t.Fatalf("writes on invalid reconfig = %v, want %v", h.ev.list, want)
	}
	if _, ok := h.loop.Config(); ok {
		t.Fatal("board still configured after an invalid Config")
	}
	if len(h.sent(boardlink.TypeHello)) != 1 {
		t.Error("no Hello right after the refused Config")
	}

	h.reset()
	h.queueCommand(testSpeedMPS, testSteerRad)
	h.run(time.Second)
	if len(h.ev.list) != 0 {
		t.Fatalf("unconfigured board acted: %v", h.ev.list)
	}
}

func TestConfig_ReconfigRestartsFromRestWithoutReconnecting(t *testing.T) {
	t.Parallel()

	h := newHarness(t, 0, false)
	h.configure()
	h.queueCommand(testSpeedMPS, testSteerRad)
	h.step()
	h.reset()

	cfg := validConfig()
	cfg.ServoCenterPulseUS = 1520
	cfg.InvertDrive = true
	h.queueConfig(cfg)
	h.step()
	want := []string{"drive -0.000", "servo 1520.0"}
	if !slices.Equal(h.ev.list, want) && !slices.Equal(h.ev.list, []string{"drive 0.000", "servo 1520.0"}) {
		t.Fatalf("writes on reconfig = %v, want %v", h.ev.list, want)
	}
	if h.drive.connects != 1 {
		t.Errorf("Connect calls = %d, want 1 (reconfig must not reconnect)", h.drive.connects)
	}

	h.reset()
	h.queueCommand(testSpeedMPS, 0)
	h.step()
	if len(h.drive.speeds) != 1 || !near(h.drive.speeds[0], -wantDuty) {
		t.Fatalf("inverted drive speeds = %v, want [%v]", h.drive.speeds, -wantDuty)
	}
	h.run(testStatusMS * time.Millisecond)
	if st := h.lastStatus(); !near(float64(st.Duty), wantDuty) {
		t.Errorf("Status.Duty = %v, want the commanded %v before inversion", st.Duty, wantDuty)
	}
}

func TestConfig_ConnectFailureStaysUnconfiguredAndRetries(t *testing.T) {
	t.Parallel()

	h := newHarness(t, 0, false)
	h.drive.connectErr = errActuator
	h.queueConfig(validConfig())
	h.run(time.Second)
	if _, ok := h.loop.Config(); ok {
		t.Fatal("configured although the drive failed to connect")
	}
	hellos := h.sent(boardlink.TypeHello)
	if len(hellos) == 0 || hellos[len(hellos)-1].Hello.Faults&boardlink.FaultActuator == 0 {
		t.Fatal("Hello does not report FaultActuator after a failed connect")
	}
	if slices.Contains(h.ev.list, wantCenterMsg) {
		t.Fatalf("servo written although the Config was not applied: %v", h.ev.list)
	}

	h.drive.connectErr = nil
	h.configure()
	if h.drive.connects != 2 {
		t.Errorf("Connect calls = %d, want 2", h.drive.connects)
	}
}

// TestConfig_ReconfigStartsAFreshSession: a repeated, identical Config still
// re-asserts the center pulse, and the command accepted before it no longer
// arms the watchdog: the new session starts stopped.
func TestConfig_ReconfigStartsAFreshSession(t *testing.T) {
	t.Parallel()

	h := newHarness(t, 0, false)
	h.configure()
	h.queueCommand(testSpeedMPS, 0)
	h.step()
	h.reset()

	h.queueConfig(validConfig())
	h.step()
	want := []string{"drive 0.000", wantCenterMsg}
	if !slices.Equal(h.ev.list, want) {
		t.Fatalf("writes on identical reconfig = %v, want %v", h.ev.list, want)
	}
	h.run(2 * testTimeoutMS * time.Millisecond)
	if !slices.Equal(h.ev.list, want) {
		t.Fatalf("the previous session's command fired the watchdog after reconfig: %v", h.ev.list)
	}
}
