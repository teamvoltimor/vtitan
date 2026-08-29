package motor_test

import (
	"context"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/motor"
)

// testHarness bundles a Controller with the fakes wired into it, so tests
// don't have to juggle five separate return values from a constructor.
type testHarness struct {
	ctrl *motor.Controller
	rpwm *fakeDutyWriter
	lpwm *fakeDutyWriter
	rEn  *fakeEnableWriter
	lEn  *fakeEnableWriter
	log  *callLog
}

func newTestHarness(invert bool) *testHarness {
	log := &callLog{}
	rpwm := newFakeDutyWriter("rpwm", log)
	lpwm := newFakeDutyWriter("lpwm", log)
	rEn := newFakeEnableWriter("rEn", log)
	lEn := newFakeEnableWriter("lEn", log)
	return &testHarness{
		ctrl: motor.NewController(rpwm, lpwm, rEn, lEn, invert),
		rpwm: rpwm,
		lpwm: lpwm,
		rEn:  rEn,
		lEn:  lEn,
		log:  log,
	}
}

// TestController_Connect_ZerosBothChannelsBeforeEnabling is the actual point
// of this package: it encodes the real hardware bug fixed in commit
// f0fc617b ("assert BTS7960 R_EN/L_EN only after PWM channels confirm 0
// duty") -- a boot-time full-speed-reverse motor kick caused by enabling
// R_EN/L_EN before the PWM channels were confirmed at 0 duty.
//
// This test was verified to actually catch that regression, not just pass
// tautologically: reordering Controller.Connect's four writes so
// rEn.SetHigh(true)/lEn.SetHigh(true) run before rpwm.SetDuty(0)/
// lpwm.SetDuty(0) (i.e. reintroducing the pre-f0fc617b bug) was tried
// locally and this test failed against that reordering, exactly as
// expected, before the fix was restored -- see the task report for the
// verification transcript.
func TestController_Connect_ZerosBothChannelsBeforeEnabling(t *testing.T) {
	t.Parallel()

	h := newTestHarness(false)

	if err := h.ctrl.Connect(context.Background()); err != nil {
		t.Fatalf("Connect() error = %v, want nil", err)
	}

	want := []string{
		"rpwm.SetDuty(0.000)",
		"lpwm.SetDuty(0.000)",
		"rEn.SetHigh(true)",
		"lEn.SetHigh(true)",
	}
	got := h.log.entries()
	if len(got) != len(want) {
		t.Fatalf("Connect() call log = %v, want %v", got, want)
	}
	for i, entry := range want {
		if got[i] != entry {
			t.Fatalf("Connect() call log[%d] = %q, want %q (full log: %v)", i, got[i], entry, got)
		}
	}
}

// TestController_Connect_NeverEnablesBeforeZeroingEitherChannel is a second,
// order-agnostic angle on the same property: however Connect sequences its
// writes internally, no enable call may appear before both duty-zero calls
// have already been recorded.
func TestController_Connect_NeverEnablesBeforeZeroingEitherChannel(t *testing.T) {
	t.Parallel()

	h := newTestHarness(false)

	if err := h.ctrl.Connect(context.Background()); err != nil {
		t.Fatalf("Connect() error = %v, want nil", err)
	}

	zeroedRPWM, zeroedLPWM := false, false
	for _, entry := range h.log.entries() {
		switch entry {
		case "rpwm.SetDuty(0.000)":
			zeroedRPWM = true
		case "lpwm.SetDuty(0.000)":
			zeroedLPWM = true
		case "rEn.SetHigh(true)", "lEn.SetHigh(true)":
			if !zeroedRPWM || !zeroedLPWM {
				t.Fatalf("%q recorded before both PWM channels confirmed at 0 duty (log so far: %v)",
					entry, h.log.entries())
			}
		}
	}
}

// TestController_SetSpeed_NeverBothChannelsNonzero drives the controller
// through a mix of forward/reverse/zero commands and asserts the Fast-Brake
// invariant (see splitDuty's doc comment) holds after every single one --
// not just that the final state is safe.
func TestController_SetSpeed_NeverBothChannelsNonzero(t *testing.T) {
	t.Parallel()

	h := newTestHarness(false)
	ctx := context.Background()

	if err := h.ctrl.Connect(ctx); err != nil {
		t.Fatalf("Connect() error = %v, want nil", err)
	}

	speeds := []float64{0.5, -0.3, 0.9, 0, -1.0, 1.0, 0}
	for _, speed := range speeds {
		if err := h.ctrl.SetSpeed(ctx, speed); err != nil {
			t.Fatalf("SetSpeed(%v) error = %v, want nil", speed, err)
		}
		if h.rpwm.lastDuty() != 0 && h.lpwm.lastDuty() != 0 {
			t.Fatalf("after SetSpeed(%v): rpwm=%v lpwm=%v, both nonzero -- Fast Brake fault state",
				speed, h.rpwm.lastDuty(), h.lpwm.lastDuty())
		}
	}
}

// TestController_SetSpeed_ZeroesOutgoingChannelBeforeRaisingIncoming asserts
// the write *order* within a single direction change, not just the settled
// end state: the channel becoming inactive is always zeroed before the
// newly active channel's duty is raised.
func TestController_SetSpeed_ZeroesOutgoingChannelBeforeRaisingIncoming(t *testing.T) {
	t.Parallel()

	h := newTestHarness(false)
	ctx := context.Background()

	if err := h.ctrl.Connect(ctx); err != nil {
		t.Fatalf("Connect() error = %v, want nil", err)
	}
	h.log.calls = nil // discard Connect's own log entries, isolate the transition below

	if err := h.ctrl.SetSpeed(ctx, 0.5); err != nil {
		t.Fatalf("SetSpeed(0.5) error = %v, want nil", err)
	}
	if err := h.ctrl.SetSpeed(ctx, -0.3); err != nil {
		t.Fatalf("SetSpeed(-0.3) error = %v, want nil", err)
	}

	want := []string{
		"lpwm.SetDuty(0.000)", // forward: zero the outgoing (LPWM) channel first
		"rpwm.SetDuty(0.500)", // then raise the incoming (RPWM) channel
		"rpwm.SetDuty(0.000)", // reverse: zero the outgoing (RPWM) channel first
		"lpwm.SetDuty(0.300)", // then raise the incoming (LPWM) channel
	}
	got := h.log.entries()
	if len(got) != len(want) {
		t.Fatalf("call log = %v, want %v", got, want)
	}
	for i, entry := range want {
		if got[i] != entry {
			t.Fatalf("call log[%d] = %q, want %q (full log: %v)", i, got[i], entry, got)
		}
	}
}

func TestController_SetSpeed_BeforeConnect_ReturnsError(t *testing.T) {
	t.Parallel()

	h := newTestHarness(false)

	if err := h.ctrl.SetSpeed(context.Background(), 0.5); err == nil {
		t.Fatal("SetSpeed() before Connect: got nil error, want an error")
	}
}

func TestController_SetSpeed_ClampsOutOfRangeInput(t *testing.T) {
	t.Parallel()

	h := newTestHarness(false)
	ctx := context.Background()

	if err := h.ctrl.Connect(ctx); err != nil {
		t.Fatalf("Connect() error = %v, want nil", err)
	}
	if err := h.ctrl.SetSpeed(ctx, 2.5); err != nil {
		t.Fatalf("SetSpeed(2.5) error = %v, want nil", err)
	}
	if got := h.rpwm.lastDuty(); got != 1.0 {
		t.Errorf("rpwm duty after SetSpeed(2.5) = %v, want 1.0 (clamped)", got)
	}
}

func TestController_SetSpeed_InvertFlipsSign(t *testing.T) {
	t.Parallel()

	h := newTestHarness(true)
	ctx := context.Background()

	if err := h.ctrl.Connect(ctx); err != nil {
		t.Fatalf("Connect() error = %v, want nil", err)
	}
	if err := h.ctrl.SetSpeed(ctx, 0.5); err != nil {
		t.Fatalf("SetSpeed(0.5) error = %v, want nil", err)
	}

	if h.rpwm.lastDuty() != 0 {
		t.Errorf("inverted SetSpeed(0.5): rpwm duty = %v, want 0", h.rpwm.lastDuty())
	}
	if h.lpwm.lastDuty() != 0.5 {
		t.Errorf("inverted SetSpeed(0.5): lpwm duty = %v, want 0.5", h.lpwm.lastDuty())
	}
}

func TestController_Close_ZeroesDutyAndDisablesEnableLines(t *testing.T) {
	t.Parallel()

	h := newTestHarness(false)
	ctx := context.Background()

	if err := h.ctrl.Connect(ctx); err != nil {
		t.Fatalf("Connect() error = %v, want nil", err)
	}
	if err := h.ctrl.SetSpeed(ctx, 0.8); err != nil {
		t.Fatalf("SetSpeed(0.8) error = %v, want nil", err)
	}
	if err := h.ctrl.Close(ctx); err != nil {
		t.Fatalf("Close() error = %v, want nil", err)
	}

	if h.rpwm.lastDuty() != 0 {
		t.Errorf("rpwm duty after Close() = %v, want 0", h.rpwm.lastDuty())
	}
	if h.lpwm.lastDuty() != 0 {
		t.Errorf("lpwm duty after Close() = %v, want 0", h.lpwm.lastDuty())
	}
	if h.rEn.isHigh() {
		t.Error("rEn still high after Close()")
	}
	if h.lEn.isHigh() {
		t.Error("lEn still high after Close()")
	}

	if err := h.ctrl.SetSpeed(ctx, 0.1); err == nil {
		t.Error("SetSpeed() after Close(): got nil error, want an error")
	}
}
