package core_test

import (
	"errors"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/statemachine/core"
)

// TestTransitionTo_ValidTransitions exercises every (From, Reason, To)
// rule in the real transition table, one subtest per rule -- not just the
// happy BootCheck->Ready->Racing->Finished path, since the reset edges
// (SYSTEM_RESET from three different states) and the two ways into
// Finished from Racing are just as real and just as easy to silently
// break.
func TestTransitionTo_ValidTransitions(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name   string
		from   core.RobotState
		reason core.StateTransitionReason
		to     core.RobotState
	}{
		{"boot complete moves to ready", core.StateBootCheck, core.ReasonBootComplete, core.StateReady},
		{"boot failed moves to finished", core.StateBootCheck, core.ReasonBootFailed, core.StateFinished},
		{"button pressed starts racing", core.StateReady, core.ReasonButtonPressed, core.StateRacing},
		{"system reset from ready returns to boot check", core.StateReady, core.ReasonSystemReset, core.StateBootCheck},
		{"laps completed finishes the race", core.StateRacing, core.ReasonLapsCompleted, core.StateFinished},
		{"emergency stop finishes the race", core.StateRacing, core.ReasonEmergencyStop, core.StateFinished},
		{
			"system reset from racing returns to boot check",
			core.StateRacing,
			core.ReasonSystemReset,
			core.StateBootCheck,
		},
		{
			"system reset from finished returns to boot check",
			core.StateFinished,
			core.ReasonSystemReset,
			core.StateBootCheck,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			m := core.New()
			forceState(t, m, tt.from)

			applied, err := m.TransitionTo(tt.to, tt.reason)

			if !applied {
				t.Fatalf("TransitionTo(%v, %v) from %v: applied = false, want true", tt.to, tt.reason, tt.from)
			}
			if err != nil {
				t.Fatalf("TransitionTo(%v, %v) from %v: unexpected error: %v", tt.to, tt.reason, tt.from, err)
			}
			if got := m.CurrentState(); got != tt.to {
				t.Fatalf("CurrentState() = %v, want %v", got, tt.to)
			}
		})
	}
}

// TestTransitionTo_InvalidTransitions covers every state × reason
// combination NOT in the transition table -- the whole point of this
// package is that a transition that shouldn't be possible stays
// impossible, so this is exhaustive over the reasons rather than
// spot-checking a couple.
func TestTransitionTo_InvalidTransitions(t *testing.T) {
	t.Parallel()

	allReasons := []core.StateTransitionReason{
		core.ReasonBootComplete,
		core.ReasonBootFailed,
		core.ReasonButtonPressed,
		core.ReasonLapsCompleted,
		core.ReasonEmergencyStop,
		core.ReasonSystemReset,
	}
	allStates := []core.RobotState{
		core.StateBootCheck,
		core.StateReady,
		core.StateRacing,
		core.StateFinished,
	}

	// validTo reports the one state a (from, reason) pair is allowed to
	// reach, mirroring transitionTable without importing its internals --
	// any (from, reason, to) triple not covered here must be blocked.
	validTo := func(from core.RobotState, reason core.StateTransitionReason) (core.RobotState, bool) {
		switch {
		case from == core.StateBootCheck && reason == core.ReasonBootComplete:
			return core.StateReady, true
		case from == core.StateBootCheck && reason == core.ReasonBootFailed:
			return core.StateFinished, true
		case from == core.StateReady && reason == core.ReasonButtonPressed:
			return core.StateRacing, true
		case from == core.StateReady && reason == core.ReasonSystemReset:
			return core.StateBootCheck, true
		case from == core.StateRacing && reason == core.ReasonLapsCompleted:
			return core.StateFinished, true
		case from == core.StateRacing && reason == core.ReasonEmergencyStop:
			return core.StateFinished, true
		case from == core.StateRacing && reason == core.ReasonSystemReset:
			return core.StateBootCheck, true
		case from == core.StateFinished && reason == core.ReasonSystemReset:
			return core.StateBootCheck, true
		default:
			return core.RobotState(0), false
		}
	}

	for _, from := range allStates {
		for _, reason := range allReasons {
			for _, to := range allStates {
				wantTo, isValid := validTo(from, reason)
				if isValid && wantTo == to {
					continue // covered by TestTransitionTo_ValidTransitions
				}

				t.Run(from.String()+"_"+reason.String()+"_to_"+to.String(), func(t *testing.T) {
					t.Parallel()

					m := core.New()
					forceState(t, m, from)

					applied, err := m.TransitionTo(to, reason)

					if applied {
						t.Fatalf("TransitionTo(%v, %v) from %v: applied = true, want false (blocked)", to, reason, from)
					}
					if err != nil {
						t.Fatalf(
							"TransitionTo(%v, %v) from %v: unexpected error on a blocked transition: %v",
							to,
							reason,
							from,
							err,
						)
					}
					if got := m.CurrentState(); got != from {
						t.Fatalf("CurrentState() = %v after a blocked transition, want unchanged %v", got, from)
					}
				})
			}
		}
	}
}

// TestTransitionTo_CallbacksNotifiedInOrder confirms every registered
// callback runs, in registration order, with the transition it was
// registered for.
func TestTransitionTo_CallbacksNotifiedInOrder(t *testing.T) {
	t.Parallel()

	m := core.New()
	var got []core.StateTransition
	m.RegisterTransitionCallback(func(transition core.StateTransition) error {
		got = append(got, transition)
		return nil
	})
	m.RegisterTransitionCallback(func(transition core.StateTransition) error {
		got = append(got, transition)
		return nil
	})

	applied, err := m.TransitionTo(core.StateReady, core.ReasonBootComplete)
	if !applied || err != nil {
		t.Fatalf("TransitionTo() = (%v, %v), want (true, nil)", applied, err)
	}

	want := core.StateTransition{From: core.StateBootCheck, To: core.StateReady, Reason: core.ReasonBootComplete}
	if len(got) != 2 {
		t.Fatalf("callbacks notified %d times, want 2", len(got))
	}
	for i, transition := range got {
		if transition != want {
			t.Fatalf("callback %d received %+v, want %+v", i, transition, want)
		}
	}
}

// TestTransitionTo_BlockedTransitionDoesNotNotifyCallbacks confirms a
// blocked transition (invalid from the current state) never reaches
// registered callbacks, matching core.py returning False before
// constructing a StateTransition at all.
func TestTransitionTo_BlockedTransitionDoesNotNotifyCallbacks(t *testing.T) {
	t.Parallel()

	m := core.New()
	called := false
	m.RegisterTransitionCallback(func(core.StateTransition) error {
		called = true
		return nil
	})

	// StateBootCheck has no rule for ReasonButtonPressed.
	applied, err := m.TransitionTo(core.StateRacing, core.ReasonButtonPressed)
	if applied {
		t.Fatalf("TransitionTo() applied = true, want false")
	}
	if err != nil {
		t.Fatalf("TransitionTo() unexpected error: %v", err)
	}
	if called {
		t.Fatal("callback was notified for a blocked transition")
	}
}

// TestTransitionTo_CallbackFailureForcesFinishedAndStopsFurtherCallbacks
// is the direct port of core.py's callback-exception branch: a failing
// callback forces the state to FINISHED (a safe state) and TransitionTo
// reports the failure, and no callback registered after the failing one
// runs -- matching the Python for-loop unwinding via `raise` at the first
// exception.
func TestTransitionTo_CallbackFailureForcesFinishedAndStopsFurtherCallbacks(t *testing.T) {
	t.Parallel()

	m := core.New()
	wantErr := errors.New("oled write failed")
	secondCalled := false
	m.RegisterTransitionCallback(func(core.StateTransition) error {
		return wantErr
	})
	m.RegisterTransitionCallback(func(core.StateTransition) error {
		secondCalled = true
		return nil
	})

	applied, err := m.TransitionTo(core.StateReady, core.ReasonBootComplete)

	if !applied {
		t.Fatal("TransitionTo() applied = false, want true (the transition itself was valid)")
	}
	if !errors.Is(err, wantErr) {
		t.Fatalf("TransitionTo() error = %v, want it to wrap %v", err, wantErr)
	}
	if got := m.CurrentState(); got != core.StateFinished {
		t.Fatalf("CurrentState() = %v after a callback failure, want StateFinished (forced safe state)", got)
	}
	if secondCalled {
		t.Fatal("a callback registered after the failing one was still invoked")
	}
}

// TestStateMachine_QueryHelpers exercises CanStartRace/IsRacing/
// IsFinished/IsBootChecking across all four states, confirming exactly
// one is true per state -- these back real competition-rule checks
// (e.g. "ignore a button press unless CanStartRace()"), so a helper
// reporting true in the wrong state is a real safety bug, not cosmetic.
func TestStateMachine_QueryHelpers(t *testing.T) {
	t.Parallel()

	tests := []struct {
		state          core.RobotState
		canStartRace   bool
		isRacing       bool
		isFinished     bool
		isBootChecking bool
	}{
		{core.StateBootCheck, false, false, false, true},
		{core.StateReady, true, false, false, false},
		{core.StateRacing, false, true, false, false},
		{core.StateFinished, false, false, true, false},
	}

	for _, tt := range tests {
		t.Run(tt.state.String(), func(t *testing.T) {
			t.Parallel()

			m := core.New()
			forceState(t, m, tt.state)

			if got := m.CanStartRace(); got != tt.canStartRace {
				t.Errorf("CanStartRace() = %v, want %v", got, tt.canStartRace)
			}
			if got := m.IsRacing(); got != tt.isRacing {
				t.Errorf("IsRacing() = %v, want %v", got, tt.isRacing)
			}
			if got := m.IsFinished(); got != tt.isFinished {
				t.Errorf("IsFinished() = %v, want %v", got, tt.isFinished)
			}
			if got := m.IsBootChecking(); got != tt.isBootChecking {
				t.Errorf("IsBootChecking() = %v, want %v", got, tt.isBootChecking)
			}
		})
	}
}

// TestRobotState_String and TestStateTransitionReason_String pin the
// exact wire/log spellings ported from types.py's StrEnum values -- a
// silent rename here would desync from the /robot_state wire payload and
// from any log-based tooling that greps for these strings.
func TestRobotState_String(t *testing.T) {
	t.Parallel()

	tests := []struct {
		state core.RobotState
		want  string
	}{
		{core.StateBootCheck, "boot_check"},
		{core.StateReady, "ready"},
		{core.StateRacing, "racing"},
		{core.StateFinished, "finished"},
	}

	for _, tt := range tests {
		if got := tt.state.String(); got != tt.want {
			t.Errorf("RobotState(%d).String() = %q, want %q", tt.state, got, tt.want)
		}
	}
}

func TestStateTransitionReason_String(t *testing.T) {
	t.Parallel()

	tests := []struct {
		reason core.StateTransitionReason
		want   string
	}{
		{core.ReasonBootComplete, "boot_complete"},
		{core.ReasonBootFailed, "boot_failed"},
		{core.ReasonButtonPressed, "button_pressed"},
		{core.ReasonLapsCompleted, "laps_completed"},
		{core.ReasonEmergencyStop, "emergency_stop"},
		{core.ReasonSystemReset, "system_reset"},
	}

	for _, tt := range tests {
		if got := tt.reason.String(); got != tt.want {
			t.Errorf("StateTransitionReason(%d).String() = %q, want %q", tt.reason, got, tt.want)
		}
	}
}

func TestScenarioType_String(t *testing.T) {
	t.Parallel()

	tests := []struct {
		scenario core.ScenarioType
		want     string
	}{
		{core.ScenarioOpen, "open"},
		{core.ScenarioObstacles, "obstacles"},
	}

	for _, tt := range tests {
		if got := tt.scenario.String(); got != tt.want {
			t.Errorf("ScenarioType(%d).String() = %q, want %q", tt.scenario, got, tt.want)
		}
	}
}

// forceState drives m from its default StateBootCheck to want using only
// real transitions from transitionTable, so every test in this file
// exercises TransitionTo itself rather than a package-internal test-only
// setter -- there deliberately isn't one, since core.py's StateMachine
// has no equivalent "just set the state" escape hatch either.
func forceState(t *testing.T, m *core.StateMachine, want core.RobotState) {
	t.Helper()

	switch want {
	case core.StateBootCheck:
		return
	case core.StateReady:
		mustTransition(t, m, core.StateReady, core.ReasonBootComplete)
	case core.StateRacing:
		mustTransition(t, m, core.StateReady, core.ReasonBootComplete)
		mustTransition(t, m, core.StateRacing, core.ReasonButtonPressed)
	case core.StateFinished:
		mustTransition(t, m, core.StateFinished, core.ReasonBootFailed)
	}
}

func mustTransition(t *testing.T, m *core.StateMachine, to core.RobotState, reason core.StateTransitionReason) {
	t.Helper()

	applied, err := m.TransitionTo(to, reason)
	if !applied || err != nil {
		t.Fatalf("setup: TransitionTo(%v, %v) = (%v, %v), want (true, nil)", to, reason, applied, err)
	}
}
