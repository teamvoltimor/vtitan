package core

import (
	"fmt"
	"log/slog"
)

// TransitionCallback is notified after StateMachine has already applied a
// transition, mirroring core.py's `Callable[[StateTransition], None]`
// callbacks registered via RegisterTransitionCallback. Unlike the Python
// original, a Go callback reports failure by returning an error rather
// than raising an arbitrary exception -- see TransitionTo's doc comment
// for how that error is handled.
type TransitionCallback func(StateTransition) error

// transitionRule is one (From, Reason) -> To entry in transitionTable.
// Modeled as a slice of structs rather than a nested map (from_state ->
// reason -> to_state, as core.py's _VALID_TRANSITIONS is shaped) per this
// project's "avoid maps" convention -- the table is small (eight rules)
// and read far more often than written, so a linear scan in
// isValidTransition costs nothing observable.
type transitionRule struct {
	From   RobotState
	Reason StateTransitionReason
	To     RobotState
}

// StateMachine is the 4-stage competition state machine for the WRO
// robot. Mirrors core.py's StateMachine class. Not safe for concurrent
// use -- like the Python original, it assumes a single caller drives
// TransitionTo (state_machine_node.py's single-threaded ROS2 executor
// today; a single goroutine owning the state-machine loop in the Go
// port), matching this project's "sync.RWMutex only for cached
// latest-value reads" convention: nothing here is a cached read, so no
// mutex is added speculatively.
type StateMachine struct {
	logger       *slog.Logger
	callbacks    []TransitionCallback
	currentState RobotState
}

// Option configures a StateMachine at construction time. Functional
// options rather than a config struct: New only ever takes an optional
// logger override today, but this keeps room for more without a breaking
// signature change, matching go-architect §3.
type Option func(*StateMachine)

// transitionTable is the complete set of valid transitions, a direct,
// exhaustive port of core.py's _VALID_TRANSITIONS. Every (From, Reason)
// pair not listed here is invalid -- see the states below for the
// specific rules ported:
//
//   - StateBootCheck: ReasonBootComplete -> StateReady,
//     ReasonBootFailed -> StateFinished.
//   - StateReady: ReasonButtonPressed -> StateRacing,
//     ReasonSystemReset -> StateBootCheck.
//   - StateRacing: ReasonLapsCompleted -> StateFinished,
//     ReasonEmergencyStop -> StateFinished,
//     ReasonSystemReset -> StateBootCheck.
//   - StateFinished: ReasonSystemReset -> StateBootCheck.
var transitionTable = []transitionRule{
	{From: StateBootCheck, Reason: ReasonBootComplete, To: StateReady},
	{From: StateBootCheck, Reason: ReasonBootFailed, To: StateFinished},
	{From: StateReady, Reason: ReasonButtonPressed, To: StateRacing},
	{From: StateReady, Reason: ReasonSystemReset, To: StateBootCheck},
	{From: StateRacing, Reason: ReasonLapsCompleted, To: StateFinished},
	{From: StateRacing, Reason: ReasonEmergencyStop, To: StateFinished},
	{From: StateRacing, Reason: ReasonSystemReset, To: StateBootCheck},
	{From: StateFinished, Reason: ReasonSystemReset, To: StateBootCheck},
}

// isValidTransition reports whether (from, reason) maps to to in
// transitionTable, matching core.py's `_is_valid_transition`
// (`allowed.get(reason) == to_state`).
func isValidTransition(from, to RobotState, reason StateTransitionReason) bool {
	for _, rule := range transitionTable {
		if rule.From == from && rule.Reason == reason {
			return rule.To == to
		}
	}
	return false
}

// WithLogger overrides the *slog.Logger used for transition
// warn/info/error logging (see TransitionTo). Defaults to slog.Default()
// if not supplied.
func WithLogger(logger *slog.Logger) Option {
	return func(m *StateMachine) {
		m.logger = logger
	}
}

// New builds a StateMachine starting in StateBootCheck, matching core.py's
// `__init__` (`self._current_state: RobotState = RobotState.BOOT_CHECK`).
func New(opts ...Option) *StateMachine {
	m := &StateMachine{
		currentState: StateBootCheck,
		logger:       slog.Default(),
	}
	for _, opt := range opts {
		opt(m)
	}
	return m
}

// CurrentState returns the state machine's current state, matching
// core.py's `current_state` property.
func (m *StateMachine) CurrentState() RobotState {
	return m.currentState
}

// RegisterTransitionCallback appends callback to the list notified after
// every successful TransitionTo call, matching core.py's
// `register_transition_callback`. Callbacks run in registration order.
func (m *StateMachine) RegisterTransitionCallback(callback TransitionCallback) {
	m.callbacks = append(m.callbacks, callback)
}

// TransitionTo attempts to move the state machine to newState for reason.
// It returns applied=true if the transition was in transitionTable and
// was therefore applied; applied=false (with a warning logged, err nil)
// if the transition was not in transitionTable, matching core.py's
// `transition_to` returning False for a blocked transition without
// raising.
//
// If the transition is applied but a registered callback returns an
// error, that is the Go analog of core.py's callback raising an
// exception: the Python original logs the failure at CRITICAL, forces
// _current_state to RobotState.FINISHED, and re-raises so the caller
// sees the failure. TransitionTo does the same -- forces currentState to
// StateFinished and returns applied=true with a non-nil, wrapped error --
// rather than panicking, since an arbitrary callback failure is exactly
// the kind of expected-to-happen-eventually condition go-architect §8
// says shouldn't be modeled as a panic. Callbacks after the failing one
// are not invoked, matching the Python for-loop's exception unwinding
// stopping at the first failure.
func (m *StateMachine) TransitionTo(newState RobotState, reason StateTransitionReason) (applied bool, err error) {
	if !isValidTransition(m.currentState, newState, reason) {
		m.logger.Warn("invalid state transition blocked",
			slog.String("from", m.currentState.String()),
			slog.String("to", newState.String()),
			slog.String("reason", reason.String()),
		)
		return false, nil
	}

	transition := StateTransition{From: m.currentState, To: newState, Reason: reason}

	m.logger.Info("state transition",
		slog.String("from", transition.From.String()),
		slog.String("to", transition.To.String()),
		slog.String("reason", transition.Reason.String()),
	)
	m.currentState = newState

	for _, callback := range m.callbacks {
		if cbErr := callback(transition); cbErr != nil {
			m.logger.Error("state transition callback FAILED -- triggering emergency stop",
				slog.String("error", cbErr.Error()),
			)
			m.currentState = StateFinished
			return true, fmt.Errorf("state transition callback failed: %w", cbErr)
		}
	}

	return true, nil
}

// CanStartRace reports whether the robot is in StateReady, matching
// core.py's `can_start_race`.
func (m *StateMachine) CanStartRace() bool {
	return m.currentState == StateReady
}

// IsRacing reports whether the robot is in StateRacing, matching core.py's
// `is_racing`.
func (m *StateMachine) IsRacing() bool {
	return m.currentState == StateRacing
}

// IsFinished reports whether the robot is in StateFinished, matching
// core.py's `is_finished`.
func (m *StateMachine) IsFinished() bool {
	return m.currentState == StateFinished
}

// IsBootChecking reports whether the robot is in StateBootCheck, matching
// core.py's `is_boot_checking`.
func (m *StateMachine) IsBootChecking() bool {
	return m.currentState == StateBootCheck
}
