// Package core implements the 4-stage competition state machine for the
// WRO robot -- the Go port of platform/robot/src/state_machine/types.py
// and platform/robot/src/state_machine/core.py. Like
// internal/driver/button's Evaluator, this is pure logic with zero
// hardware/transport dependency: StateMachine only knows about states,
// reasons, and the transition rules between them, driven entirely by
// TransitionTo calls from whatever owns the real BOOT_CHECK/READY/
// RACING/FINISHED orchestration loop (state_machine_node.py today; a
// future cmd/state-machine node once NATS wiring lands, per
// docs/internal/plans/go-migration-plan.md step 5).
//
// The four states are:
//
//   - StateBootCheck: hardware verification and initialization.
//   - StateReady: waiting for a button press to start the race.
//   - StateRacing: autonomous racing.
//   - StateFinished: race completed, or an emergency stop was triggered.
//
// Deliberately out of scope for this package (left in the orchestration
// layer, same as it is in Python): the challenge-mode jumper debounce/
// latch/timeout logic (state_machine_node.py's
// _sample_challenge_mode/_on_jumper_state/_latch_challenge_mode) and the
// /robot_state pub/sub glue (src/ros2/race_state.py). Both need wall-clock
// timers and topic subscriptions layered on top of this package's pure
// state, not pure state themselves -- see the package comment in
// internal/statemachine/command for the equivalent scoping note on the
// command-dispatch half of the state machine node.
package core
