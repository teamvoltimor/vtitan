package core

// RobotState is one of the four competition state-machine states. Mirrors
// shared.domain.enums.RobotState (re-exported by types.py as the single
// source of truth) -- an internal-only enum in the Go port, since no
// RobotState.proto exists yet (the wire form today is a plain latched
// std_msgs/String on /robot_state; a future protobuf RobotState enum
// would supersede this type at the wire boundary per go-architect's
// "wire-level enums come from generated code" guidance, but nothing in
// proto/ defines one yet).
type RobotState int

// The four RobotState values, in the order the happy path visits them.
// Every switch over RobotState should be gated by the exhaustive linter
// so a fifth state can't be added without every consumer being forced to
// handle it.
const (
	StateBootCheck RobotState = iota
	StateReady
	StateRacing
	StateFinished
)

// unknownEnumLabel is the String() fallback shared by every enum in this
// package (RobotState, StateTransitionReason, ScenarioType) for a value
// outside its defined range -- named once, at package level, rather than
// repeating the "unknown" literal per switch's default case.
const unknownEnumLabel = "unknown"

// String returns the same lowercase snake_case spelling as
// RobotState.value in types.py (e.g. "boot_check"), since this string is
// what ends up on the wire (/robot_state's String payload) and in
// structured log fields -- callers must not reformat it.
func (s RobotState) String() string {
	switch s {
	case StateBootCheck:
		return "boot_check"
	case StateReady:
		return "ready"
	case StateRacing:
		return "racing"
	case StateFinished:
		return "finished"
	default:
		return unknownEnumLabel
	}
}
