package core

// StateTransition is one state-transition event, passed to every
// registered TransitionCallback. Mirrors types.py's frozen StateTransition
// dataclass.
type StateTransition struct {
	From   RobotState
	To     RobotState
	Reason StateTransitionReason
}
