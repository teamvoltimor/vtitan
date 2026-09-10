package core

// StateTransitionReason is why a StateTransition happened. Mirrors
// StateTransitionReason(StrEnum) in types.py exactly, including which
// (from, reason) pairs are meaningful -- see transitionTable in
// machine.go for the actual from/reason/to mapping.
type StateTransitionReason int

// The six transition reasons, grouped by which edge of the state machine
// they apply to (matching types.py's grouping comments).
const (
	// ReasonBootComplete fires BootCheck -> Ready: all hardware checks
	// passed.
	ReasonBootComplete StateTransitionReason = iota
	// ReasonBootFailed fires BootCheck -> Finished: hardware verification
	// failed.
	ReasonBootFailed
	// ReasonButtonPressed fires Ready -> Racing: the physical button was
	// pressed to start the race.
	ReasonButtonPressed
	// ReasonLapsCompleted fires Racing -> Finished: the target lap count
	// was completed successfully.
	ReasonLapsCompleted
	// ReasonEmergencyStop fires Racing -> Finished: an emergency stop was
	// triggered (a 2-second button hold, in the current hardware).
	ReasonEmergencyStop
	// ReasonSystemReset fires from any state back to BootCheck: a system
	// reset was requested.
	ReasonSystemReset
)

// String returns the same lowercase snake_case spelling as
// StateTransitionReason.value in types.py (e.g. "boot_complete"), since
// this string ends up in structured log fields the same way RobotState's
// does -- callers must not reformat it.
func (r StateTransitionReason) String() string {
	switch r {
	case ReasonBootComplete:
		return "boot_complete"
	case ReasonBootFailed:
		return "boot_failed"
	case ReasonButtonPressed:
		return "button_pressed"
	case ReasonLapsCompleted:
		return "laps_completed"
	case ReasonEmergencyStop:
		return "emergency_stop"
	case ReasonSystemReset:
		return "system_reset"
	default:
		return unknownEnumLabel
	}
}
