package command

// Status is the ack outcome Dispatcher.Dispatch reports for a Command,
// mirroring commands.proto's CommandExecutionStatus enum
// (COMMAND_EXECUTION_STATUS_*) as used by command_channel.py's
// `_dispatch_*` return values.
type Status int

const (
	// StatusUnspecified is the zero value -- never returned by Dispatch,
	// matching COMMAND_EXECUTION_STATUS_UNSPECIFIED never being a real ack.
	StatusUnspecified Status = iota
	// StatusAccepted means the command was handed off but its outcome
	// can't be observed from here -- `_dispatch_button_event`'s only
	// possible result, since publishing a button event is fire-and-forget
	// and state_machine_node reports no reply path.
	StatusAccepted
	// StatusCompleted means the command's effect was applied and
	// confirmed.
	StatusCompleted
	// StatusFailed means the command could not be applied.
	StatusFailed
)

// String returns a short label for logging. Not a wire value -- see
// Kind.String's doc comment for why.
func (s Status) String() string {
	switch s {
	case StatusUnspecified:
		return "unspecified"
	case StatusAccepted:
		return "accepted"
	case StatusCompleted:
		return "completed"
	case StatusFailed:
		return "failed"
	default:
		return "unknown"
	}
}
