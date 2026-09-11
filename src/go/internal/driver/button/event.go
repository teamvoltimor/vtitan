package button

// Kind identifies which button event Read returned. Mirrors
// platform/robot/src/hardware/button/event.py's ButtonEvent enum.
type Kind int

// Event is one debounced button transition: a press, a release, or a hold
// crossing one of the two configured thresholds while still held.
type Event struct {
	Kind Kind
	// HeldSec is how long the button had been held, in seconds, at the
	// moment this event fired. Zero for KindPressed.
	HeldSec float64
}

const (
	// KindPressed fires the instant a debounced press begins.
	KindPressed Kind = iota
	// KindLongPress fires once, while still held, when the hold crosses
	// Thresholds.LongPressThreshold.
	KindLongPress
	// KindShutdownPress fires once, while still held, when the hold
	// crosses Thresholds.ShutdownPressThreshold. Never emitted while
	// racing -- see event.py's ButtonEvent.SHUTDOWN_PRESS doc comment;
	// that policy lives in the state machine consumer, not here.
	KindShutdownPress
	// KindShortPress fires on release if neither hold threshold fired
	// during the press.
	KindShortPress
	// KindReleased fires on release if a hold threshold already fired
	// during the press -- the hold event itself was the actionable
	// signal, so release only clears it rather than firing a second
	// action for the same gesture.
	KindReleased
)

// String returns the wire-style spelling used by the Python driver this
// ports (platform/robot/src/hardware/button/event.py), e.g. "short_press".
func (k Kind) String() string {
	switch k {
	case KindPressed:
		return "pressed"
	case KindLongPress:
		return "long_press"
	case KindShutdownPress:
		return "shutdown_press"
	case KindShortPress:
		return "short_press"
	case KindReleased:
		return "released"
	default:
		return "unknown"
	}
}
