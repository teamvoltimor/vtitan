package button

import (
	"google.golang.org/protobuf/types/known/timestamppb"

	driverbutton "github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/button"
	uiv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/ui/v1"
)

// KindToProto maps internal/driver/button.Kind onto its wire counterpart --
// an explicit switch rather than a numeric cast, since ButtonEvent_Kind's
// zero value is KIND_UNSPECIFIED (not KIND_PRESSED), so the two enums'
// ordinals don't line up.
func KindToProto(kind driverbutton.Kind) uiv1.ButtonEvent_Kind {
	switch kind {
	case driverbutton.KindPressed:
		return uiv1.ButtonEvent_KIND_PRESSED
	case driverbutton.KindLongPress:
		return uiv1.ButtonEvent_KIND_LONG_PRESS
	case driverbutton.KindShutdownPress:
		return uiv1.ButtonEvent_KIND_SHUTDOWN_PRESS
	case driverbutton.KindShortPress:
		return uiv1.ButtonEvent_KIND_SHORT_PRESS
	case driverbutton.KindReleased:
		return uiv1.ButtonEvent_KIND_RELEASED
	default:
		return uiv1.ButtonEvent_KIND_UNSPECIFIED
	}
}

// EventMessageFor converts a decoded driver event into the ButtonEvent
// message to publish.
func EventMessageFor(event driverbutton.Event) *uiv1.ButtonEvent {
	return &uiv1.ButtonEvent{
		Stamp:   timestamppb.Now(),
		Kind:    KindToProto(event.Kind),
		HeldSec: event.HeldSec,
	}
}

// SyntheticMessageFor builds the ButtonEvent message a synthetic
// (non-hardware) press publishes -- HeldSec is always 0, matching
// command_channel.py's `_dispatch_button_event`, which has no real hold
// duration to report.
func SyntheticMessageFor(kind driverbutton.Kind) *uiv1.ButtonEvent {
	return &uiv1.ButtonEvent{
		Stamp:   timestamppb.Now(),
		Kind:    KindToProto(kind),
		HeldSec: 0,
	}
}
