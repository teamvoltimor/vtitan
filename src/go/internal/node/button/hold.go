package button

import (
	"time"

	"google.golang.org/protobuf/types/known/timestamppb"

	uiv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/ui/v1"
)

// HoldThreshold is one hold duration at which something happens, mirroring
// wire_models.py's ButtonHoldThreshold.
//
// Kind is free-form because the producer sends a human-readable label the
// OLED renders verbatim -- it is display copy, not a value anything branches
// on, so closing it into an enum would buy nothing and break the moment a
// new hold action is added.
type HoldThreshold struct {
	At   time.Duration
	Kind string
}

// HoldMessageFor converts an in-progress button hold into its wire form for
// the vtitan.ui.v1.button_hold subject.
//
// Distinct from EventMessageFor, which reports a COMPLETED press: this is the
// continuous signal published while the button is still down, so the display
// can show progress toward the next threshold.
//
// Thresholds with an empty Kind are dropped rather than published: the schema
// requires a non-empty kind, so passing one through would produce a message
// that fails its own validation at the publisher.
func HoldMessageFor(held time.Duration, thresholds []HoldThreshold) *uiv1.ButtonHold {
	message := &uiv1.ButtonHold{
		Stamp: timestamppb.Now(),
		HeldS: held.Seconds(),
	}
	for _, threshold := range thresholds {
		if threshold.Kind == "" {
			continue
		}
		message.Thresholds = append(message.Thresholds, &uiv1.ButtonHold_Threshold{
			AtS:  threshold.At.Seconds(),
			Kind: threshold.Kind,
		})
	}
	return message
}
