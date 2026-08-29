package statemachine

import (
	"fmt"

	driverbutton "github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/button"
	nodebutton "github.com/teamvoltimor/vtitan/platform/robot-go/internal/node/button"
	uiv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/ui/v1"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/transport/nats"
)

// NATSButtonSink implements command.ButtonSink by publishing a synthetic
// ButtonEvent on the same vtitan.ui.v1.button_event subject the physical
// button driver publishes real presses to (see cmd/pi-zero's buttonLoop) --
// the Go analog of command_channel.py's `_button_pub` rclpy Publisher.
type NATSButtonSink struct {
	pub *nats.Publisher[*uiv1.ButtonEvent]
}

// NewNATSButtonSink builds a NATSButtonSink over an already-connected pub.
func NewNATSButtonSink(pub *nats.Publisher[*uiv1.ButtonEvent]) *NATSButtonSink {
	return &NATSButtonSink{pub: pub}
}

// PublishButtonEvent implements command.ButtonSink.
func (s *NATSButtonSink) PublishButtonEvent(kind driverbutton.Kind) error {
	if err := s.pub.Publish(nodebutton.SyntheticMessageFor(kind)); err != nil {
		return fmt.Errorf("node/statemachine: publishing synthetic ButtonEvent: %w", err)
	}
	return nil
}
