package statemachine

import (
	"fmt"

	"google.golang.org/protobuf/types/known/timestamppb"

	statev1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/state/v1"
	smcore "github.com/teamvoltimor/vtitan/platform/robot-go/internal/statemachine/core"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/transport/nats"
)

// NATSStateSink publishes the machine's current state on the
// vtitan.state.v1.robot_state subject, replacing state_machine_node's
// /robot_state std_msgs/String publisher.
//
// Published on every transition rather than on a timer: the OLED and the
// navigator both branch on state, and the FINISHED -> BOOT_CHECK -> READY ->
// RACING cycle can be driven entirely by the physical button with no process
// restart, so a missed transition leaves a consumer acting on a state the
// robot has already left.
type NATSStateSink struct {
	pub *nats.Publisher[*statev1.RobotState]
}

// NewNATSStateSink builds a NATSStateSink over an already-connected pub.
func NewNATSStateSink(pub *nats.Publisher[*statev1.RobotState]) *NATSStateSink {
	return &NATSStateSink{pub: pub}
}

// PublishState publishes state as the machine's current state.
func (s *NATSStateSink) PublishState(state smcore.RobotState) error {
	if err := s.pub.Publish(StateMessageFor(state)); err != nil {
		return fmt.Errorf("node/statemachine: publishing RobotState: %w", err)
	}
	return nil
}

// StateMessageFor converts a domain state into its wire form. Exported so
// tests and any future combined publisher share one mapping rather than each
// re-deriving it.
func StateMessageFor(state smcore.RobotState) *statev1.RobotState {
	return &statev1.RobotState{
		Stamp: timestamppb.Now(),
		State: stateFor(state),
	}
}

// stateFor maps the domain state onto the wire enum.
//
// An exhaustive switch rather than an index: the two enums are independent
// numbering spaces, and the wire one reserves 0 for UNSPECIFIED while the
// domain one starts BootCheck at 0, so an index-based mapping would be off
// by one on every value.
func stateFor(state smcore.RobotState) statev1.RobotState_State {
	switch state {
	case smcore.StateBootCheck:
		return statev1.RobotState_STATE_BOOT_CHECK
	case smcore.StateReady:
		return statev1.RobotState_STATE_READY
	case smcore.StateRacing:
		return statev1.RobotState_STATE_RACING
	case smcore.StateFinished:
		return statev1.RobotState_STATE_FINISHED
	default:
		return statev1.RobotState_STATE_UNSPECIFIED
	}
}
