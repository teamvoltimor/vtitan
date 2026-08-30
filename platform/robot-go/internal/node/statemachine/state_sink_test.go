package statemachine_test

import (
	"testing"

	nodestatemachine "github.com/teamvoltimor/vtitan/platform/robot-go/internal/node/statemachine"
	statev1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/state/v1"
	smcore "github.com/teamvoltimor/vtitan/platform/robot-go/internal/statemachine/core"
)

// TestStateMessageFor covers the mapping value by value. The domain enum
// starts BootCheck at 0 while the wire enum reserves 0 for UNSPECIFIED, so
// every value here would be off by one under an index-based conversion --
// which is exactly the bug this pins.
func TestStateMessageFor(t *testing.T) {
	t.Parallel()

	tests := map[string]struct {
		state smcore.RobotState
		want  statev1.RobotState_State
	}{
		"boot check": {smcore.StateBootCheck, statev1.RobotState_STATE_BOOT_CHECK},
		"ready":      {smcore.StateReady, statev1.RobotState_STATE_READY},
		"racing":     {smcore.StateRacing, statev1.RobotState_STATE_RACING},
		"finished":   {smcore.StateFinished, statev1.RobotState_STATE_FINISHED},
	}

	for name, tt := range tests {
		t.Run(name, func(t *testing.T) {
			t.Parallel()

			got := nodestatemachine.StateMessageFor(tt.state)
			if got.GetState() != tt.want {
				t.Fatalf("State = %v, want %v", got.GetState(), tt.want)
			}
			if got.GetStamp() == nil {
				t.Fatal("Stamp = nil, which fails the schema's required constraint")
			}
		})
	}
}

// TestStateMessageFor_IsExhaustive guards a state being added to the domain
// machine and never mapped, which would publish STATE_UNSPECIFIED forever.
func TestStateMessageFor_IsExhaustive(t *testing.T) {
	t.Parallel()

	for state := smcore.StateBootCheck; state <= smcore.StateFinished; state++ {
		if got := nodestatemachine.StateMessageFor(state).GetState(); got == statev1.RobotState_STATE_UNSPECIFIED {
			t.Fatalf("state %v (%d) maps to STATE_UNSPECIFIED", state, state)
		}
	}
}
