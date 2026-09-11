package statemachine_test

import (
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/node/statemachine"
	statev1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/state/v1"
	"github.com/teamvoltimor/vtitan/src/go/internal/statemachine/core"
)

// TestStateMessageFor covers the mapping value by value. The domain enum
// starts BootCheck at 0 while the wire enum reserves 0 for UNSPECIFIED, so
// every value here would be off by one under an index-based conversion --
// which is exactly the bug this pins.
func TestStateMessageFor(t *testing.T) {
	t.Parallel()

	tests := map[string]struct {
		state core.RobotState
		want  statev1.RobotState_State
	}{
		"boot check": {core.StateBootCheck, statev1.RobotState_STATE_BOOT_CHECK},
		"ready":      {core.StateReady, statev1.RobotState_STATE_READY},
		"racing":     {core.StateRacing, statev1.RobotState_STATE_RACING},
		"finished":   {core.StateFinished, statev1.RobotState_STATE_FINISHED},
	}

	for name, tt := range tests {
		t.Run(name, func(t *testing.T) {
			t.Parallel()

			got := statemachine.StateMessageFor(tt.state)
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

	for state := core.StateBootCheck; state <= core.StateFinished; state++ {
		if got := statemachine.StateMessageFor(state).GetState(); got == statev1.RobotState_STATE_UNSPECIFIED {
			t.Fatalf("state %v (%d) maps to STATE_UNSPECIFIED", state, state)
		}
	}
}
