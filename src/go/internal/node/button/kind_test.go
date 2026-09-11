package button_test

import (
	"testing"

	driverbutton "github.com/teamvoltimor/vtitan/src/go/internal/driver/button"
	nodebutton "github.com/teamvoltimor/vtitan/src/go/internal/node/button"
	uiv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/ui/v1"
)

func TestKindToProto(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name string
		in   driverbutton.Kind
		want uiv1.ButtonEvent_Kind
	}{
		{name: "pressed", in: driverbutton.KindPressed, want: uiv1.ButtonEvent_KIND_PRESSED},
		{name: "long press", in: driverbutton.KindLongPress, want: uiv1.ButtonEvent_KIND_LONG_PRESS},
		{name: "shutdown press", in: driverbutton.KindShutdownPress, want: uiv1.ButtonEvent_KIND_SHUTDOWN_PRESS},
		{name: "short press", in: driverbutton.KindShortPress, want: uiv1.ButtonEvent_KIND_SHORT_PRESS},
		{name: "released", in: driverbutton.KindReleased, want: uiv1.ButtonEvent_KIND_RELEASED},
		{name: "unknown", in: driverbutton.Kind(99), want: uiv1.ButtonEvent_KIND_UNSPECIFIED},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()

			if got := nodebutton.KindToProto(tt.in); got != tt.want {
				t.Errorf("KindToProto(%v) = %v, want %v", tt.in, got, tt.want)
			}
		})
	}
}

func TestEventMessageFor(t *testing.T) {
	t.Parallel()

	got := nodebutton.EventMessageFor(driverbutton.Event{Kind: driverbutton.KindLongPress, HeldSec: 3.5})

	if got.GetKind() != uiv1.ButtonEvent_KIND_LONG_PRESS {
		t.Errorf("Kind = %v, want KIND_LONG_PRESS", got.GetKind())
	}
	if got.GetHeldSec() != 3.5 {
		t.Errorf("HeldSec = %v, want 3.5", got.GetHeldSec())
	}
}

func TestSyntheticMessageFor(t *testing.T) {
	t.Parallel()

	got := nodebutton.SyntheticMessageFor(driverbutton.KindShortPress)

	if got.GetKind() != uiv1.ButtonEvent_KIND_SHORT_PRESS {
		t.Errorf("Kind = %v, want KIND_SHORT_PRESS", got.GetKind())
	}
	if got.GetHeldSec() != 0 {
		t.Errorf("HeldSec = %v, want 0", got.GetHeldSec())
	}
}
