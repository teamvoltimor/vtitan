package picolink_test

import (
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/internal/node/picolink"
	uiv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/ui/v1"
	driverbutton "github.com/teamvoltimor/vtitan/src/go/pkg/driver/button"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
)

// The board's raw button edges are evaluated on the host, as the Zero's button
// node evaluates its own GPIO: a press held past the debounce and released
// before the long-press threshold publishes a short press.
func TestButton_RawEdgesBecomeShortPress(t *testing.T) {
	t.Parallel()

	cfg := picolink.SessionConfig{
		Board:  sampleBoard,
		Button: &picolink.ButtonParams{Thresholds: driverbutton.DefaultThresholds()},
	}
	h := startSession(t, cfg)

	h.board.send(t, boardlink.Packet{Type: boardlink.TypeButton, Button: boardlink.Button{Pressed: true}})
	time.Sleep(150 * time.Millisecond)
	h.board.send(t, boardlink.Packet{Type: boardlink.TypeButton, Button: boardlink.Button{Pressed: false}})

	// Skip the press event: the actionable one is the short press on release.
	deadline := time.After(waitTimeout)
	for {
		select {
		case ev := <-h.button:
			if ev.GetKind() == uiv1.ButtonEvent_KIND_SHORT_PRESS {
				return
			}
		case <-deadline:
			t.Fatal("no KIND_SHORT_PRESS ButtonEvent within the timeout")
		}
	}
}
