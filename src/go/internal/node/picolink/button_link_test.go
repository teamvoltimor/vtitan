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

// While the button is held, ButtonHold is published every poll tick with the
// running duration and both configured thresholds; on release, exactly one
// final empty frame clears the display instead of leaving it on the last
// number -- the Go analog of button_node.py's _publish_hold_progress.
func TestButton_HoldProgressPublishesWhilePressedThenClears(t *testing.T) {
	t.Parallel()

	thresholds := driverbutton.DefaultThresholds()
	cfg := picolink.SessionConfig{
		Board:  sampleBoard,
		Button: &picolink.ButtonParams{Thresholds: thresholds},
	}
	h := startSession(t, cfg)

	h.board.send(t, boardlink.Packet{Type: boardlink.TypeButton, Button: boardlink.Button{Pressed: true}})

	deadline := time.After(waitTimeout)
	var last *uiv1.ButtonHold
	for last == nil || last.GetHeldS() == 0 {
		select {
		case hold := <-h.buttonHold:
			last = hold
		case <-deadline:
			t.Fatal("no ButtonHold with a nonzero HeldS within the timeout")
		}
	}
	if got := len(last.GetThresholds()); got != 2 {
		t.Fatalf("thresholds while pressed = %d, want 2 (long, shutdown)", got)
	}

	h.board.send(t, boardlink.Packet{Type: boardlink.TypeButton, Button: boardlink.Button{Pressed: false}})

	deadline = time.After(waitTimeout)
	for {
		select {
		case hold := <-h.buttonHold:
			if hold.GetHeldS() == 0 && len(hold.GetThresholds()) == 0 {
				return
			}
		case <-deadline:
			t.Fatal("no clearing ButtonHold frame within the timeout")
		}
	}
}
