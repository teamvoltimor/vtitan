package boardloop_test

import (
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardloop"
)

// The board reports the raw button state and nothing else: the host runs the
// debounce and hold evaluator, so no timing policy crosses the link. A frame
// goes out on the first Step (so the host learns the initial state) and on
// every change, never on a steady state.
func TestButton_SendsRawEdgesOnChange(t *testing.T) {
	t.Parallel()

	btn := &fakeButton{}
	ev := &events{}
	link := &fakeLink{}
	loop, err := boardloop.New(boardloop.Hardware{
		Link:   link,
		Drive:  &fakeDrive{ev: ev},
		Servo:  &fakeServo{ev: ev},
		Button: btn,
	}, boardloop.Options{})
	if err != nil {
		t.Fatalf("New: %v", err)
	}

	sent := func() []boardlink.Packet {
		var out []boardlink.Packet
		for i := range link.out {
			if link.out[i].Type == boardlink.TypeButton {
				out = append(out, link.out[i])
			}
		}
		return out
	}

	loop.Step(time.Millisecond)
	if got := sent(); len(got) != 1 || got[0].Button.Pressed {
		t.Fatalf("first Step button frames = %+v, want one released", got)
	}

	loop.Step(2 * time.Millisecond)
	if got := sent(); len(got) != 1 {
		t.Fatalf("steady state sent %d button frames, want 1", len(got))
	}

	btn.pressed = true
	loop.Step(3 * time.Millisecond)
	btn.pressed = false
	loop.Step(4 * time.Millisecond)

	got := sent()
	if len(got) != 3 || !got[1].Button.Pressed || got[2].Button.Pressed {
		t.Fatalf("button frames = %+v, want released, pressed, released", got)
	}
}
