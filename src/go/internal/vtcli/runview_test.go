package vtcli

import (
	"fmt"
	"testing"

	tea "github.com/charmbracelet/bubbletea"
)

// TestRunPaneScrollsWithTheWheel checks that a mouse-wheel event scrolls the
// output. The run pane is the one screen with scrollback worth wheeling, and
// the viewport only reacts when the wheel is enabled on it.
func TestRunPaneScrollsWithTheWheel(t *testing.T) {
	t.Parallel()

	pane := newRunModel(NewUI(), "vt test", nil, "", 80, 10)
	for i := range 100 {
		pane.buf.feed(fmt.Appendf(nil, "line %d\r\n", i))
	}

	pane.refresh()
	pane.view.GotoBottom()
	bottom := pane.view.YOffset

	_, _ = pane.Update(tea.MouseMsg{Action: tea.MouseActionPress, Button: tea.MouseButtonWheelUp})

	if pane.view.YOffset >= bottom {
		t.Errorf("wheel up did not scroll: offset %d, was at the bottom (%d)", pane.view.YOffset, bottom)
	}
}
