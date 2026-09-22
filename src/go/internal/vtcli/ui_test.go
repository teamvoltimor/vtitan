package vtcli

import (
	"strings"
	"testing"

	tea "github.com/charmbracelet/bubbletea"
)

// TestBannerFollowsTheTerminal checks the three renderings: a pipe gets one
// plain line, NO_COLOR on a terminal keeps the art without escapes, and the
// picker header drops to one line when the art would not fit.
func TestBannerFollowsTheTerminal(t *testing.T) {
	t.Parallel()

	piped := UI{}.Banner()
	if strings.Contains(piped, "██") || strings.Contains(piped, "\x1b") {
		t.Errorf("piped banner has art or escapes: %q", piped)
	}

	noColor := UI{art: true}.Banner()
	if !strings.Contains(noColor, "██") || strings.Contains(noColor, "\x1b") {
		t.Errorf("NO_COLOR banner should keep the art and drop escapes: %q", noColor)
	}

	for row := range strings.SplitSeq(wordmark, "\n") {
		if width := len([]rune(row)); width > wordmarkWidth {
			t.Errorf("wordmark row is %d columns, wordmarkWidth says %d", width, wordmarkWidth)
		}
	}

	if header := (UI{art: true}).Header(minArtWidth, minArtHeight); !strings.Contains(header, "██") {
		t.Error("header dropped the art in a terminal big enough for it")
	}

	for _, size := range [][2]int{{minArtWidth - 1, minArtHeight}, {minArtWidth, minArtHeight - 1}} {
		if header := (UI{art: true}).Header(size[0], size[1]); strings.Contains(header, "\n") {
			t.Errorf("header at %dx%d should be one line, got %q", size[0], size[1], header)
		}
	}
}

// TestPickerKeepsTerminalSizeAcrossLevels checks that the list fills the
// terminal under the header, and still does after descending: every level
// builds a new list, which used to reset to the 80x24 default.
func TestPickerKeepsTerminalSizeAcrossLevels(t *testing.T) {
	t.Parallel()

	ui := UI{art: true}
	root := []pickItem{{title: "go", segment: "go"}}
	child := func([]string) []pickItem { return []pickItem{backRow(), {title: "test"}} }

	model := newPickerModel(root, child, nil, ui.Header)
	model.Update(tea.WindowSizeMsg{Width: 100, Height: 50})

	headerRows := strings.Count(ui.Header(100, 50), "\n") + 1
	want := 50 - headerRows - 1

	if got := model.list.Height(); got != want {
		t.Errorf("root list height = %d, want %d", got, want)
	}

	model.descend("go")

	if got := model.list.Height(); got != want {
		t.Errorf("list height after descending = %d, want %d", got, want)
	}
}
