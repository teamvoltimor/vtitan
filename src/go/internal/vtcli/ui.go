// Package vtcli builds the vt command tree over the repository Taskfiles. Its
// design is recorded in ADR 0096: cobra is the tree, the Taskfiles remain the
// single source of truth for how each thing runs, and every wrapped leaf shells
// out to `task`. Colors and sizes live in theme.go.
package vtcli

import (
	"fmt"
	"os"
	"strings"

	"github.com/charmbracelet/lipgloss"
	"github.com/charmbracelet/x/term"
)

// UI renders the human-facing text vt emits: the banner, the menu, the form
// and errors. Styling is presentation only and is skipped entirely when colour
// is not wanted, so CI logs and piped output stay plain. The wordmark follows
// the TTY, not NO_COLOR: NO_COLOR asks for no colour, not for no art.
type UI struct {
	color bool
	art   bool
	// width is the terminal width at start-up, 0 when it is not a terminal
	// or will not say. It only centers the static banner; the picker gets
	// live sizes from bubbletea.
	width int
}

// MenuEntry is one row of the home menu: a first-level command and its
// one-line description.
type MenuEntry struct {
	Name  string
	Short string
}

// MenuSection is one titled group of the home menu.
type MenuSection struct {
	Title   string
	Entries []MenuEntry
}

// NewUI returns a UI bound to the capabilities of standard output.
func NewUI() UI {
	ui := UI{color: useColor(os.Stdout), art: isTerminal(os.Stdout) && os.Getenv("TERM") != "dumb"}
	if width, _, err := term.GetSize(os.Stdout.Fd()); err == nil {
		ui.width = width
	}

	return ui
}

// useColor reports whether ANSI styling should be emitted to f. NO_COLOR and a
// dumb terminal both win over the TTY check.
func useColor(f *os.File) bool {
	if _, disabled := os.LookupEnv("NO_COLOR"); disabled {
		return false
	}

	if os.Getenv("TERM") == "dumb" {
		return false
	}

	return isTerminal(f)
}

// isTerminal reports whether f is a character device (a console or TTY).
func isTerminal(f *os.File) bool {
	info, err := f.Stat()
	if err != nil {
		return false
	}

	return info.Mode()&os.ModeCharDevice != 0
}

// Banner is the header shown by a bare `vt`: the centered wordmark on a
// terminal, one plain line in a pipe or a CI log.
func (u UI) Banner() string {
	if !u.art {
		return plainHeadline + "\n" + escapeHint
	}

	width := u.width
	if width <= 0 {
		width = fallbackTermWidth
	}

	return u.Header(width, minArtHeight) + "\n" + u.center(u.Muted(escapeHint), width)
}

// Header is the picker's top: the wordmark centered in the terminal with a
// margin row above and below when there is room for it and the list, otherwise
// one line. Its height is what the picker subtracts from the list.
func (u UI) Header(width, height int) string {
	top := strings.Repeat("\n", headerMarginTop)
	bottom := strings.Repeat("\n", headerMarginBottom)

	if width < minArtWidth || height < minArtHeight {
		line := u.Accent(brandGlyph+" "+compactHeadline, true)

		return top + u.center(line, width) + bottom
	}

	return top + u.center(u.wordmarkBlock(), width) + bottom
}

// Accent paints text in the accent colour, bold when asked.
func (u UI) Accent(text string, bold bool) string {
	return u.paint(text, lipgloss.NewStyle().Bold(bold).Foreground(colorAccent))
}

// Muted paints secondary text: hints, descriptions, key help.
func (u UI) Muted(text string) string {
	return u.paint(text, lipgloss.NewStyle().Foreground(colorMuted))
}

// Warning paints a caution the user should read before confirming.
func (u UI) Warning(text string) string {
	return u.paint(text, lipgloss.NewStyle().Bold(true).Foreground(colorWarning))
}

// Menu renders the first-level command list, names in the accent colour. The
// plain rendering keeps the same columns, so pipes and CI stay readable.
func (u UI) Menu(entries []MenuEntry) string {
	width := nameColumnWidth(entries)
	indent := strings.Repeat(" ", menuIndent)

	rows := make([]string, 0, len(entries))
	for _, entry := range entries {
		name := fmt.Sprintf("%-*s", width, entry.Name)
		rows = append(rows, indent+u.Accent(name, true)+" "+entry.Short)
	}

	return strings.Join(rows, "\n")
}

// MenuSections renders the home menu group by group, titles muted, with one
// name column shared by every section so they line up.
func (u UI) MenuSections(sections []MenuSection) string {
	var all []MenuEntry
	for _, section := range sections {
		all = append(all, section.Entries...)
	}

	width := nameColumnWidth(all)
	blocks := make([]string, 0, len(sections))

	for _, section := range sections {
		rows := []string{u.Muted(section.Title)}
		for _, entry := range section.Entries {
			rows = append(rows, strings.Repeat(" ", menuIndent)+u.Accent(fmt.Sprintf("%-*s", width, entry.Name), true)+
				" "+entry.Short)
		}

		blocks = append(blocks, strings.Join(rows, "\n"))
	}

	return strings.Join(blocks, "\n\n")
}

// Danger paints a failure the user has to act on.
func (u UI) Danger(text string) string {
	return u.paint(text, lipgloss.NewStyle().Foreground(colorDanger))
}

// Error renders a failure message for standard error.
func (u UI) Error(err error) string {
	return u.Danger("vt: " + err.Error())
}

// wordmarkBlock renders the wordmark down the gradient with the tagline under
// it, as one block whose rows share a left edge.
func (u UI) wordmarkBlock() string {
	rows := strings.Split(wordmark, "\n")
	for i, row := range rows {
		padded := fmt.Sprintf("%-*s", wordmarkWidth, row)
		rows[i] = u.paint(padded, lipgloss.NewStyle().Foreground(wordmarkGradient[i%len(wordmarkGradient)]))
	}

	line := u.Accent(brandGlyph+" ", false) + u.paint(tagline, lipgloss.NewStyle().Bold(true))

	return lipgloss.JoinVertical(lipgloss.Center, strings.Join(rows, "\n"), line)
}

// center places a block in the middle of width columns, as a unit, so a
// multi-row block keeps its own alignment.
func (u UI) center(block string, width int) string {
	return lipgloss.PlaceHorizontal(width, lipgloss.Center, block)
}

// paint applies style only when colour is wanted.
func (u UI) paint(text string, style lipgloss.Style) string {
	if !u.color {
		return text
	}

	return style.Render(text)
}

// nameColumnWidth fits the longest name, never narrower than menuColumnWidth.
func nameColumnWidth(entries []MenuEntry) int {
	width := menuColumnWidth
	for _, entry := range entries {
		width = max(width, len(entry.Name)+1)
	}

	return width
}
