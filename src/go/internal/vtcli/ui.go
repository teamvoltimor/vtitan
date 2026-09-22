// Package vtcli builds the vt command tree over the repository Taskfiles. Its
// design is recorded in other/docs/development/plan-cli-unificada.md: cobra is
// the tree, the Taskfiles remain the single source of truth for how each thing
// runs, and every wrapped leaf shells out to `task`.
package vtcli

import (
	"fmt"
	"os"
	"strings"

	"github.com/charmbracelet/lipgloss"
)

// UI renders the human-facing text vt emits: the bare-command banner and
// errors. Styling is presentation only and is skipped entirely when colour is
// not wanted, so CI logs and piped output stay plain. The wordmark follows the
// TTY, not NO_COLOR: NO_COLOR asks for no colour, not for no art.
type UI struct {
	color bool
	art   bool
}

// MenuEntry is one row of the home menu: a first-level command and its
// one-line description.
type MenuEntry struct {
	Name  string
	Short string
}

// menuColumnWidth is the width reserved for the command name in the home
// menu, in both the styled and the plain renderings.
const menuColumnWidth = 12

// wordmark is the vTitan banner. It is 48 columns wide and 6 rows tall;
// Header falls back to one line below that, plus a margin.
const wordmark = `██╗   ██╗████████╗██╗████████╗ █████╗ ███╗   ██╗
██║   ██║╚══██╔══╝██║╚══██╔══╝██╔══██╗████╗  ██║
██║   ██║   ██║   ██║   ██║   ███████║██╔██╗ ██║
╚██╗ ██╔╝   ██║   ██║   ██║   ██╔══██║██║╚██╗██║
 ╚████╔╝    ██║   ██║   ██║   ██║  ██║██║ ╚████║
  ╚═══╝     ╚═╝   ╚═╝   ╚═╝   ╚═╝  ╚═╝╚═╝  ╚═══╝`

// Wordmark geometry and the room Header needs before drawing it.
const (
	wordmarkWidth   = 48
	minArtWidth     = wordmarkWidth + 4
	minArtHeight    = 22
	tagline         = "Team Voltimor · development CLI over the Taskfiles"
	compactHeadline = "vTitan · Team Voltimor"
)

// wordmarkGradient runs the logo's electric blue from light to deep, one
// colour per row of the wordmark.
var wordmarkGradient = []lipgloss.Color{"#7DD3FC", "#38BDF8", "#0EA5E9", "#0284C7", "#0369A1", "#075985"}

// Palette shared by the banner, the menu and the errors.
var (
	accent = lipgloss.Color("#38BDF8")
	muted  = lipgloss.Color("241")
)

// NewUI returns a UI bound to the capabilities of standard output.
func NewUI() UI {
	return UI{color: useColor(os.Stdout), art: isTerminal(os.Stdout) && os.Getenv("TERM") != "dumb"}
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

// Banner is the header shown by a bare `vt`: the wordmark on a terminal, one
// plain line in a pipe or a CI log.
func (u UI) Banner() string {
	hint := "`task <name>` keeps working unchanged; `vt run <name>` is the escape hatch."

	if !u.art {
		return "vt: vTitan development CLI, Team Voltimor\n" + hint
	}

	return u.wordmarkBlock() + "\n\n" + u.paint(hint, lipgloss.NewStyle().Foreground(muted))
}

// Header is the picker's top: the wordmark when the terminal has room for it
// and still leaves the list usable, otherwise one line.
func (u UI) Header(width, height int) string {
	if width >= minArtWidth && height >= minArtHeight {
		return u.wordmarkBlock()
	}

	return u.paint("⚡ "+compactHeadline, lipgloss.NewStyle().Bold(true).Foreground(accent))
}

// Menu renders the first-level command list. With colour it is a styled
// two-column listing; otherwise a plain aligned one, so pipes and CI stay
// readable.
func (u UI) Menu(entries []MenuEntry) string {
	if !u.color {
		return plainMenu(entries)
	}

	nameStyle := lipgloss.NewStyle().Bold(true).Foreground(accent).Width(nameColumnWidth(entries))
	shortStyle := lipgloss.NewStyle().Foreground(lipgloss.Color("250"))

	rows := make([]string, 0, len(entries))
	for _, entry := range entries {
		rows = append(rows, lipgloss.JoinHorizontal(lipgloss.Top,
			nameStyle.Render(entry.Name), shortStyle.Render(entry.Short)))
	}

	return lipgloss.JoinVertical(lipgloss.Left, rows...)
}

// plainMenu is the NO_COLOR / non-TTY fallback for Menu.
func plainMenu(entries []MenuEntry) string {
	width := nameColumnWidth(entries)

	rows := make([]string, 0, len(entries))
	for _, entry := range entries {
		rows = append(rows, "  "+fmt.Sprintf("%-*s %s", width, entry.Name, entry.Short))
	}

	return strings.Join(rows, "\n")
}

// nameColumnWidth fits the longest name, never narrower than menuColumnWidth.
func nameColumnWidth(entries []MenuEntry) int {
	width := menuColumnWidth
	for _, entry := range entries {
		width = max(width, len(entry.Name)+1)
	}

	return width
}

// Error renders a failure message for standard error.
func (u UI) Error(err error) string {
	if u.color {
		return lipgloss.NewStyle().Foreground(lipgloss.Color("196")).Render("vt: " + err.Error())
	}

	return "vt: " + err.Error()
}

// wordmarkBlock renders the wordmark, row by row down the gradient, with the
// tagline under it.
func (u UI) wordmarkBlock() string {
	rows := strings.Split(wordmark, "\n")
	for i, row := range rows {
		rows[i] = u.paint(row, lipgloss.NewStyle().Foreground(wordmarkGradient[i%len(wordmarkGradient)]))
	}

	line := u.paint("⚡ ", lipgloss.NewStyle().Foreground(accent)) +
		u.paint(tagline, lipgloss.NewStyle().Bold(true))

	return strings.Join(rows, "\n") + "\n" + line
}

// paint applies style only when colour is wanted.
func (u UI) paint(text string, style lipgloss.Style) string {
	if !u.color {
		return text
	}

	return style.Render(text)
}
