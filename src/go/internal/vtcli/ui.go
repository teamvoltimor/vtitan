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
// not wanted, so CI logs and piped output stay plain.
type UI struct {
	color bool
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

// NewUI returns a UI bound to the capabilities of standard output.
func NewUI() UI {
	return UI{color: useColor(os.Stdout)}
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

// Banner is the header shown by a bare `vt`.
func (u UI) Banner() string {
	line := "vt — CLI de desarrollo sobre los Taskfiles"
	hint := "`task <nombre>` sigue funcionando igual; `vt run <nombre>` es la puerta de escape."

	if !u.color {
		return line + "\n" + hint
	}

	title := lipgloss.NewStyle().Bold(true).Foreground(lipgloss.Color("205"))
	muted := lipgloss.NewStyle().Foreground(lipgloss.Color("241"))

	return title.Render(line) + "\n" + muted.Render(hint)
}

// Menu renders the first-level command list. With colour it is a styled
// two-column listing; otherwise a plain aligned one, so pipes and CI stay
// readable.
func (u UI) Menu(entries []MenuEntry) string {
	if !u.color {
		return plainMenu(entries)
	}

	nameStyle := lipgloss.NewStyle().Bold(true).Foreground(lipgloss.Color("205")).Width(menuColumnWidth)
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
	rows := make([]string, 0, len(entries))
	for _, entry := range entries {
		rows = append(rows, "  "+fmt.Sprintf("%-*s %s", menuColumnWidth, entry.Name, entry.Short))
	}

	return strings.Join(rows, "\n")
}

// Error renders a failure message for standard error.
func (u UI) Error(err error) string {
	if u.color {
		return lipgloss.NewStyle().Foreground(lipgloss.Color("196")).Render("vt: " + err.Error())
	}

	return "vt: " + err.Error()
}
