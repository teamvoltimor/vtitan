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

	"github.com/ralvarezdev/termkit"
)

// UI renders the human-facing text vt emits: the banner, the menu, the form
// and errors. Styling is presentation only and is skipped entirely when colour
// is not wanted, so CI logs and piped output stay plain. The wordmark follows
// the TTY, not NO_COLOR: NO_COLOR asks for no colour, not for no art.
type UI struct {
	// cap is termkit's own resolved colour/art/width capability, shared with
	// every other termkit consumer.
	cap termkit.Capability
	// dark is the terminal's detected background, for the palette and for
	// answering a task that asks the terminal for it.
	dark bool
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

// vtitanBannerSpec is vt's BannerSpec: the wordmark art, its gradient, copy,
// and layout thresholds, built from theme.go's constants. termkit owns the
// mechanism (tier selection, centering, margins); this spec owns everything
// the banner says and looks like.
var vtitanBannerSpec = termkit.BannerSpec{
	Wordmark:         wordmark,
	WordmarkWidth:    wordmarkWidth,
	WordmarkGradient: wordmarkGradient,
	BrandGlyph:       brandGlyph,
	Tagline:          tagline,
	CompactHeadline:  compactHeadline,
	PlainHeadline:    plainHeadline,
	MinArtWidth:      minArtWidth,
	MinArtHeight:     minArtHeight,
	MarginTop:        headerMarginTop,
	MarginBottom:     headerMarginBottom,
	FallbackWidth:    fallbackTermWidth,
}

// NewUI returns a UI bound to the capabilities of standard output.
func NewUI() UI {
	setVtitanPalette()

	ui := UI{cap: termkit.NewCapability(useColor(os.Stdout)), dark: true}
	if ui.cap.Color {
		// The same detection AdaptiveColor uses; only worth asking a terminal.
		ui.dark = lipgloss.HasDarkBackground()
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
// terminal, one plain line in a pipe or a CI log. It composes termkit's
// Header directly rather than calling Capability.Banner(spec): vt's plain
// fallback is two lines (plainHeadline + escapeHint, not just PlainHeadline),
// and the line under the wordmark is escapeHint, not spec.Tagline (which
// wordmarkBlock already renders bold next to the brand glyph).
func (u UI) Banner() string {
	if !u.cap.Art {
		return plainHeadline + "\n" + escapeHint
	}

	width := u.cap.Width
	if width <= 0 {
		width = fallbackTermWidth
	}

	return u.Header(width, minArtHeight) + "\n" + u.cap.Center(u.Muted(escapeHint), width)
}

// Header is the picker's top: the wordmark centered in the terminal with a
// margin row above and below when there is room for it and the list, otherwise
// one line. Its height is what the picker subtracts from the list. It
// delegates entirely to termkit's tier-selection mechanism.
func (u UI) Header(width, height int) string {
	return u.cap.Header(vtitanBannerSpec, width, height)
}

// Accent paints text in the accent colour, bold when asked.
func (u UI) Accent(text string, bold bool) string {
	return u.cap.Accent(text, bold)
}

// Muted paints secondary text: hints, descriptions, key help.
func (u UI) Muted(text string) string {
	return u.cap.Muted(text)
}

// Warning paints a caution the user should read before confirming.
func (u UI) Warning(text string) string {
	return u.cap.Paint(text, lipgloss.NewStyle().Bold(true).Foreground(termkit.ColorWarning))
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
	return u.cap.Paint(text, lipgloss.NewStyle().Foreground(termkit.ColorDanger))
}

// Error renders a failure message for standard error.
func (u UI) Error(err error) string {
	return u.Danger("vt: " + err.Error())
}

// nameColumnWidth fits the longest name, never narrower than menuColumnWidth.
func nameColumnWidth(entries []MenuEntry) int {
	width := menuColumnWidth
	for _, entry := range entries {
		width = max(width, len(entry.Name)+1)
	}

	return width
}
