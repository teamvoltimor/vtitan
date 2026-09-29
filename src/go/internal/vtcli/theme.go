package vtcli

import (
	"github.com/charmbracelet/lipgloss"

	"github.com/ralvarezdev/termkit"
)

// This file is every presentation knob vt has: the palette, the wordmark, and
// the layout sizes. Nothing outside it names a colour or a magic width.

// docsTokensFile is the colour system vt follows: the docs site's accent
// tokens, with a light (:root) and a dark (html.dark) value each. lipgloss picks
// between them from the terminal background. TestPaletteMatchesDocsTokens
// fails if a value here drifts from that file.
const docsTokensFile = "other/apps/hugo-docs/assets/custom/_root.scss"

// Brand text.
const (
	// wordmark is the vTitan banner, wordmarkWidth columns by six rows. Rows
	// are padded to the full width at render time so centering keeps them
	// aligned.
	wordmark = `██╗   ██╗████████╗██╗████████╗ █████╗ ███╗   ██╗
██║   ██║╚══██╔══╝██║╚══██╔══╝██╔══██╗████╗  ██║
██║   ██║   ██║   ██║   ██║   ███████║██╔██╗ ██║
╚██╗ ██╔╝   ██║   ██║   ██║   ██╔══██║██║╚██╗██║
 ╚████╔╝    ██║   ██║   ██║   ██║  ██║██║ ╚████║
  ╚═══╝     ╚═╝   ╚═╝   ╚═╝   ╚═╝  ╚═╝╚═╝  ╚═══╝`
	wordmarkWidth = 48

	brandGlyph      = "⚡"
	tagline         = "Team Voltimor · development CLI over the Taskfiles"
	compactHeadline = "vTitan · Team Voltimor"
	plainHeadline   = "vt: vTitan development CLI, Team Voltimor"
	escapeHint      = "`task <name>` keeps working unchanged; `vt task <name>` runs any task from here."

	// catchAllName is the escape hatch's command: any Task task by name.
	catchAllName = "task"
	// secretMask replaces a Secret flag's value wherever vt prints it.
	secretMask = "***"
	// equivalentPrefix starts the line vt prints after a picker run: the
	// command that repeats it without the picker.
	equivalentPrefix = "$ "
	// dryRunFlag names the global flag that prints instead of running.
	dryRunFlag = "dry-run"
	// recentMark prefixes the picker's recently used rows.
	recentMark = "↺ "
)

// Recent picks, kept in the user cache directory.
const (
	recentLimit         = 5
	recentDir           = "vt"
	recentFile          = "recent"
	recentCommandPrefix = "cmd:"
	recentTaskPrefix    = "task:"
	recentDirPerm       = 0o755
	recentFilePerm      = 0o644
)

// Layout. The header is centered in the terminal with a blank row above and
// below; under minArtWidth x minArtHeight it drops to one line so the list
// keeps room.
const (
	headerMarginTop    = 1
	headerMarginBottom = 1
	headerSideMargin   = 2
	minArtWidth        = wordmarkWidth + 2*headerSideMargin + 2
	minArtHeight       = 22

	// fallbackTermWidth centers the static banner when the terminal will not
	// report its size.
	fallbackTermWidth = 80

	// menuIndent and menuColumnWidth lay out the home menu: the name column
	// never narrower than menuColumnWidth.
	menuIndent      = 2
	menuColumnWidth = 12

	// taskShortWidth caps a task description in listings; Task descs run to
	// several sentences and the first clause is what identifies the task.
	taskShortWidth = 90

	// Picker sizing: a default for the first frame, before the terminal
	// reports its size, and a floor so a tiny terminal still shows some rows.
	defaultPickerWidth  = 80
	defaultPickerHeight = 24
	minPickerListHeight = 6

	// Form sizing. A zero-width input renders only its first placeholder
	// rune, so the default width matters before the first resize.
	formCharLimit         = 512
	defaultFormInputWidth = 40
	formWidthMargin       = 4
	minFormInputWidth     = 10
)

// Palette, one entry per docs token, named for the role it plays in vt.
var (
	// colorAccent is --color-accent-info: command names, the selection, the
	// picker title and the brand glyph.
	colorAccent = lipgloss.AdaptiveColor{Light: "#0284c7", Dark: "#38bdf8"}
	// colorDanger is --color-accent-danger: errors.
	colorDanger = lipgloss.AdaptiveColor{Light: "#dc2626", Dark: "#f87171"}
	// colorWarning is --color-accent-warning: the Heavy-task warning.
	colorWarning = lipgloss.AdaptiveColor{Light: "#b45309", Dark: "#fbbf24"}
	// colorMuted is --color-accent-default: hints, descriptions, key help.
	colorMuted = lipgloss.AdaptiveColor{Light: "#52525b", Dark: "#a1a1aa"}
)

// vtitanTheme is vt's own docs-token colors as a termkit.Theme, so vt's
// banner/picker styling runs through termkit's Capability mechanism while
// every color still comes from the docs site's tokens rather than termkit's
// own Tokyo Night default. Info is deliberately --color-accent-note, not
// another copy of colorAccent: termkit's Info role wants a hue distinct from
// Accent, and vt's own "info" token is already claimed by Accent. NewUI hands
// it to termkit.Capability.Theme; there is no global state to switch.
var vtitanTheme = termkit.Theme{
	Accent:  colorAccent,
	Danger:  colorDanger,
	Warning: colorWarning,
	Success: lipgloss.AdaptiveColor{Light: "#059669", Dark: "#34d399"}, // --color-accent-success
	Info:    lipgloss.AdaptiveColor{Light: "#4f46e5", Dark: "#818cf8"}, // --color-accent-note
	Brand:   lipgloss.AdaptiveColor{Light: "#7c3aed", Dark: "#a78bfa"}, // --color-accent-important
	Muted:   colorMuted,
}

// paletteTokens maps each palette color to the docs token it mirrors, for the
// drift test.
var paletteTokens = map[string]lipgloss.AdaptiveColor{
	"info":      colorAccent,
	"danger":    colorDanger,
	"warning":   colorWarning,
	"default":   colorMuted,
	"success":   vtitanTheme.Success,
	"note":      vtitanTheme.Info,
	"important": vtitanTheme.Brand,
}

// wordmarkGradient shades the wordmark one row at a time along the info hue,
// light to deep. Row 2 is exactly --color-accent-info in both modes; on a light
// background the scale sits two steps deeper so the top rows keep contrast.
var wordmarkGradient = []lipgloss.AdaptiveColor{
	{Light: "#0ea5e9", Dark: "#7dd3fc"},
	{Light: "#0284c7", Dark: "#38bdf8"},
	{Light: "#0369a1", Dark: "#0ea5e9"},
	{Light: "#075985", Dark: "#0284c7"},
	{Light: "#0c4a6e", Dark: "#0369a1"},
	{Light: "#082f49", Dark: "#075985"},
}
