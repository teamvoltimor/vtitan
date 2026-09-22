package vtcli

import (
	"strings"

	"github.com/charmbracelet/bubbles/list"
	tea "github.com/charmbracelet/bubbletea"
	"github.com/charmbracelet/lipgloss"
)

// pickItem is one selectable row in the interactive picker: either a domain to
// descend into, a leaf command to fill in, or the escape hatch.
type pickItem struct {
	title   string
	desc    string
	segment string
	spec    *Command
	task    string
	escape  bool
}

// pickerModel drives the command list as a stack of levels: the root lists
// domains, descending shows that domain's children. Selecting a leaf quits
// with selected set; q/ctrl+c sets cancelled.
type pickerModel struct {
	list      list.Model
	root      []pickItem
	levels    [][]pickItem
	trail     []string
	tasks     []pickItem
	selected  *pickItem
	cancelled bool

	// ui draws the header above the list and styles the list; the list gets
	// whatever height the header leaves.
	ui            UI
	width, height int

	// resolve maps a trail of segments to the rows shown at that level. It is
	// injected by App because the level data lives in the spec.
	resolve func([]string) []pickItem
}

// buildPickerRoot turns the curated spec into the root level: first-level
// domains, with the leaf commands directly under the root kept at that level.
func (a *App) buildPickerRoot() []pickItem {
	// childLevel already separates leaves from namespaces (a root command with
	// children, such as lint, must open rather than run); drop its back row.
	items := append(a.recentItems(), a.childLevel(nil)[1:]...)

	items = append(items, pickItem{
		title:  catchAllName,
		desc:   "Any Task task: browse and filter the full inventory",
		escape: true,
	})

	return items
}

// childLevel returns the rows shown after descending into segments: a back
// row, then the level's own leaf action (if the path itself is a command), its
// leaf children, and the sub-namespaces to descend into. A name that has both
// children and its own action is shown as a namespace, with its action as a
// "(run) ..." row inside it.
func (a *App) childLevel(segments []string) []pickItem {
	hasChildren := make(map[string]struct{})
	for i := range a.spec {
		path := a.spec[i].Path
		if a.spec[i].Available(a.goos) && hasPrefixPath(path, segments) && len(path) > len(segments)+1 {
			hasChildren[path[len(segments)]] = struct{}{}
		}
	}

	items := make([]pickItem, 0)
	seen := make(map[string]struct{})

	for i := range a.spec {
		command := &a.spec[i]
		path := command.Path
		if !command.Available(a.goos) || !hasPrefixPath(path, segments) {
			continue
		}

		if len(path) == len(segments) {
			items = append(items, pickItem{
				title: "(run) " + path[len(path)-1],
				desc:  command.Short,
				spec:  command,
			})

			continue
		}

		child := path[len(segments)]
		if _, ok := seen[child]; ok {
			continue
		}

		seen[child] = struct{}{}

		if _, isNamespace := hasChildren[child]; isNamespace {
			items = append(items, pickItem{title: child, desc: segmentDesc(segments, child), segment: child})

			continue
		}

		items = append(items, pickItem{title: child, desc: command.Short, spec: command})
	}

	return append([]pickItem{backRow()}, items...)
}

// taskLevel lists every task in the inventory, for the escape hatch: this is
// what replaces scrolling `task --list-all`.
func (a *App) taskLevel() []pickItem {
	entries := TaskEntries(a.tasks)
	items := make([]pickItem, 0, len(entries))

	for _, entry := range entries {
		items = append(items, pickItem{title: entry.Name, desc: entry.Short, task: entry.Name})
	}

	return items
}

// segmentDesc describes a namespace row: first-level domains have their own
// table, deeper segments share segmentHelp.
func segmentDesc(parent []string, segment string) string {
	if len(parent) == 0 {
		return rootDomainShort[segment]
	}

	return segmentHelp[segment]
}

// backRow is the row that returns to the parent level.
func backRow() pickItem {
	return pickItem{title: backRowTitle, desc: "back to the previous level", segment: backSegment}
}

// hasPrefixPath reports whether path starts with prefix.
func hasPrefixPath(path, prefix []string) bool {
	if len(path) < len(prefix) {
		return false
	}

	for i := range prefix {
		if path[i] != prefix[i] {
			return false
		}
	}

	return true
}

// Title implements list.Item.
func (i pickItem) Title() string { return i.title }

// Description implements list.Item.
func (i pickItem) Description() string { return i.desc }

// FilterValue implements list.Item.
func (i pickItem) FilterValue() string { return i.title }

// newPickerModel builds the picker rooted at the first-level domains.
func newPickerModel(
	root []pickItem,
	resolve func([]string) []pickItem,
	tasks []pickItem,
	ui UI,
) *pickerModel {
	model := &pickerModel{
		root:    root,
		list:    newPickerList(root, ui),
		levels:  [][]pickItem{root},
		tasks:   tasks,
		resolve: resolve,
		ui:      ui,
		width:   defaultPickerWidth,
		height:  defaultPickerHeight,
	}
	model.fit()
	model.updateTitle()

	return model
}

// newPickerList builds a sized list model from one level of rows.
func newPickerList(items []pickItem, ui UI) list.Model {
	entries := make([]list.Item, 0, len(items))
	for index := range items {
		entries = append(entries, items[index])
	}

	model := list.New(entries, pickerDelegate(ui), defaultPickerWidth, defaultPickerHeight)
	styleList(&model, ui)
	model.SetFilteringEnabled(true)
	model.SetShowHelp(true)
	model.SetShowStatusBar(true)
	model.SetShowPagination(true)

	return model
}

// Init implements tea.Model.
func (m *pickerModel) Init() tea.Cmd { return nil }

// Update implements tea.Model. Keys are always delegated to the list first:
// inside the filter input, enter applies the filter rather than choosing, and
// that distinction is the list's to make, not ours.
func (m *pickerModel) Update(msg tea.Msg) (tea.Model, tea.Cmd) {
	if size, isResize := msg.(tea.WindowSizeMsg); isResize {
		m.width, m.height = size.Width, size.Height
		m.fit()

		return m, nil
	}

	key, isKey := msg.(tea.KeyMsg)
	if isKey && m.list.FilterState() != list.Filtering {
		switch key.String() {
		case "ctrl+c":
			m.cancelled = true

			return m, tea.Quit
		case "q":
			// Quit from any level. Inside a submenu there is no way to type
			// into the picker, so q cannot be mistaken for input; the back
			// row and esc are the ways up.
			m.cancelled = true

			return m, tea.Quit
		case "esc":
			// Back out one level, and quit from the root: a single key that
			// widens as you leave, so it never traps you at the top.
			if m.canPop() {
				m.pop()

				return m, nil
			}

			m.cancelled = true

			return m, tea.Quit
		case "enter":
			if item, selected := m.list.SelectedItem().(pickItem); selected {
				return m.choose(item)
			}
		}
	}

	var cmd tea.Cmd
	m.list, cmd = m.list.Update(msg)

	return m, cmd
}

// View implements tea.Model.
func (m *pickerModel) View() string {
	return m.ui.Header(m.width, m.height) + "\n" + m.list.View()
}

// fit sizes the list to the terminal minus the header. Every level change
// builds a new list, so it has to be re-applied there too, not only on resize.
func (m *pickerModel) fit() {
	used := lipgloss.Height(m.ui.Header(m.width, m.height))

	m.list.SetSize(m.width, max(m.height-used, minPickerListHeight))
}

// choose descends into a domain or quits with a leaf/escape selection.
func (m *pickerModel) choose(item pickItem) (tea.Model, tea.Cmd) {
	switch {
	case item.escape:
		m.push(catchAllName, append([]pickItem{backRow()}, m.tasks...))

		return m, nil
	case item.task != "", item.spec != nil:
		m.selected = &item

		return m, tea.Quit
	case item.segment == backSegment:
		m.pop()

		return m, nil
	case item.segment != "":
		m.descend(item.segment)

		return m, nil
	default:
		return m, nil
	}
}

// descend pushes the children of segment onto the level stack.
func (m *pickerModel) descend(segment string) {
	m.push(segment, m.resolve(append(m.trail, segment)))
}

// push shows level as the child of the current one, named segment.
func (m *pickerModel) push(segment string, level []pickItem) {
	m.trail = append(m.trail, segment)
	m.levels = append(m.levels, level)
	m.list = newPickerList(level, m.ui)
	m.fit()
	m.updateTitle()
}

// pop returns to the previous level.
func (m *pickerModel) pop() {
	if len(m.levels) <= 1 {
		return
	}

	m.levels = m.levels[:len(m.levels)-1]
	m.trail = m.trail[:len(m.trail)-1]
	m.list = newPickerList(m.levels[len(m.levels)-1], m.ui)
	m.fit()
	m.updateTitle()
}

// canPop reports whether popping is possible (not at the root).
func (m *pickerModel) canPop() bool { return len(m.levels) > 1 }

// updateTitle refreshes the list title with the breadcrumb.
func (m *pickerModel) updateTitle() {
	if len(m.trail) == 0 {
		m.list.Title = "pick a command"

		return
	}

	m.list.Title = "vt " + strings.Join(m.trail, " › ")
}

// pickerDelegate renders list rows in the palette: the selected row in the
// accent colour instead of bubbles' default magenta.
func pickerDelegate(ui UI) list.DefaultDelegate {
	delegate := list.NewDefaultDelegate()
	if !ui.color {
		return delegate
	}

	delegate.Styles.SelectedTitle = delegate.Styles.SelectedTitle.
		Foreground(colorAccent).BorderLeftForeground(colorAccent)
	delegate.Styles.SelectedDesc = delegate.Styles.SelectedDesc.
		Foreground(colorAccent).BorderLeftForeground(colorAccent)
	delegate.Styles.NormalDesc = delegate.Styles.NormalDesc.Foreground(colorMuted)

	return delegate
}

// styleList puts the list chrome (title, filter prompt) in the palette.
func styleList(model *list.Model, ui UI) {
	if !ui.color {
		return
	}

	model.Styles.Title = model.Styles.Title.UnsetBackground().Bold(true).Foreground(colorAccent)
	model.Styles.FilterPrompt = model.Styles.FilterPrompt.Foreground(colorAccent)
	model.Styles.FilterCursor = model.Styles.FilterCursor.Foreground(colorAccent)
}
