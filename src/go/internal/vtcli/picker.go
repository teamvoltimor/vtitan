package vtcli

import (
	"fmt"
	"io"
	"slices"
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
	// groupHeader marks a section label in the root list: drawn as a rule and
	// a title, and never selectable.
	groupHeader bool
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

// pickerRowsDelegate renders list rows in the palette: the selected row in the
// accent colour instead of bubbles' default magenta, and a group header as a
// rule plus its label rather than a selectable row.
type pickerRowsDelegate struct {
	inner list.DefaultDelegate
	ui    UI
}

// recentGroupTitle heads the picker's recent rows.
const recentGroupTitle = "Recent:"

// buildPickerRoot turns the curated spec into the root level: the recent rows
// and then each first-level domain under its root group's header, so the
// picker's sections match --help and the home menu. The leaf commands directly
// under the root are kept at this level.
func (a *App) buildPickerRoot() []pickItem {
	// childLevel already separates leaves from namespaces (a root command with
	// children, such as lint, must open rather than run); drop its back row.
	domains := a.childLevel(nil)[1:]

	byTitle := make(map[string]pickItem, len(domains))
	for _, domain := range domains {
		byTitle[domain.title] = domain
	}

	items := make([]pickItem, 0, len(domains)+len(rootGroups)+1)

	if recent := a.recentItems(); len(recent) > 0 {
		items = append(items, groupHeaderRow(recentGroupTitle))
		items = append(items, recent...)
	}

	for _, group := range rootGroups {
		members := make([]pickItem, 0, len(group.members))

		for _, member := range group.members {
			if member == catchAllName {
				members = append(members, pickItem{
					title:  catchAllName,
					desc:   "Any Task task: browse and filter the full inventory",
					escape: true,
				})

				continue
			}

			if item, ok := byTitle[member]; ok {
				members = append(members, item)
				delete(byTitle, member)
			}
		}

		if len(members) == 0 {
			continue
		}

		items = append(items, groupHeaderRow(group.title))
		items = append(items, members...)
	}

	// A domain in no root group would otherwise be unreachable; TestRootGroups
	// keeps this empty.
	for _, domain := range domains {
		if _, ok := byTitle[domain.title]; ok {
			items = append(items, domain)
		}
	}

	return items
}

// groupHeaderRow is a non-selectable section label.
func groupHeaderRow(title string) pickItem {
	return pickItem{title: title, groupHeader: true}
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
			// A command with children (sim view) opens as a namespace, and
			// its own description is what --help shows for it too.
			desc := segmentDesc(segments, child)
			if desc == "" && len(path) == len(segments)+1 {
				desc = command.Short
			}

			items = append(items, pickItem{title: child, desc: desc, segment: child})

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

// segmentDesc describes a namespace row by its full path.
func segmentDesc(parent []string, segment string) string {
	return namespaceShort[strings.Join(append(slices.Clone(parent), segment), " ")]
}

// rootRank is a first-level name's position in rootGroups, so the picker's
// root follows the same order as --help and the home menu rather than the
// order the spec tables happen to be concatenated in. Unlisted names sort
// last, in spec order.
func rootRank(name string) int {
	rank := 0

	for _, group := range rootGroups {
		for _, member := range group.members {
			if member == name {
				return rank
			}

			rank++
		}
	}

	return rank
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
	selectFirstRow(&model)

	return model
}

// selectFirstRow moves the cursor off a leading group header, so the list
// opens on a row that can actually be chosen.
func selectFirstRow(model *list.Model) {
	for range model.Items() {
		row, ok := model.SelectedItem().(pickItem)
		if !ok || !row.groupHeader {
			return
		}

		model.CursorDown()
	}
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

	before := m.list.Index()

	var cmd tea.Cmd
	m.list, cmd = m.list.Update(msg)
	m.skipHeaders(before)

	return m, cmd
}

// View implements tea.Model.
func (m *pickerModel) View() string {
	return m.ui.Header(m.width, m.height) + "\n" + m.list.View()
}

// skipHeaders moves the cursor off a group header, which cannot be chosen.
// Headers are not adjacent, but a clamped move (home, or the first row) can
// land on one, so it keeps moving while the selection is a header.
func (m *pickerModel) skipHeaders(before int) {
	for range m.list.Items() {
		row, ok := m.list.SelectedItem().(pickItem)
		if !ok || !row.groupHeader {
			return
		}

		if m.list.Index() > before || m.list.Index() == 0 {
			m.list.CursorDown()
		} else {
			m.list.CursorUp()
		}
	}
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

// setRoot replaces the root level, keeping the cursor where it was if the
// root is showing; the session calls it after a run so the run shows up in
// the recent rows.
func (m *pickerModel) setRoot(root []pickItem) {
	m.root = root
	m.levels[0] = root

	if len(m.levels) == 1 {
		index := m.list.Index()
		m.list = newPickerList(root, m.ui)
		m.list.Select(min(index, len(root)-1))
		m.fit()
		m.updateTitle()
	}
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

// Height implements list.ItemDelegate.
func (d pickerRowsDelegate) Height() int { return d.inner.Height() }

// Spacing implements list.ItemDelegate.
func (d pickerRowsDelegate) Spacing() int { return d.inner.Spacing() }

// Update implements list.ItemDelegate.
func (d pickerRowsDelegate) Update(msg tea.Msg, m *list.Model) tea.Cmd { return d.inner.Update(msg, m) }

// Render implements list.ItemDelegate.
func (d pickerRowsDelegate) Render(w io.Writer, m list.Model, index int, item list.Item) {
	row, ok := item.(pickItem)
	if !ok {
		return
	}

	if row.groupHeader {
		rule := strings.Repeat("─", max(m.Width(), 1))
		fmt.Fprintf(w, "%s\n%s", d.ui.Muted(rule), d.ui.Muted("  "+row.title))

		return
	}

	d.inner.Render(w, m, index, item)
}

// pickerDelegate builds the row delegate in the palette.
func pickerDelegate(ui UI) list.ItemDelegate {
	delegate := list.NewDefaultDelegate()
	if ui.color {
		delegate.Styles.SelectedTitle = delegate.Styles.SelectedTitle.
			Foreground(colorAccent).BorderLeftForeground(colorAccent)
		delegate.Styles.SelectedDesc = delegate.Styles.SelectedDesc.
			Foreground(colorAccent).BorderLeftForeground(colorAccent)
		delegate.Styles.NormalDesc = delegate.Styles.NormalDesc.Foreground(colorMuted)
	}

	return pickerRowsDelegate{inner: delegate, ui: ui}
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
