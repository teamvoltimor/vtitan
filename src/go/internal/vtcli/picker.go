package vtcli

import (
	"strings"

	"github.com/charmbracelet/bubbles/list"
	tea "github.com/charmbracelet/bubbletea"
)

// pickItem is one selectable row in the interactive picker: either a domain to
// descend into, a leaf command to fill in, or the escape hatch.
type pickItem struct {
	title   string
	desc    string
	segment string
	spec    *Command
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
	selected  *pickItem
	cancelled bool

	// resolve maps a trail of segments to the rows shown at that level. It is
	// injected by App because the level data lives in the spec.
	resolve func([]string) []pickItem
}

// buildPickerRoot turns the curated spec into the root level: first-level
// domains, with the leaf commands directly under the root kept at that level.
func (a *App) buildPickerRoot() []pickItem {
	items := make([]pickItem, 0)

	for i := range a.spec {
		command := &a.spec[i]
		if len(command.Path) == 1 {
			items = append(items, pickItem{title: command.Path[0], desc: command.Short, spec: command})

			continue
		}

		segment := command.Path[0]
		if !hasPickItem(items, segment) {
			items = append(items, pickItem{
				title:   segment,
				desc:    rootDomainShort[segment],
				segment: segment,
			})
		}
	}

	items = append(items, pickItem{
		title:  "run",
		desc:   "Escape hatch: any Task task, with its arguments",
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
		if hasPrefixPath(path, segments) && len(path) > len(segments)+1 {
			hasChildren[path[len(segments)]] = struct{}{}
		}
	}

	items := make([]pickItem, 0)
	seen := make(map[string]struct{})

	for i := range a.spec {
		command := &a.spec[i]
		path := command.Path
		if !hasPrefixPath(path, segments) {
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
			items = append(items, pickItem{title: child, desc: segmentHelp[child], segment: child})

			continue
		}

		items = append(items, pickItem{title: child, desc: command.Short, spec: command})
	}

	return append([]pickItem{{title: backRowTitle, desc: "back to the previous level", segment: backSegment}}, items...)
}

// hasPickItem reports whether a row with the given title already exists.
func hasPickItem(items []pickItem, title string) bool {
	for _, item := range items {
		if item.title == title {
			return true
		}
	}

	return false
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
func newPickerModel(root []pickItem, resolve func([]string) []pickItem) *pickerModel {
	model := newPickerList(root)
	model.Title = "vt — pick a command"

	return &pickerModel{
		root:    root,
		list:    model,
		levels:  [][]pickItem{root},
		resolve: resolve,
	}
}

// newPickerList builds a sized list model from one level of rows.
func newPickerList(items []pickItem) list.Model {
	entries := make([]list.Item, 0, len(items))
	for index := range items {
		entries = append(entries, items[index])
	}

	model := list.New(entries, list.NewDefaultDelegate(), defaultPickerWidth, defaultPickerHeight)
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
		m.list.SetSize(size.Width, size.Height)

		return m, nil
	}

	key, isKey := msg.(tea.KeyMsg)
	if isKey && m.list.FilterState() != list.Filtering {
		switch key.String() {
		case "ctrl+c":
			m.cancelled = true

			return m, tea.Quit
		case "q":
			if !m.canPop() {
				m.cancelled = true

				return m, tea.Quit
			}
		case "esc":
			if m.canPop() {
				m.pop()

				return m, nil
			}
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
func (m *pickerModel) View() string { return m.list.View() }

// choose descends into a domain or quits with a leaf/escape selection.
func (m *pickerModel) choose(item pickItem) (tea.Model, tea.Cmd) {
	switch {
	case item.escape, item.spec != nil:
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
	m.trail = append(m.trail, segment)
	level := m.resolve(m.trail)
	m.levels = append(m.levels, level)
	m.list = newPickerList(level)
	m.updateTitle()
}

// pop returns to the previous level.
func (m *pickerModel) pop() {
	if len(m.levels) <= 1 {
		return
	}

	m.levels = m.levels[:len(m.levels)-1]
	m.trail = m.trail[:len(m.trail)-1]
	m.list = newPickerList(m.levels[len(m.levels)-1])
	m.updateTitle()
}

// canPop reports whether popping is possible (not at the root).
func (m *pickerModel) canPop() bool { return len(m.levels) > 1 }

// updateTitle refreshes the list title with the breadcrumb.
func (m *pickerModel) updateTitle() {
	if len(m.trail) == 0 {
		m.list.Title = "vt — pick a command"

		return
	}

	m.list.Title = "vt " + strings.Join(m.trail, " › ")
}
