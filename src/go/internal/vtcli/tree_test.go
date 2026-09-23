package vtcli

import (
	"strings"
	"testing"

	tea "github.com/charmbracelet/bubbletea"
	"github.com/spf13/cobra"
)

// newSpecApp assembles the real curated tree without a task inventory.
func newSpecApp(t *testing.T) *App {
	t.Helper()

	app, err := NewApp("", &cobra.Command{Use: "vt"}, NewUI(), CuratedSpec(), nil)
	if err != nil {
		t.Fatalf("build app: %v", err)
	}

	return app
}

// TestNamespacesDescribed checks that every namespace in the tree has a
// description, and that namespaceShort holds no entry for a path that is not
// one, so a restructure cannot leave empty rows or stale text behind.
func TestNamespacesDescribed(t *testing.T) {
	t.Parallel()

	app := newSpecApp(t)
	namespaces := make(map[string]struct{})

	var walk func(cmd *cobra.Command, path []string)
	walk = func(cmd *cobra.Command, path []string) {
		for _, child := range cmd.Commands() {
			childPath := append(append([]string{}, path...), child.Name())
			if child.RunE == nil && child.Run == nil {
				key := strings.Join(childPath, " ")
				namespaces[key] = struct{}{}

				if child.Short == "" {
					t.Errorf("namespace `vt %s` has no description; add it to namespaceShort", key)
				}
			}

			walk(child, childPath)
		}
	}
	walk(app.Root, nil)

	for key := range namespaceShort {
		if _, ok := namespaces[key]; !ok {
			t.Errorf("namespaceShort describes %q, which is not a namespace in the tree", key)
		}
	}
}

// TestRootGroups checks that every first-level command sits in exactly one
// root group and every group member exists, so nothing falls out of --help's
// sections or the picker's order.
func TestRootGroups(t *testing.T) {
	t.Parallel()

	app := newSpecApp(t)
	grouped := make(map[string]string)

	for _, group := range rootGroups {
		for _, member := range group.members {
			if previous, dup := grouped[member]; dup {
				t.Errorf("%q is in both %q and %q", member, previous, group.id)
			}

			grouped[member] = group.id

			if findChild(app.Root, member) == nil {
				t.Errorf("root group %q lists %q, which is not a command", group.id, member)
			}
		}
	}

	for _, child := range app.Root.Commands() {
		if _, ok := grouped[child.Name()]; !ok {
			t.Errorf("first-level command %q is in no root group", child.Name())
		}
	}
}

// TestPickerRootFollowsGroups checks that the picker lists the first level in
// rootGroups order, the same order --help and the home menu use.
func TestPickerRootFollowsGroups(t *testing.T) {
	t.Parallel()

	app := newSpecApp(t)
	app.recentPath = ""

	previous := -1

	for _, item := range app.buildPickerRoot() {
		if item.escape || item.groupHeader {
			continue
		}

		rank := rootRank(item.title)
		if rank < previous {
			t.Errorf("picker lists %q out of rootGroups order", item.title)
		}

		previous = rank
	}
}

// TestPickerRootIsSectioned checks the root carries a header per root group
// that has members, and that the list opens on a row rather than a header.
func TestPickerRootIsSectioned(t *testing.T) {
	t.Parallel()

	app := newSpecApp(t)
	app.recentPath = ""

	headers := make(map[string]bool)
	for _, row := range app.buildPickerRoot() {
		if row.groupHeader {
			headers[row.title] = true
		}
	}

	for _, group := range rootGroups {
		hasMember := false
		for _, member := range group.members {
			if findChild(app.Root, member) != nil {
				hasMember = true

				break
			}
		}

		if hasMember && !headers[group.title] {
			t.Errorf("root group %q has no header in the picker", group.title)
		}
	}

	picker := newPickerModel(app.buildPickerRoot(), app.childLevel, app.taskLevel(), NewUI())
	if row, ok := picker.list.SelectedItem().(pickItem); !ok || row.groupHeader {
		t.Errorf("the picker opens on a header: %+v", row)
	}
}

// TestPickerRendersGroupHeaders checks the root view draws a group header as a
// label, not as an ordinary row.
func TestPickerRendersGroupHeaders(t *testing.T) {
	t.Parallel()

	app := newSpecApp(t)
	app.recentPath = ""

	picker := newPickerModel(app.buildPickerRoot(), app.childLevel, app.taskLevel(), NewUI())
	if view := picker.list.View(); !strings.Contains(view, "Robot and simulation:") {
		t.Errorf("the root view has no group header:\n%s", view)
	}
}

// TestPickerSkipsHeaders checks the cursor never lands on a group header.
func TestPickerSkipsHeaders(t *testing.T) {
	t.Parallel()

	app := newSpecApp(t)
	app.recentPath = ""

	picker := newPickerModel(app.buildPickerRoot(), app.childLevel, app.taskLevel(), NewUI())

	for range len(picker.list.Items()) + 2 {
		if row, ok := picker.list.SelectedItem().(pickItem); !ok || row.groupHeader {
			t.Fatalf("the cursor landed on a header: %+v", row)
		}

		picker.Update(tea.KeyMsg{Type: tea.KeyDown})
	}
}

// TestNestedLevelsAreGrouped checks a level whose children span groups is
// sectioned by command group, while the root keeps its rootGroups sections.
func TestNestedLevelsAreGrouped(t *testing.T) {
	t.Parallel()

	app := newSpecApp(t)
	app.recentPath = ""

	headers := func(level []pickItem) map[string]bool {
		found := make(map[string]bool)
		for _, row := range level {
			if row.groupHeader {
				found[row.title] = true
			}
		}

		return found
	}

	if got := headers(app.childLevel([]string{"go"})); !got["Run:"] || !got["Check:"] {
		t.Errorf("the go level is not sectioned: %v", got)
	}
}

// TestPickerRowsDescribed walks every picker level on every dev platform and
// checks that no row is blank and no namespace opens onto nothing, since the
// picker builds its levels from the spec rather than from the cobra tree.
func TestPickerRowsDescribed(t *testing.T) {
	t.Parallel()

	for _, goos := range devPlatforms {
		app := newSpecApp(t)
		app.goos, app.recentPath = goos, ""

		var walk func(trail []string, rows []pickItem)
		walk = func(trail []string, rows []pickItem) {
			for _, row := range rows {
				if row.escape || row.segment == backSegment || row.groupHeader {
					continue
				}

				if row.desc == "" {
					t.Errorf("%s: picker row %q under `vt %s` has no description",
						goos, row.title, strings.Join(trail, " "))
				}

				if row.segment == "" {
					continue
				}

				childTrail := append(append([]string{}, trail...), row.segment)

				level := app.childLevel(childTrail)
				if len(level) < 2 {
					t.Errorf("%s: `vt %s` opens onto nothing", goos, strings.Join(childTrail, " "))
				}

				walk(childTrail, level)
			}
		}
		walk(nil, app.buildPickerRoot())
	}
}
