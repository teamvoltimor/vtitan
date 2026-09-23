package vtcli

import (
	"strings"
	"testing"

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
		if item.escape {
			continue
		}

		rank := rootRank(item.title)
		if rank < previous {
			t.Errorf("picker lists %q out of rootGroups order", item.title)
		}

		previous = rank
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
				if row.escape || row.segment == backSegment {
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
