package vtcli

import (
	"strings"
	"testing"
)

// TestTaskEntriesSortsAndTrims checks the inventory listing: sorted by name,
// first description line only, long lines capped.
func TestTaskEntriesSortsAndTrims(t *testing.T) {
	t.Parallel()

	long := strings.Repeat("x", taskShortWidth+10)
	entries := TaskEntries([]TaskInfo{
		{Name: "sim:test", Desc: "Simulator tests\nsecond line"},
		{Name: "go:build", Desc: long},
	})

	if entries[0].Name != "go:build" || entries[1].Name != "sim:test" {
		t.Fatalf("not sorted by name: %+v", entries)
	}

	if got := len([]rune(entries[0].Short)); got != taskShortWidth {
		t.Errorf("long description is %d runes, want %d", got, taskShortWidth)
	}

	if entries[1].Short != "Simulator tests" {
		t.Errorf("short = %q, want only the first line", entries[1].Short)
	}
}

// TestCommandLineMatchesInvocation checks that the confirmation screen shows
// what buildExtra actually forwards: the Task variable name, not the flag
// name, and the passthrough after `--`.
func TestCommandLineMatchesInvocation(t *testing.T) {
	t.Parallel()

	command := Command{
		Task:        "go:hw:run",
		Passthrough: true,
		Flags: []Flag{
			{Name: "host", Var: "HOST", Default: "rpi-5-local"},
			{Name: "pkg", Var: "PKG", Default: "lidar"},
		},
	}

	host := newTextField("host", "", false, FlagString)
	host.varName = "HOST"
	host.defaultValue = "rpi-5-local"
	host.input.SetValue("rpi-5-remote")

	pkg := newTextField("pkg", "", false, FlagString)
	pkg.varName = "PKG"
	pkg.defaultValue = "lidar"
	pkg.input.SetValue("lidar")

	args := newTextField("args", "", false, FlagString)
	args.placement = placePassthrough
	args.input.SetValue("--scenario 5")

	model := formModel{taskName: command.Task, fields: []*formField{host, pkg, args}}

	want := "task go:hw:run HOST=rpi-5-remote -- --scenario 5"
	if got := model.commandLine(); got != want {
		t.Errorf("preview = %q, want %q", got, want)
	}

	extra, err := buildExtra(command, map[string]string{"host": "rpi-5-remote", "pkg": "lidar"}, "--scenario 5")
	if err != nil {
		t.Fatal(err)
	}

	if got := "task " + command.Task + " " + strings.Join(extra, " "); got != want {
		t.Errorf("invocation = %q, want %q", got, want)
	}
}

// TestOwnerDomainMatchesBareUmbrella checks that an umbrella domain owns its
// bare verb and its children, but not a task that merely shares the letters.
func TestOwnerDomainMatchesBareUmbrella(t *testing.T) {
	t.Parallel()

	domains := []Domain{{ID: "lint", TaskPrefix: "lint:"}}

	for name, want := range map[string]bool{"lint": true, "lint:fix": true, "linter": false, "robot:lint": false} {
		if _, got := ownerDomain(name, domains); got != want {
			t.Errorf("ownerDomain(%q) owned = %v, want %v", name, got, want)
		}
	}
}
