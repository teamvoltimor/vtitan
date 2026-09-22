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
