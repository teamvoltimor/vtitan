package vtcli

import (
	"fmt"
	"path/filepath"
	"slices"
	"testing"
)

// TestRecentHistory checks the picker's memory: newest first, no duplicates,
// capped at recentLimit, and rows for commands or tasks that no longer exist
// or do not run here are dropped rather than offered.
func TestRecentHistory(t *testing.T) {
	t.Parallel()

	app := &App{
		recentPath: filepath.Join(t.TempDir(), recentDir, recentFile),
		goos:       "linux",
		spec: []Command{
			{Path: []string{"sim", "view"}, Task: "sim:navigate:visualize:all", Short: "watch"},
			{
				Path:      []string{"fleet", "setup", "route", "add"},
				Task:      "windows:route:add",
				Platforms: []string{"windows"},
			},
		},
		tasks: []TaskInfo{{Name: "gen:corpus:all", Desc: "both corpora"}},
	}

	for i := range recentLimit + 2 {
		app.remember(fmt.Sprintf("cmd:filler %d", i))
	}

	app.remember(recentCommandPrefix + "fleet setup route add")
	app.remember(recentTaskPrefix + "gen:corpus:all")
	app.remember(recentCommandPrefix + "sim view")
	app.remember(recentTaskPrefix + "gen:corpus:all")

	entries := loadRecent(app.recentPath)
	if len(entries) != recentLimit {
		t.Fatalf("kept %d entries, want the cap %d: %v", len(entries), recentLimit, entries)
	}

	if entries[0] != recentTaskPrefix+"gen:corpus:all" || entries[1] != recentCommandPrefix+"sim view" {
		t.Errorf("not newest-first without duplicates: %v", entries)
	}

	items := app.recentItems()
	titles := make([]string, 0, len(items))

	for _, item := range items {
		titles = append(titles, item.title)
	}

	want := []string{recentMark + catchAllName + " gen:corpus:all", recentMark + "sim view"}
	if !slices.Equal(titles, want) {
		t.Errorf("picker rows = %v, want %v (fillers unknown, route add windows-only)", titles, want)
	}
}
