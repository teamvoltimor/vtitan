package vtcli

import (
	"os"
	"path/filepath"
	"slices"
	"strings"
)

// defaultRecentPath is where picks are remembered, in the user cache
// directory; "" when there is none, which turns remembering off.
func defaultRecentPath() string {
	dir, err := os.UserCacheDir()
	if err != nil {
		return ""
	}

	return filepath.Join(dir, recentDir, recentFile)
}

// loadRecent returns the remembered picks, most recent first. A missing or
// unreadable file is just an empty history.
func loadRecent(path string) []string {
	if path == "" {
		return nil
	}

	raw, err := os.ReadFile(path)
	if err != nil {
		return nil
	}

	var entries []string
	for line := range strings.SplitSeq(string(raw), "\n") {
		if line = strings.TrimSpace(line); line != "" {
			entries = append(entries, line)
		}
	}

	return entries
}

// remember records entry as the most recent pick, capped at recentLimit.
// History is a convenience, so a failure to write it is not an error.
func (a *App) remember(entry string) {
	if a.recentPath == "" {
		return
	}

	entries := slices.DeleteFunc(loadRecent(a.recentPath), func(old string) bool { return old == entry })
	entries = append([]string{entry}, entries...)

	if len(entries) > recentLimit {
		entries = entries[:recentLimit]
	}

	if err := os.MkdirAll(filepath.Dir(a.recentPath), recentDirPerm); err != nil {
		return
	}

	if err := os.WriteFile(a.recentPath, []byte(strings.Join(entries, "\n")+"\n"), recentFilePerm); err != nil {
		return
	}
}

// recentItems turns the history into picker rows, dropping entries whose
// command or task no longer exists or does not run here.
func (a *App) recentItems() []pickItem {
	var items []pickItem

	for _, entry := range loadRecent(a.recentPath) {
		if path, isCommand := strings.CutPrefix(entry, recentCommandPrefix); isCommand {
			for i := range a.spec {
				command := &a.spec[i]
				if strings.Join(command.Path, " ") == path && command.Available(a.goos) {
					items = append(items, pickItem{title: recentMark + path, desc: command.Short, spec: command})
				}
			}

			continue
		}

		if name, isTask := strings.CutPrefix(entry, recentTaskPrefix); isTask && KnownTask(a.tasks, name) {
			items = append(
				items,
				pickItem{title: recentMark + catchAllName + " " + name, desc: taskShort(a.tasks, name), task: name},
			)
		}
	}

	return items
}
