package vtcli

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"slices"
	"strings"
)

// TaskInfo is the subset of `task --list-all --json` vt needs. Task does not
// expose variables or requires, so typed flags stay a manual decision per
// command (see the plan, section 1).
type TaskInfo struct {
	Name    string   `json:"name"`
	Desc    string   `json:"desc"`
	Aliases []string `json:"aliases"`
}

// tasksEnvelope mirrors the top-level object Task emits.
type tasksEnvelope struct {
	Tasks []TaskInfo `json:"tasks"`
}

// taskBinary is the Task runner vt wraps. Keeping it a constant (rather than a
// flag) matches the plan: vt is the discovery surface, Task is the engine.
const taskBinary = "task"

// LoadTasks returns every task Task knows about, rooted at repoRoot. It shells
// out to `task -d <repoRoot> --list-all --json` so there is one inventory and
// no second Task parser to drift from the first.
func LoadTasks(ctx context.Context, repoRoot string) ([]TaskInfo, error) {
	if _, err := exec.LookPath(taskBinary); err != nil {
		return nil, fmt.Errorf("task binary not found in PATH: %w", err)
	}

	cmd := exec.CommandContext(ctx, taskBinary, "-d", repoRoot, "--list-all", "--json")

	raw, err := cmd.Output()
	if err != nil {
		return nil, fmt.Errorf("run task --list-all --json: %w", err)
	}

	var envelope tasksEnvelope
	if unmarshalErr := json.Unmarshal(raw, &envelope); unmarshalErr != nil {
		return nil, fmt.Errorf("parse task list: %w", unmarshalErr)
	}

	return envelope.Tasks, nil
}

// FindRepoRoot walks up from start until it finds the checkout root, the
// directory holding .git.
func FindRepoRoot(start string) (string, error) {
	dir, err := filepath.Abs(start)
	if err != nil {
		return "", fmt.Errorf("resolve %q: %w", start, err)
	}

	for {
		if _, statErr := os.Stat(filepath.Join(dir, ".git")); statErr == nil {
			return dir, nil
		}

		parent := filepath.Dir(dir)
		if parent == dir {
			return "", fmt.Errorf("no .git above %q", start)
		}

		dir = parent
	}
}

// TaskEntries turns the inventory into listing rows, sorted by name, each with
// the first line of its description.
func TaskEntries(tasks []TaskInfo) []MenuEntry {
	entries := make([]MenuEntry, 0, len(tasks))
	for _, task := range tasks {
		short, _, _ := strings.Cut(strings.TrimSpace(task.Desc), "\n")
		if runes := []rune(short); len(runes) > taskShortWidth {
			short = string(runes[:taskShortWidth-1]) + "…"
		}

		entries = append(entries, MenuEntry{Name: task.Name, Short: short})
	}

	slices.SortFunc(entries, func(left, right MenuEntry) int { return strings.Compare(left.Name, right.Name) })

	return entries
}

// KnownTask reports whether name (or one of its aliases) is in the inventory.
func KnownTask(tasks []TaskInfo, name string) bool {
	for _, task := range tasks {
		if task.Name == name || slices.Contains(task.Aliases, name) {
			return true
		}
	}

	return false
}

// ownerDomain returns the curated domain a task name belongs to, if any.
func ownerDomain(name string, domains []Domain) (string, bool) {
	for _, domain := range domains {
		if strings.HasPrefix(name, domain.TaskPrefix) || name == strings.TrimSuffix(domain.TaskPrefix, ":") {
			return domain.ID, true
		}
	}

	return "", false
}
