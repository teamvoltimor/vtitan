package vtcli

import (
	"maps"
	"os"
	"path/filepath"
	"slices"
	"strings"
	"testing"

	"gopkg.in/yaml.v3"
)

// allPlatforms stands for "runs everywhere" in effectivePlatforms.
const allPlatforms = "*"

// devPlatforms are the OSes a dev machine can be; a task covering all of them
// (one command per platform, like windows:ping) runs everywhere.
var devPlatforms = []string{"darwin", "linux", "windows"}

// TestPlatformsMatchTaskfiles is anti-drift check 5. A task whose commands
// only run on some OSes (Task's `platforms:`, on the task or on every command
// that does real work) is a silent no-op elsewhere: Task skips the commands
// and exits 0. vt hides such commands on other platforms, so the spec must
// declare exactly the platforms the Taskfile does. The spec may be stricter
// than a Taskfile that runs everywhere (a task that half works on Linux is
// still Windows-only in intent), never looser.
func TestPlatformsMatchTaskfiles(t *testing.T) {
	t.Parallel()

	root, err := FindRepoRoot(".")
	if err != nil {
		t.Fatalf("find repo root: %v", err)
	}

	defs := taskDefinitions(t, root)

	for _, command := range CuratedSpec() {
		declared := slices.Sorted(slices.Values(command.Platforms))

		for _, task := range command.Tasks() {
			effective := effectivePlatforms(defs, task, map[string]bool{})
			if slices.Contains(effective, allPlatforms) {
				continue
			}

			if !slices.Equal(declared, effective) {
				t.Errorf("vt %s: %s only runs on %v, the spec says %v",
					strings.Join(command.Path, " "), task, effective, orAll(declared))
			}
		}
	}
}

// TestPlatformHiding checks that a platform-limited command disappears from
// the picker elsewhere, and so does a namespace left with nothing in it.
func TestPlatformHiding(t *testing.T) {
	t.Parallel()

	spec := []Command{
		{Path: []string{"fleet", "ping"}, Task: "windows:ping"},
		{Path: []string{"fleet", "route", "add"}, Task: "windows:route:add", Platforms: []string{"windows"}},
	}

	for goos, wantRoute := range map[string]bool{"windows": true, "linux": false} {
		app := &App{spec: spec, goos: goos}

		level := app.childLevel([]string{"fleet"})
		titles := make([]string, 0, len(level))

		for _, item := range level {
			titles = append(titles, item.title)
		}

		if got := slices.Contains(titles, "route"); got != wantRoute {
			t.Errorf("on %s the route namespace shown = %v, want %v (rows %v)", goos, got, wantRoute, titles)
		}
	}
}

// effectivePlatforms is the set of OSes a task does real work on: its own
// `platforms:`, else the union over its commands, deps and task calls, where a
// command with no `platforms:` counts as everywhere and an echo as nothing.
func effectivePlatforms(defs map[string]map[string]any, name string, seen map[string]bool) []string {
	if seen[name] {
		return []string{allPlatforms}
	}

	seen[name] = true

	def, known := defs[name]
	if !known {
		return []string{allPlatforms}
	}

	if own := platformList(def["platforms"]); own != nil {
		return own
	}

	found := make(map[string]bool)
	add := func(list []string) {
		for _, goos := range list {
			found[goos] = true
		}
	}

	for _, dep := range asList(def["deps"]) {
		add(callPlatforms(defs, dep, seen))
	}

	for _, cmd := range asList(def["cmds"]) {
		add(cmdPlatforms(defs, cmd, seen))
	}

	covered := true
	for _, goos := range devPlatforms {
		covered = covered && found[goos]
	}

	if len(found) == 0 || found[allPlatforms] || covered {
		return []string{allPlatforms}
	}

	return slices.Sorted(maps.Keys(found))
}

// cmdPlatforms classifies one entry of a task's cmds.
func cmdPlatforms(defs map[string]map[string]any, cmd any, seen map[string]bool) []string {
	switch entry := cmd.(type) {
	case string:
		if isEcho(entry) {
			return nil
		}

		return []string{allPlatforms}
	case map[string]any:
		if own := platformList(entry["platforms"]); own != nil {
			return own
		}

		if _, isCall := entry["task"]; isCall {
			return callPlatforms(defs, entry, seen)
		}

		if text, isCmd := entry["cmd"].(string); isCmd && !isEcho(text) {
			return []string{allPlatforms}
		}
	}

	return nil
}

// callPlatforms follows a dep or `task:` call to the called task.
func callPlatforms(defs map[string]map[string]any, call any, seen map[string]bool) []string {
	switch target := call.(type) {
	case string:
		return effectivePlatforms(defs, target, seen)
	case map[string]any:
		if name, ok := target["task"].(string); ok {
			return effectivePlatforms(defs, name, seen)
		}
	}

	return nil
}

// taskDefinitions indexes every task of the root Taskfile and its includes by
// the name Task gives it: bare for flattened includes, namespaced otherwise.
func taskDefinitions(t *testing.T, root string) map[string]map[string]any {
	t.Helper()

	defs := make(map[string]map[string]any)
	collectTasks(t, filepath.Join(root, "Taskfile.yml"), "", defs)

	return defs
}

// collectTasks adds one Taskfile's tasks under prefix, then its includes'.
func collectTasks(t *testing.T, path, prefix string, defs map[string]map[string]any) {
	t.Helper()

	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatalf("read %s: %v", path, err)
	}

	var doc struct {
		Tasks    map[string]any `yaml:"tasks"`
		Includes map[string]any `yaml:"includes"`
	}
	if parseErr := yaml.Unmarshal(raw, &doc); parseErr != nil {
		t.Fatalf("parse %s: %v", path, parseErr)
	}

	for name, def := range doc.Tasks {
		if body, ok := def.(map[string]any); ok {
			defs[prefix+name] = body
		} else {
			defs[prefix+name] = map[string]any{"cmds": def}
		}
	}

	for key, spec := range doc.Includes {
		file, flatten := "", false

		switch include := spec.(type) {
		case string:
			file = include
		case map[string]any:
			file, _ = include["taskfile"].(string)
			flatten, _ = include["flatten"].(bool)
		}

		full := filepath.Join(filepath.Dir(path), file)
		if info, statErr := os.Stat(full); statErr == nil && info.IsDir() {
			full = filepath.Join(full, "Taskfile.yml")
		}

		childPrefix := prefix
		if !flatten {
			childPrefix = prefix + key + ":"
		}

		collectTasks(t, full, childPrefix, defs)
	}
}

// platformList reads a `platforms:` value as sorted GOOS names (the part
// before any /arch), nil when absent.
func platformList(value any) []string {
	list := asList(value)
	if list == nil {
		return nil
	}

	found := make(map[string]bool)
	for _, item := range list {
		if text, ok := item.(string); ok {
			goos, _, _ := strings.Cut(text, "/")
			found[goos] = true
		}
	}

	return slices.Sorted(maps.Keys(found))
}

// asList returns value as a list, nil when it is not one.
func asList(value any) []any {
	list, _ := value.([]any)

	return list
}

// isEcho reports whether a command only prints.
func isEcho(cmd string) bool {
	return strings.HasPrefix(strings.TrimSpace(cmd), "echo ")
}

// orAll renders an empty platform list as "all" for messages.
func orAll(platforms []string) any {
	if len(platforms) == 0 {
		return "all"
	}

	return platforms
}
