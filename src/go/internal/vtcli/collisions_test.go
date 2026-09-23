package vtcli

import (
	"os"
	"path/filepath"
	"testing"

	"gopkg.in/yaml.v3"
)

// taskOrigin records where one task definition lives and how it is reached.
type taskOrigin struct {
	file string
	// namespace is the prefix Task gives the task in the flattened global
	// namespace: "" when the include is flattened, otherwise "<key>:".
	namespace string
}

// namespacedDuplicateTasks lists the raw task keys defined by more than one
// included Taskfile but landing in different namespaces, where Task keeps them
// apart and include order does not matter. Each is intentional -- the same
// verb under two apps, or an app's own task beside the repo-wide umbrella --
// and each would become a real collision the moment one of its files is
// flattened (which check 7 above would then catch, but only that way). Listing
// them here turns a new cross-file duplicate into an explicit decision instead
// of something include order hides.
var namespacedDuplicateTasks = map[string]string{
	"clean:all":        "hailo's own deep clean, beside platform.yml's repo-wide clean:all",
	"default":          "the root's task list, and each non-flattened app's own (auto-annotator, hailo, hugo-docs)",
	"docker:down":      "each app's own compose stack (auto-annotator) beside the repo-level one in platform.yml",
	"docker:logs":      "each app's own compose stack (auto-annotator, hailo) beside the repo-level one in platform.yml",
	"docker:up":        "each app's own compose stack (auto-annotator) beside the repo-level one in platform.yml",
	"frontend:build":   "auto-annotator's own frontend, beside the flattened frontend: domain",
	"frontend:dev":     "auto-annotator's own frontend, beside the flattened frontend: domain",
	"frontend:install": "auto-annotator's own frontend, beside the flattened frontend: domain",
	"frontend:preview": "auto-annotator's own frontend, beside the flattened frontend: domain",
	"lint":             "hailo's own ruff lint, beside the repo-wide umbrella in platform.yml",
	"lint:fix":         "hailo's own ruff fix, beside the repo-wide umbrella in platform.yml",
}

// TestNoTaskNameCollisions is anti-drift check 7. Task's global namespace is
// flat: two includes that both define `lint` collide, and the winner is
// decided by include order. Today that is invisible because the four
// colliding files (auto-annotator, hailo, models, hugo-docs) happen to be
// included without `flatten: true`, so their tasks land under `hailo:*` and
// friends. Flipping one `flatten:` to true would silently shadow
// other/tasks/platform.yml's `lint`, which CI runs -- and nothing would fail
// to warn about it.
//
// The check reports two names that resolve to the same name in the global
// namespace. Duplicates that stay namespaced apart are fine, so they must be
// listed in namespacedDuplicateTasks; a duplicate that collapses into one
// namespace is a real collision and fails. A stale allowlist entry fails too.
func TestNoTaskNameCollisions(t *testing.T) {
	t.Parallel()

	root, err := FindRepoRoot(".")
	if err != nil {
		t.Fatalf("find repo root: %v", err)
	}

	origins := make(map[string][]taskOrigin)
	collectOrigins(t, filepath.Join(root, "Taskfile.yml"), "", origins)

	for name, where := range origins {
		if len(where) < 2 {
			continue
		}

		// Two definitions of the same raw key are a real collision when they
		// land in the same namespace; otherwise Task keeps them apart.
		byNamespace := make(map[string][]string)
		for _, origin := range where {
			byNamespace[origin.namespace] = append(byNamespace[origin.namespace], origin.file)
		}

		collided := false
		for namespace, files := range byNamespace {
			if len(files) < 2 {
				continue
			}

			collided = true
			t.Errorf("task %q is defined %d times in namespace %q (%v); include order "+
				"decides the winner, so which one CI's `task %s` runs depends on the "+
				"includes block", name, len(files), namespace, files, name)
		}

		if collided {
			continue
		}

		if _, documented := namespacedDuplicateTasks[name]; !documented {
			t.Errorf("task %q is defined in more than one included Taskfile (%v); the "+
				"namespaces keep them apart today, but flattening any of those includes "+
				"would let include order decide the winner. Add it to "+
				"namespacedDuplicateTasks with a reason, or rename it", name, namesByFile(where))
		}
	}

	for name, reason := range namespacedDuplicateTasks {
		if len(origins[name]) < 2 {
			t.Errorf("namespacedDuplicateTasks lists %q (%s), but it is no longer defined "+
				"in more than one included Taskfile; remove the entry", name, reason)
		}
	}
}

// namesByFile renders the files a task is defined in, for failure messages.
func namesByFile(where []taskOrigin) []string {
	files := make([]string, 0, len(where))
	for _, origin := range where {
		files = append(files, origin.file)
	}

	return files
}

// collectOrigins walks the Taskfile tree, recording every task definition and
// the namespace prefix Task would give it.
func collectOrigins(t *testing.T, path, prefix string, origins map[string][]taskOrigin) {
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

	for name := range doc.Tasks {
		display, relErr := filepath.Rel(repoRootOr(t, path), path)
		if relErr != nil {
			display = path
		}

		origins[name] = append(origins[name], taskOrigin{file: display, namespace: prefix})
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

		collectOrigins(t, full, childPrefix, origins)
	}
}

// repoRootOr walks up from path to the checkout root, for display purposes.
func repoRootOr(t *testing.T, path string) string {
	t.Helper()

	root, err := FindRepoRoot(filepath.Dir(path))
	if err != nil {
		return filepath.Dir(path)
	}

	return root
}
