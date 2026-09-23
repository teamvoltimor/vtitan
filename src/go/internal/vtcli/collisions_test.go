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
// namespace. Duplicates that stay namespaced apart are fine and are the
// intended state, so the check is about the namespace each definition lands
// in, not about the raw task key.
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

		// Two definitions of the same raw key are only a real collision when
		// they land in the same namespace; otherwise Task keeps them apart.
		namespaces := make(map[string]bool)
		for _, origin := range where {
			namespaces[origin.namespace] = true
		}

		// Group by namespace, and report only a namespace holding 2+, so a
		// pre-existing namespaced duplicate is not dragged into the message.
		byNamespace := make(map[string][]string)
		for _, origin := range where {
			byNamespace[origin.namespace] = append(byNamespace[origin.namespace], origin.file)
		}

		for namespace, files := range byNamespace {
			if len(files) < 2 {
				continue
			}

			t.Errorf("task %q is defined %d times in namespace %q (%v); include order "+
				"decides the winner, so which one CI's `task %s` runs depends on the "+
				"includes block", name, len(files), namespace, files, name)
		}
	}
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
