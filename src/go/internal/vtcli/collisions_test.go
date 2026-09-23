package vtcli

import (
	"os"
	"path/filepath"
	"testing"

	"gopkg.in/yaml.v3"
)

// taskDef is the part of a task this check reads.
type taskDef struct {
	Aliases []string `yaml:"aliases"`
}

// TestNoTaskNameCollisions is anti-drift check 7. Task's global namespace is
// flat: two included Taskfiles that both define `lint` collide, and the winner
// is decided by include order. Every include is flattened, so a task key -- and
// every alias, which is a global name too -- must be unique across the whole
// tree. The app Taskfiles carry fully-qualified keys and aliases
// (`hailo:lint`, `auto-annotator:lint`) for exactly this reason, which also
// keeps their reachable names unchanged. There is no allowlist: a duplicate is
// a real collision, and a new one must be renamed rather than hidden by an
// include's namespace.
func TestNoTaskNameCollisions(t *testing.T) {
	t.Parallel()

	root, err := FindRepoRoot(".")
	if err != nil {
		t.Fatalf("find repo root: %v", err)
	}

	origins := make(map[string][]string)
	collectOrigins(t, filepath.Join(root, "Taskfile.yml"), origins)

	for name, files := range origins {
		if len(files) > 1 {
			t.Errorf("name %q is defined in more than one included Taskfile (%v); "+
				"Task's namespace is flat and every include is flattened, so include "+
				"order decides which one runs. Give each definition a unique name", name, files)
		}
	}
}

// collectOrigins walks the Taskfile tree, recording the file every task key and
// alias is defined in.
func collectOrigins(t *testing.T, path string, origins map[string][]string) {
	t.Helper()

	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatalf("read %s: %v", path, err)
	}

	var doc struct {
		Tasks    map[string]taskDef `yaml:"tasks"`
		Includes map[string]any     `yaml:"includes"`
	}
	if parseErr := yaml.Unmarshal(raw, &doc); parseErr != nil {
		t.Fatalf("parse %s: %v", path, parseErr)
	}

	display, relErr := filepath.Rel(repoRootOr(t, path), path)
	if relErr != nil {
		display = path
	}

	for name, task := range doc.Tasks {
		origins[name] = append(origins[name], display)
		for _, alias := range task.Aliases {
			origins[alias] = append(origins[alias], display+" (alias of "+name+")")
		}
	}

	for _, spec := range doc.Includes {
		file := ""

		switch include := spec.(type) {
		case string:
			file = include
		case map[string]any:
			file, _ = include["taskfile"].(string)
		}

		if file == "" {
			continue
		}

		full := filepath.Join(filepath.Dir(path), file)
		if info, statErr := os.Stat(full); statErr == nil && info.IsDir() {
			full = filepath.Join(full, "Taskfile.yml")
		}

		collectOrigins(t, full, origins)
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
