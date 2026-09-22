package vtcli

import (
	"os"
	"path/filepath"
	"regexp"
	"testing"

	"gopkg.in/yaml.v3"
)

// taskfileDoc is the part of a Taskfile this check reads: its global vars and
// the files it includes.
type taskfileDoc struct {
	Vars     map[string]any `yaml:"vars"`
	Includes map[string]any `yaml:"includes"`
}

// TestForwardedVarsAreOverridable is anti-drift check 4. Task lets a global var
// of an INCLUDED Taskfile beat a CLI `VAR=x` unless the var is written as its
// own default ('{{.VAR | default "x"}}'). vt forwards every flag as a CLI var,
// so a plain global there silently turns the flag into a no-op: --profile was
// ignored on every run until this test existed. The root Taskfile is exempt;
// its globals do yield to the CLI.
func TestForwardedVarsAreOverridable(t *testing.T) {
	t.Parallel()

	root, err := FindRepoRoot(".")
	if err != nil {
		t.Fatalf("find repo root: %v", err)
	}

	forwarded := make(map[string]string)
	for _, command := range CuratedSpec() {
		for _, flag := range command.Flags {
			if flag.Var != "" {
				forwarded[flag.Var] = command.Task
			}
		}

		for _, arg := range command.Args {
			if arg.Var != "" {
				forwarded[arg.Var] = command.Task
			}
		}
	}

	rootDoc := readTaskfile(t, filepath.Join(root, "Taskfile.yml"))
	for _, included := range includedTaskfiles(t, root, rootDoc) {
		doc := readTaskfile(t, included)

		for name, value := range doc.Vars {
			task, isForwarded := forwarded[name]
			if !isForwarded {
				continue
			}

			text, isString := value.(string)
			selfDefault := regexp.MustCompile(`\.` + regexp.QuoteMeta(name) + `\s*\|\s*default\b`)

			if !isString || !selfDefault.MatchString(text) {
				rel, _ := filepath.Rel(root, included)
				t.Errorf("%s: global var %s shadows the CLI, so vt's flag for it on %s does nothing; "+
					"write it as '{{.%s | default ...}}'", rel, name, task, name)
			}
		}
	}
}

// includedTaskfiles resolves a Taskfile's includes to file paths, recursively.
func includedTaskfiles(t *testing.T, dir string, doc taskfileDoc) []string {
	t.Helper()

	var files []string

	for _, spec := range doc.Includes {
		path := ""

		switch include := spec.(type) {
		case string:
			path = include
		case map[string]any:
			path, _ = include["taskfile"].(string)
		}

		if path == "" {
			continue
		}

		full := filepath.Join(dir, path)
		if info, err := os.Stat(full); err == nil && info.IsDir() {
			full = filepath.Join(full, "Taskfile.yml")
		}

		files = append(files, full)
		files = append(files, includedTaskfiles(t, filepath.Dir(full), readTaskfile(t, full))...)
	}

	return files
}

// readTaskfile parses the vars and includes of one Taskfile.
func readTaskfile(t *testing.T, path string) taskfileDoc {
	t.Helper()

	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatalf("read %s: %v", path, err)
	}

	var doc taskfileDoc
	if parseErr := yaml.Unmarshal(raw, &doc); parseErr != nil {
		t.Fatalf("parse %s: %v", path, parseErr)
	}

	return doc
}
