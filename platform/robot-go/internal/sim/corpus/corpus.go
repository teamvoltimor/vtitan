package corpus

import (
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"
)

// Scenario identifies one scenario metadata file to run. ID is derived from
// the filename (without the "_metadata.json" suffix) so it reads the same
// way the Python sweep tooling's own scenario labels do, e.g.
// "scenario_0000".
type Scenario struct {
	ID           string
	MetadataPath string
}

// metadataGlob is the filename pattern every scenario metadata file matches,
// mirroring _load_fixture_scenarios' own "*_metadata.json" glob in
// platform/robot/src/simulation/scenario_catalog.py.
const metadataGlob = "*_metadata.json"

// ErrNoScenarios is returned by Load when a directory contains no files
// matching metadataGlob, or a single-file path doesn't match it.
var ErrNoScenarios = errors.New("corpus: no scenario metadata files found")

// Load discovers scenarios at path. If path is a directory, every file
// matching metadataGlob directly inside it is loaded, sorted by filename —
// the same order Python's sorted(root.glob("*_metadata.json")) produces, so
// a Go run and a Python run against the same directory process scenarios in
// the same order. If path is a single file, it is loaded as the one
// scenario in the corpus regardless of its name.
func Load(path string) ([]Scenario, error) {
	info, err := os.Stat(path)
	if err != nil {
		return nil, fmt.Errorf("corpus: stat %s: %w", path, err)
	}

	if !info.IsDir() {
		return []Scenario{scenarioFromPath(path)}, nil
	}

	matches, err := filepath.Glob(filepath.Join(path, metadataGlob))
	if err != nil {
		return nil, fmt.Errorf("corpus: globbing %s: %w", path, err)
	}
	if len(matches) == 0 {
		return nil, fmt.Errorf("%w: %s", ErrNoScenarios, path)
	}
	sort.Strings(matches)

	scenarios := make([]Scenario, 0, len(matches))
	for _, match := range matches {
		scenarios = append(scenarios, scenarioFromPath(match))
	}
	return scenarios, nil
}

// scenarioFromPath derives a Scenario's ID from its metadata filename,
// stripping the "_metadata.json" suffix when present so the ID matches the
// scenario_id-derived label the Python tooling already uses (e.g.
// "scenario_0000_metadata.json" -> "scenario_0000"). Falls back to the bare
// filename for a corpus file that doesn't follow the convention, since a
// single explicitly-named file is still a valid one-scenario corpus.
func scenarioFromPath(path string) Scenario {
	base := filepath.Base(path)
	const suffix = "_metadata.json"
	id := strings.TrimSuffix(base, suffix)
	return Scenario{ID: id, MetadataPath: path}
}
