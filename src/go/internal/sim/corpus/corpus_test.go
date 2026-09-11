package corpus_test

import (
	"errors"
	"os"
	"path/filepath"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/sim/corpus"
)

func TestLoad_Directory(t *testing.T) {
	t.Parallel()

	dir := t.TempDir()
	names := []string{
		"scenario_0002_metadata.json",
		"scenario_0000_metadata.json",
		"scenario_0001_metadata.json",
		"scenario_0000.sdf", // non-matching sibling, must be ignored
	}
	for _, name := range names {
		writeEmptyFile(t, filepath.Join(dir, name))
	}

	scenarios, err := corpus.Load(dir)
	if err != nil {
		t.Fatalf("Load() error = %v, want nil", err)
	}

	wantIDs := []string{"scenario_0000", "scenario_0001", "scenario_0002"}
	if len(scenarios) != len(wantIDs) {
		t.Fatalf("Load() returned %d scenarios, want %d", len(scenarios), len(wantIDs))
	}
	for i, want := range wantIDs {
		if scenarios[i].ID != want {
			t.Errorf("scenarios[%d].ID = %q, want %q", i, scenarios[i].ID, want)
		}
	}
}

func TestLoad_SingleFile(t *testing.T) {
	t.Parallel()

	dir := t.TempDir()
	path := filepath.Join(dir, "custom_metadata.json")
	writeEmptyFile(t, path)

	scenarios, err := corpus.Load(path)
	if err != nil {
		t.Fatalf("Load() error = %v, want nil", err)
	}
	if len(scenarios) != 1 {
		t.Fatalf("Load() returned %d scenarios, want 1", len(scenarios))
	}
	if scenarios[0].MetadataPath != path {
		t.Errorf("scenarios[0].MetadataPath = %q, want %q", scenarios[0].MetadataPath, path)
	}
	if scenarios[0].ID != "custom" {
		t.Errorf("scenarios[0].ID = %q, want %q", scenarios[0].ID, "custom")
	}
}

func TestLoad_EmptyDirectory(t *testing.T) {
	t.Parallel()

	dir := t.TempDir()
	if _, err := corpus.Load(dir); !errors.Is(err, corpus.ErrNoScenarios) {
		t.Fatalf("Load() error = %v, want ErrNoScenarios", err)
	}
}

func TestLoad_MissingPath(t *testing.T) {
	t.Parallel()

	if _, err := corpus.Load(filepath.Join(t.TempDir(), "does-not-exist")); err == nil {
		t.Fatal("Load() error = nil, want a stat error")
	}
}

func writeEmptyFile(t *testing.T, path string) {
	t.Helper()
	if err := os.WriteFile(path, []byte("{}"), 0o600); err != nil {
		t.Fatalf("writing fixture %s: %v", path, err)
	}
}
