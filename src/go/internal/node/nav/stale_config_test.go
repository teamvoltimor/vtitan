package nav

import (
	"log/slog"
	"os"
	"path/filepath"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/internal/adapters/natsgw"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
)

// writeSensorTOML writes a sensor.toml under a fresh config root.
func writeSensorTOML(t *testing.T, body string) string {
	t.Helper()
	root := t.TempDir()
	path := filepath.Join(root, profile.DefaultSensorTOMLPath)
	if err := os.MkdirAll(filepath.Dir(path), 0o750); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(path, []byte(body), 0o600); err != nil {
		t.Fatal(err)
	}
	return root
}

// The file must actually be read. The shipped value equals the Go default,
// so a pin against the shipped file alone would pass with a loader that
// read nothing; a root holding a different value is what proves the path.
func TestLoadStaleTimeout_ReadsTheFile(t *testing.T) {
	t.Parallel()

	logger := slog.New(slog.DiscardHandler)
	root := writeSensorTOML(t, "stale_timeout_sec = 0.25\n")
	if got, want := loadStaleTimeout(logger, root), 250*time.Millisecond; got != want {
		t.Errorf("loadStaleTimeout(0.25 s file) = %v, want %v", got, want)
	}
}

// Pin against the checked-in file: Python reads the same key, so a change
// to it moves both stacks together only if Go keeps reading it.
func TestLoadStaleTimeout_ShippedValue(t *testing.T) {
	t.Parallel()

	logger := slog.New(slog.DiscardHandler)
	repoRoot := filepath.Join("..", "..", "..", "..", "..")
	if got, want := loadStaleTimeout(logger, repoRoot), 500*time.Millisecond; got != want {
		t.Errorf("loadStaleTimeout(repo) = %v, want the shipped sensor.toml's %v", got, want)
	}
}

func TestLoadStaleTimeout_FallsBack(t *testing.T) {
	t.Parallel()

	logger := slog.New(slog.DiscardHandler)
	cases := map[string]string{
		"no config root": "",
		"missing file":   t.TempDir(),
		"zero value":     writeSensorTOML(t, "stale_timeout_sec = 0.0\n"),
		"negative value": writeSensorTOML(t, "stale_timeout_sec = -1.0\n"),
	}
	for name, root := range cases {
		if got := loadStaleTimeout(logger, root); got != natsgw.DefaultStaleTimeout {
			t.Errorf("%s: loadStaleTimeout = %v, want the default %v", name, got, natsgw.DefaultStaleTimeout)
		}
	}
}
