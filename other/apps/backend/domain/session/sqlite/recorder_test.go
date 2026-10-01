package sqlite

import (
	"path/filepath"
	"testing"
)

// TestNewAppliesPragmas guards the DSN spelling: modernc.org/sqlite silently
// ignores the mattn-style _journal_mode= and _busy_timeout= options, which
// would leave the index on the default rollback journal with no busy wait.
func TestNewAppliesPragmas(t *testing.T) {
	t.Parallel()

	dir := t.TempDir()
	r, err := New(t.Context(), filepath.Join(dir, "sessions.db"), filepath.Join(dir, "frames"), 0)
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	t.Cleanup(func() { _ = r.Close() })

	var journal string
	if err := r.sqlDB.QueryRowContext(t.Context(), "PRAGMA journal_mode").Scan(&journal); err != nil {
		t.Fatalf("read journal_mode: %v", err)
	}
	if journal != "wal" {
		t.Errorf("journal_mode = %q, want wal", journal)
	}

	var busy int
	if err := r.sqlDB.QueryRowContext(t.Context(), "PRAGMA busy_timeout").Scan(&busy); err != nil {
		t.Fatalf("read busy_timeout: %v", err)
	}
	if busy != 5000 {
		t.Errorf("busy_timeout = %d, want 5000", busy)
	}
}
