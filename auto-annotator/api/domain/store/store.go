// Package store owns the SQLite connection, applies the schema, and exposes the
// sqlc-generated query set and configuration pragmas.
package store

import (
	"context"
	"database/sql"
	_ "embed"
	"fmt"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/db"

	_ "modernc.org/sqlite" // pure-Go SQLite driver, registered as "sqlite"
)

//go:embed schema.sql
var schemaSQL string

// defaultClasses are seeded on first run when the classes table is empty,
// matching the Python DEFAULT_CLASSES (WRO 2026 traffic-sign palette).
var defaultClasses = []db.InsertClassOrIgnoreParams{
	{Name: "red_prism", Color: "#ee2737"},
	{Name: "green_prism", Color: "#44d62c"},
	{Name: "magenta_prism", Color: "#ff00ff"},
}

// Store bundles the database handle and the generated query set.
type Store struct {
	DB *sql.DB
	Q  *db.Queries
}

// Open connects to the SQLite database at dbPath (WAL mode, foreign keys on,
// busy timeout), applies the schema idempotently, and seeds default classes.
func Open(dbPath string) (*Store, error) {
	dsn := fmt.Sprintf(
		"file:%s?_pragma=%s&_pragma=%s&_pragma=%s",
		dbPath, SQLiteJournalMode, SQLiteForeignKeys, SQLiteBusyTimeout,
	)
	sqlDB, err := sql.Open("sqlite", dsn)
	if err != nil {
		return nil, fmt.Errorf("open db: %w", err)
	}
	if err := sqlDB.PingContext(context.Background()); err != nil {
		return nil, fmt.Errorf("ping db: %w", err)
	}
	if _, err := sqlDB.ExecContext(context.Background(), schemaSQL); err != nil {
		return nil, fmt.Errorf("apply schema: %w", err)
	}

	s := &Store{DB: sqlDB, Q: db.New(sqlDB)}
	if err := s.seedDefaultClasses(context.Background()); err != nil {
		return nil, err
	}
	return s, nil
}

// Close releases the underlying database handle.
func (s *Store) Close() error { return s.DB.Close() }

func (s *Store) seedDefaultClasses(ctx context.Context) error {
	n, err := s.Q.CountClasses(ctx)
	if err != nil {
		return fmt.Errorf("count classes: %w", err)
	}
	if n > 0 {
		return nil
	}
	for _, c := range defaultClasses {
		if err := s.Q.InsertClassOrIgnore(ctx, c); err != nil {
			return fmt.Errorf("seed class %q: %w", c.Name, err)
		}
	}
	return nil
}
