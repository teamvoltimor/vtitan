package db

import (
	"database/sql"
	"embed"
	"fmt"

	"github.com/pressly/goose/v3"
)

//go:embed migrations/*.sql
var migrationsFS embed.FS

// Migrate applies every pending migration under migrations/ to sqlDB, in
// order, tracking applied versions in a goose_db_version table. Each
// migration is idempotent (CREATE TABLE/INDEX IF NOT EXISTS) so re-running
// migration 00001 against a database that already has these tables — e.g.
// one created before migrations existed — is a safe no-op.
func Migrate(sqlDB *sql.DB) error {
	goose.SetBaseFS(migrationsFS)
	defer goose.SetBaseFS(nil)

	if err := goose.SetDialect("sqlite3"); err != nil {
		return fmt.Errorf("set goose dialect: %w", err)
	}
	if err := goose.Up(sqlDB, "migrations"); err != nil {
		return fmt.Errorf("apply migrations: %w", err)
	}
	return nil
}
