package handlers

import (
	"database/sql"
	"os"
)

// nullStr returns the string value of a nullable column, or "" when NULL,
// matching the Python `value or ""` convention for format/updated_at.
func nullStr(ns sql.NullString) string {
	if ns.Valid {
		return ns.String
	}
	return ""
}

// ensureDir creates dir (and parents) if it does not already exist.
func ensureDir(dir string) error {
	return os.MkdirAll(dir, 0o755)
}
