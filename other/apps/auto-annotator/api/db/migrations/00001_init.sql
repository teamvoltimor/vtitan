-- +goose Up
-- Schema for the auto-annotator manifest database.
--
-- Timestamp columns are declared TEXT (not TIMESTAMP) so the pure-Go SQLite
-- driver scans the ISO/`CURRENT_TIMESTAMP` string values cleanly. Columns that
-- are always populated are NOT NULL to keep the generated Go types non-nullable.
CREATE TABLE IF NOT EXISTS classes (
    id         INTEGER PRIMARY KEY AUTOINCREMENT,
    name       TEXT NOT NULL UNIQUE,
    color      TEXT NOT NULL DEFAULT '#dc322f'
        CHECK (color LIKE '#%' AND length(color) IN (4, 7, 9)),
    created_at TEXT DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE IF NOT EXISTS images (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    path        TEXT NOT NULL UNIQUE,
    status      INTEGER NOT NULL DEFAULT 0 CHECK (status IN (0, 1, 2)),
    format_used TEXT,
    parent_id   INTEGER REFERENCES images(id) ON DELETE CASCADE,
    created_at  TEXT DEFAULT CURRENT_TIMESTAMP,
    updated_at  TEXT,
    deleted_at  TEXT
);

CREATE INDEX IF NOT EXISTS ix_images_status    ON images(status);
CREATE INDEX IF NOT EXISTS ix_images_parent_id ON images(parent_id);

-- +goose Down
DROP INDEX IF EXISTS ix_images_parent_id;
DROP INDEX IF EXISTS ix_images_status;
DROP TABLE IF EXISTS images;
DROP TABLE IF EXISTS classes;
