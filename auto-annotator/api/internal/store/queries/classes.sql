-- name: CountClasses :one
SELECT COUNT(*) FROM classes;

-- name: ListClasses :many
SELECT id, name, color FROM classes ORDER BY id ASC;

-- name: UpsertClass :exec
INSERT INTO classes (name, color) VALUES (?, ?)
ON CONFLICT(name) DO UPDATE SET color = excluded.color;

-- name: InsertClassOrIgnore :exec
INSERT OR IGNORE INTO classes (name, color) VALUES (?, ?);
