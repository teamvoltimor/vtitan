-- name: InsertSession :exec
INSERT INTO sessions (session_id, created_at, entry_count)
VALUES (?, ?, 0);

-- name: IncrementEntryCount :exec
UPDATE sessions SET entry_count = entry_count + 1 WHERE session_id = ?;

-- name: ListSessions :many
SELECT session_id, created_at, entry_count
FROM sessions
ORDER BY created_at DESC;

-- name: GetSession :one
SELECT session_id, created_at, entry_count
FROM sessions
WHERE session_id = ?;

-- name: CountSessions :one
SELECT COUNT(*) FROM sessions;

-- name: DeleteSession :exec
DELETE FROM sessions WHERE session_id = ?;
