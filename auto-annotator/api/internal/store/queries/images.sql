-- name: ListImagesForBrowse :many
SELECT id, path, status, format_used, updated_at
FROM images
WHERE deleted_at IS NULL
ORDER BY id ASC;

-- name: GetImageByID :one
SELECT id, path, status, format_used, updated_at, parent_id
FROM images
WHERE id = ? AND deleted_at IS NULL;

-- name: GetStatusCounts :many
SELECT status, COUNT(*) AS cnt
FROM images
WHERE deleted_at IS NULL
GROUP BY status;

-- name: SoftDeleteImage :exec
UPDATE images SET deleted_at = ? WHERE id = ?;

-- name: MarkImageDone :exec
UPDATE images SET status = ?, format_used = ?, updated_at = ? WHERE id = ?;

-- name: MarkImageSkipped :exec
UPDATE images SET status = ?, updated_at = ? WHERE id = ?;

-- name: InsertImageOrIgnore :exec
INSERT OR IGNORE INTO images (path) VALUES (?);

-- name: InsertAugmentedImage :exec
INSERT OR IGNORE INTO images (path, status, format_used, parent_id, updated_at)
VALUES (?, ?, ?, ?, ?);

-- name: ListGroupedImages :many
SELECT i.id, i.path, i.status, i.format_used, i.updated_at,
       COUNT(c.id) AS aug_count
FROM images i
LEFT JOIN images c ON c.parent_id = i.id AND c.deleted_at IS NULL
WHERE i.parent_id IS NULL AND i.deleted_at IS NULL
GROUP BY i.id
ORDER BY i.id ASC;
