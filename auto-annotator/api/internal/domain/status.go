// Package domain holds core value types and mappings shared across the API,
// independent of HTTP or storage concerns.
package domain

import "strconv"

// Image annotation status codes, stored as integers in SQLite.
const (
	StatusPending int64 = 0
	StatusDone    int64 = 1
	StatusSkipped int64 = 2
)

// Human-readable status labels, matching the Python STATUS_NAMES map and the
// strings the frontend expects.
const (
	statusNamePending = "pending"
	statusNameDone    = "done"
	statusNameSkipped = "skipped"
)

// StatusName maps an integer status code to its human-readable label, falling
// back to the numeric form for unknown codes (mirrors the Python behaviour).
func StatusName(status int64) string {
	switch status {
	case StatusPending:
		return statusNamePending
	case StatusDone:
		return statusNameDone
	case StatusSkipped:
		return statusNameSkipped
	default:
		return strconv.FormatInt(status, 10)
	}
}
