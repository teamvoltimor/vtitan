// Package domain holds core value types and mappings shared across the API,
// independent of HTTP or storage concerns.
package domain

import "strconv"

const (
	// StatusPending is the integer status code for pending annotations.
	StatusPending int64 = 0
	// StatusDone is the integer status code for completed annotations.
	StatusDone int64 = 1
	// StatusSkipped is the integer status code for skipped annotations.
	StatusSkipped int64 = 2

	// statusNamePending is the human-readable label for pending status.
	statusNamePending = "pending"
	// statusNameDone is the human-readable label for done status.
	statusNameDone = "done"
	// statusNameSkipped is the human-readable label for skipped status.
	statusNameSkipped = "skipped"
)

// StatusName maps an integer status code to its human-readable label, falling
// back to the numeric form for unknown codes (mirrors the Python behavior).
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
