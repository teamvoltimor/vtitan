// Package session is the session recording bounded context. It owns the
// lifecycle of recorded telemetry sessions: creating, persisting frames,
// listing, and loading.
package session

import (
	"errors"
	"time"
)

// ErrSessionNotFound is returned when a session ID does not exist in the index.
var ErrSessionNotFound = errors.New("session not found")

// SessionInfo is a summary of a recorded session.
type SessionInfo struct {
	CreatedAt  time.Time
	SessionID  string
	EntryCount int
}
