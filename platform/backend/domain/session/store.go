package session

import (
	"context"

	telemetryv1 "github.com/teamvoldemor/voldemorbot/platform/backend/gen/telemetry/v1"
)

// SessionStore is the storage port for session recording.
type SessionStore interface {
	Record(ctx context.Context, snap *telemetryv1.RobotSnapshot) error
	ListSessions(ctx context.Context) ([]SessionInfo, error)
	LoadSession(ctx context.Context, sessionID string) ([]*telemetryv1.RobotSnapshot, error)
}
