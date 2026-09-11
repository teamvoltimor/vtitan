package session

import (
	"context"

	telemetryv1 "github.com/teamvoltimor/vtitan/apps/backend/gen/telemetry/v1"
)

// SessionService is the session management port.
type SessionService interface {
	Record(ctx context.Context, snap *telemetryv1.RobotSnapshot) error
	ListSessions(ctx context.Context) ([]SessionInfo, error)
	LoadSession(ctx context.Context, sessionID string) ([]*telemetryv1.RobotSnapshot, error)
}

// Service is the concrete implementation of SessionService.
type Service struct {
	store SessionStore
}

// NewService constructs a Service backed by the given SessionStore.
func NewService(store SessionStore) *Service {
	return &Service{store: store}
}

// Record appends snap to the active session.
func (s *Service) Record(ctx context.Context, snap *telemetryv1.RobotSnapshot) error {
	return s.store.Record(ctx, snap)
}

// ListSessions returns summaries of every recorded session.
func (s *Service) ListSessions(ctx context.Context) ([]SessionInfo, error) {
	return s.store.ListSessions(ctx)
}

// LoadSession returns the full recorded snapshot sequence for sessionID.
func (s *Service) LoadSession(ctx context.Context, sessionID string) ([]*telemetryv1.RobotSnapshot, error) {
	return s.store.LoadSession(ctx, sessionID)
}
