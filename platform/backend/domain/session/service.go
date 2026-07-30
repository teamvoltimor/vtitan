package session

import (
	"context"

	telemetryv1 "github.com/teamvoltimor/vtitan/platform/backend/gen/telemetry/v1"
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

func (s *Service) Record(ctx context.Context, snap *telemetryv1.RobotSnapshot) error {
	return s.store.Record(ctx, snap)
}

func (s *Service) ListSessions(ctx context.Context) ([]SessionInfo, error) {
	return s.store.ListSessions(ctx)
}

func (s *Service) LoadSession(ctx context.Context, sessionID string) ([]*telemetryv1.RobotSnapshot, error) {
	return s.store.LoadSession(ctx, sessionID)
}
