package telemetry

import telemetryv1 "github.com/teamvoltimor/vtitan/apps/backend/gen/telemetry/v1"

// TelemetryService is the telemetry management port.
type TelemetryService interface {
	Write(snap *telemetryv1.RobotSnapshot)
	WriteTopics(topics *telemetryv1.TopicsSnapshot)
	Latest() *telemetryv1.RobotSnapshot
	History(limit int) []*telemetryv1.RobotSnapshot
	Subscribe() (snapshots <-chan *telemetryv1.RobotSnapshot, cancel func())
	LatestTopics() *telemetryv1.TopicsSnapshot
}

// Service is the concrete implementation of TelemetryService.
type Service struct {
	store SnapshotStore
}

// NewService constructs a Service backed by the given SnapshotStore.
func NewService(store SnapshotStore) *Service {
	return &Service{store: store}
}

// Write stores snap as the latest robot snapshot.
func (s *Service) Write(snap *telemetryv1.RobotSnapshot) { s.store.Write(snap) }

// WriteTopics stores topics as the latest topic snapshot.
func (s *Service) WriteTopics(topics *telemetryv1.TopicsSnapshot) { s.store.WriteTopics(topics) }

// Latest returns the most recently written robot snapshot, or nil if none has arrived yet.
func (s *Service) Latest() *telemetryv1.RobotSnapshot { return s.store.Latest() }

// History returns up to limit of the most recently written robot snapshots.
func (s *Service) History(limit int) []*telemetryv1.RobotSnapshot { return s.store.History(limit) }

// Subscribe returns a channel of future robot snapshots and a cancel func to stop receiving.
func (s *Service) Subscribe() (<-chan *telemetryv1.RobotSnapshot, func()) {
	return s.store.Subscribe()
}

// LatestTopics returns the most recently written topic snapshot, or nil if none has arrived yet.
func (s *Service) LatestTopics() *telemetryv1.TopicsSnapshot { return s.store.LatestTopics() }
