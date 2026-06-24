package telemetry

import telemetryv1 "github.com/teamvoldemor/voldemorbot/platform/backend/gen/telemetry/v1"

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

func (s *Service) Write(snap *telemetryv1.RobotSnapshot)          { s.store.Write(snap) }
func (s *Service) WriteTopics(topics *telemetryv1.TopicsSnapshot) { s.store.WriteTopics(topics) }
func (s *Service) Latest() *telemetryv1.RobotSnapshot             { return s.store.Latest() }
func (s *Service) History(limit int) []*telemetryv1.RobotSnapshot { return s.store.History(limit) }
func (s *Service) Subscribe() (<-chan *telemetryv1.RobotSnapshot, func()) {
	return s.store.Subscribe()
}
func (s *Service) LatestTopics() *telemetryv1.TopicsSnapshot { return s.store.LatestTopics() }
