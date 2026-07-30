package telemetry

import telemetryv1 "github.com/teamvoltimor/vtitan/platform/backend/gen/telemetry/v1"

// SnapshotStore is the storage port for robot telemetry frames.
type SnapshotStore interface {
	Write(snap *telemetryv1.RobotSnapshot)
	WriteTopics(topics *telemetryv1.TopicsSnapshot)
	Latest() *telemetryv1.RobotSnapshot
	History(limit int) []*telemetryv1.RobotSnapshot
	Subscribe() (snapshots <-chan *telemetryv1.RobotSnapshot, cancel func())
	LatestTopics() *telemetryv1.TopicsSnapshot
}
