package telemetry

import (
	"context"
	"sync"

	"github.com/teamvoltimor/vtitan/src/go/internal/telemetry/diag"

	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
)

// natsSource implements diag.Source by caching the latest message received
// on each subscription -- the Go analog of telemetry_bridge_node.py's
// `_latest_scan`/`_latest_imu` instance caches, updated by each
// subscription's own read loop (watchLoop) rather than blocking Summarize
// on a topic.
type natsSource struct {
	mu   sync.RWMutex
	scan *sensorv1.Scan
	imu  *sensorv1.Imu
}

var _ diag.Source = (*natsSource)(nil)

// LatestScan implements diag.Source.
func (s *natsSource) LatestScan(_ context.Context) (*sensorv1.Scan, bool) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.scan, s.scan != nil
}

// LatestIMU implements diag.Source.
func (s *natsSource) LatestIMU(_ context.Context) (*sensorv1.Imu, bool) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.imu, s.imu != nil
}

// LatestDetections implements diag.Source. Always reports no detection --
// there is no vision detection wire schema yet (see diag.Detection's doc
// comment: vision stays Python-only and hasn't been given a proto), and
// Aggregator's own documented behavior for an input that has never arrived
// is to omit it from the summary, so this is the correct permanent answer
// here, not a placeholder to fill in later.
func (s *natsSource) LatestDetections(_ context.Context) ([]diag.Detection, bool) {
	return nil, false
}

func (s *natsSource) setScan(scan *sensorv1.Scan) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.scan = scan
}

func (s *natsSource) setIMU(imu *sensorv1.Imu) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.imu = imu
}
