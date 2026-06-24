package memory

import (
	"sync"
	"sync/atomic"

	telemetryv1 "github.com/teamvoldemor/voldemorbot/platform/backend/gen/telemetry/v1"
)

const (
	defaultHistorySize = 360
	subscriberChanBuf  = 16
)

type subscriber struct {
	ch chan *telemetryv1.RobotSnapshot
}

// Memory is a thread-safe ring-buffer store for robot snapshots and topic state.
// It is the single source of truth for both the gRPC ingest adaptor and the gin edge.
// Pointer fields are ordered first to minimize the GC-scanned span.
type Memory struct {
	latest   *telemetryv1.RobotSnapshot
	subs     map[uint64]*subscriber
	topics   *telemetryv1.TopicsSnapshot
	history  []*telemetryv1.RobotSnapshot
	mu       sync.RWMutex
	subMu    sync.RWMutex
	nextID   atomic.Uint64
	topicsMu sync.RWMutex
	cap      int
}

// NewMemory returns a Memory store with a ring-buffer capacity of historySize.
// A value ≤ 0 falls back to defaultHistorySize.
func NewMemory(historySize int) *Memory {
	if historySize <= 0 {
		historySize = defaultHistorySize
	}
	return &Memory{
		cap:  historySize,
		subs: make(map[uint64]*subscriber),
	}
}

// Write stores a snapshot and notifies all active WS subscribers.
func (m *Memory) Write(snap *telemetryv1.RobotSnapshot) {
	m.mu.Lock()
	m.latest = snap
	m.history = append(m.history, snap)
	if len(m.history) > m.cap {
		m.history = m.history[len(m.history)-m.cap:]
	}
	m.mu.Unlock()

	m.notify(snap)
}

// Latest returns the most recent snapshot, or nil if none received yet.
func (m *Memory) Latest() *telemetryv1.RobotSnapshot {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.latest
}

// History returns up to limit snapshots in chronological order.
func (m *Memory) History(limit int) []*telemetryv1.RobotSnapshot {
	m.mu.RLock()
	h := m.history
	if limit > 0 && limit < len(h) {
		h = h[len(h)-limit:]
	}
	out := make([]*telemetryv1.RobotSnapshot, len(h))
	copy(out, h)
	m.mu.RUnlock()
	return out
}

// Subscribe returns a channel that receives every new snapshot and a cancel func.
// Slow subscribers are never blocked — frames are dropped (channel buffer = 16).
func (m *Memory) Subscribe() (snapshots <-chan *telemetryv1.RobotSnapshot, cancel func()) {
	id := m.nextID.Add(1)
	sub := &subscriber{ch: make(chan *telemetryv1.RobotSnapshot, subscriberChanBuf)}

	m.subMu.Lock()
	m.subs[id] = sub
	m.subMu.Unlock()

	unsub := func() {
		m.subMu.Lock()
		delete(m.subs, id)
		m.subMu.Unlock()
	}
	return sub.ch, unsub
}

func (m *Memory) notify(snap *telemetryv1.RobotSnapshot) {
	m.subMu.RLock()
	defer m.subMu.RUnlock()
	for _, sub := range m.subs {
		select {
		case sub.ch <- snap:
		default:
		}
	}
}

// WriteTopics replaces the latest topics snapshot.
func (m *Memory) WriteTopics(topics *telemetryv1.TopicsSnapshot) {
	m.topicsMu.Lock()
	m.topics = topics
	m.topicsMu.Unlock()
}

// LatestTopics returns the most recent topics snapshot, or nil.
func (m *Memory) LatestTopics() *telemetryv1.TopicsSnapshot {
	m.topicsMu.RLock()
	defer m.topicsMu.RUnlock()
	return m.topics
}
