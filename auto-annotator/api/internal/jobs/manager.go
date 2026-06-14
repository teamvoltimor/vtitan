// Package jobs provides a single-active-job manager with an SSE-friendly
// pub/sub broker. It replaces the Python asyncio JobManager: at most one job
// runs at a time (202-start / 409-busy), and subscribers receive progress
// events over buffered channels.
package jobs

import (
	"context"
	"crypto/rand"
	"encoding/hex"
	"errors"
	"sync"
)

var (
	// ErrBusy is returned by Start when a job is already running.
	ErrBusy = errors.New("a job is already running")
)

type (
	// Event is one progress message broadcast to subscribers. The JSON shape mirrors
	// the Python JobEvent.to_dict (job_id, status, message, data).
	Event struct {
		Data    map[string]any `json:"data"`
		JobID   string         `json:"job_id"`
		Status  string         `json:"status"`
		Message string         `json:"message"`
	}

	// EmitFunc is passed to a job runner to broadcast progress.
	EmitFunc func(status JobStatus, message string, data map[string]any)

	// Manager owns the single active job slot.
	Manager struct {
		current *job
		mu      sync.Mutex
	}
)

// New creates an empty Manager.
func New() *Manager { return &Manager{} }

// Running reports whether a job is currently active.
func (m *Manager) Running() bool {
	m.mu.Lock()
	defer m.mu.Unlock()
	return m.current != nil
}

// Start runs fn as the single active job in a new goroutine. fn must emit a
// terminal (completed/failed) event itself; the manager broadcasts every emit
// and clears the slot once fn returns. Returns ErrBusy if a job is active.
func (m *Manager) Start(run func(ctx context.Context, emit EmitFunc)) (string, error) {
	m.mu.Lock()
	if m.current != nil {
		m.mu.Unlock()
		return "", ErrBusy
	}
	j := &job{id: newID()}
	m.current = j
	m.mu.Unlock()

	emit := func(status JobStatus, message string, data map[string]any) {
		j.broadcast(Event{JobID: j.id, Status: string(status), Message: message, Data: data})
	}

	go func() {
		defer func() {
			j.close()
			m.mu.Lock()
			if m.current == j {
				m.current = nil
			}
			m.mu.Unlock()
		}()
		run(context.Background(), emit)
	}()
	return j.id, nil
}

// Subscribe returns a channel of events for the active job, plus its id. ok is
// false when no job is running. The channel is closed when the job finishes.
func (m *Manager) Subscribe() (jobID string, ch <-chan Event, ok bool) {
	m.mu.Lock()
	j := m.current
	m.mu.Unlock()
	if j == nil {
		return "", nil, false
	}
	return j.id, j.addSub(), true
}

type job struct {
	id   string
	subs []chan Event
	mu   sync.Mutex
	done bool
}

func (j *job) addSub() chan Event {
	j.mu.Lock()
	defer j.mu.Unlock()
	ch := make(chan Event, 32)
	if j.done {
		close(ch)
		return ch
	}
	j.subs = append(j.subs, ch)
	return ch
}

func (j *job) broadcast(e Event) {
	j.mu.Lock()
	defer j.mu.Unlock()
	for _, ch := range j.subs {
		select {
		case ch <- e:
		default: // drop when a slow subscriber's buffer is full
		}
	}
}

func (j *job) close() {
	j.mu.Lock()
	defer j.mu.Unlock()
	if j.done {
		return
	}
	j.done = true
	for _, ch := range j.subs {
		close(ch)
	}
	j.subs = nil
}

func newID() string {
	var b [16]byte
	if _, err := rand.Read(b[:]); err != nil {
		return "job"
	}
	return hex.EncodeToString(b[:])
}
