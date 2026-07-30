// Package memory is an in-memory Store for the Vision bounded context's own
// state: annotations (real CRUD), active model selection, and pipeline
// status. There is no real model registry or inference pipeline wired into
// this Go backend yet, so ActiveModel/PipelineStatus reflect only what was
// set via this API, honestly, rather than fabricated inference metrics.
package memory

import (
	"context"
	"sync"
	"time"

	"github.com/google/uuid"

	"github.com/teamvoltimor/vtitan/platform/backend/domain/vision"
)

// Memory is a thread-safe in-memory Store for the Vision bounded context.
type Memory struct {
	mu          sync.RWMutex
	annotations map[string]vision.Annotation
	model       *vision.ModelInfo
}

// NewMemory returns an empty Memory store.
func NewMemory() *Memory {
	return &Memory{annotations: make(map[string]vision.Annotation)}
}

func (m *Memory) ListAnnotations(_ context.Context) ([]vision.Annotation, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	out := make([]vision.Annotation, 0, len(m.annotations))
	for _, a := range m.annotations {
		out = append(out, a)
	}
	return out, nil
}

func (m *Memory) CreateAnnotation(_ context.Context, req vision.CreateAnnotationRequest) (vision.Annotation, error) {
	a := vision.Annotation{
		ID:        uuid.NewString(),
		ClassName: req.ClassName,
		Bbox:      req.Bbox,
		ImageID:   req.ImageID,
		Metadata:  req.Metadata,
		CreatedAt: time.Now().UTC(),
	}
	m.mu.Lock()
	m.annotations[a.ID] = a
	m.mu.Unlock()
	return a, nil
}

func (m *Memory) UpdateAnnotation(_ context.Context, id string, req vision.UpdateAnnotationRequest) (vision.Annotation, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	a, ok := m.annotations[id]
	if !ok {
		return vision.Annotation{}, vision.ErrNotFound
	}
	if req.ClassName != nil {
		a.ClassName = *req.ClassName
	}
	if req.Bbox != nil {
		a.Bbox = *req.Bbox
	}
	now := time.Now().UTC()
	a.UpdatedAt = &now
	m.annotations[id] = a
	return a, nil
}

func (m *Memory) DeleteAnnotation(_ context.Context, id string) error {
	m.mu.Lock()
	defer m.mu.Unlock()
	if _, ok := m.annotations[id]; !ok {
		return vision.ErrNotFound
	}
	delete(m.annotations, id)
	return nil
}

func (m *Memory) ActiveModel(_ context.Context) (vision.ModelInfo, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	if m.model == nil {
		return vision.ModelInfo{}, nil
	}
	return *m.model, nil
}

func (m *Memory) SetActiveModel(_ context.Context, req vision.SetModelRequest) (vision.ModelInfo, error) {
	version := ""
	if req.Version != nil {
		version = *req.Version
	}
	info := vision.ModelInfo{Name: req.Name, Version: version}
	m.mu.Lock()
	m.model = &info
	m.mu.Unlock()
	return info, nil
}

// PipelineStatus reports honestly that no inference pipeline is wired into
// this Go backend yet, rather than fabricating fps/frame-count metrics.
func (m *Memory) PipelineStatus(_ context.Context) (vision.PipelineStatus, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	st := vision.PipelineStatus{Active: false, ModelLoaded: m.model != nil}
	if m.model != nil {
		st.ModelName = &m.model.Name
	}
	return st, nil
}
