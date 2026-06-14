package jobs

import (
	"context"
	"errors"
	"testing"
	"time"
)

func waitFor(t *testing.T, cond func() bool) {
	t.Helper()
	deadline := time.Now().Add(2 * time.Second)
	for time.Now().Before(deadline) {
		if cond() {
			return
		}
		time.Sleep(5 * time.Millisecond)
	}
	t.Fatal("condition not met within timeout")
}

func TestSingleJobLock(t *testing.T) {
	m := New()
	release := make(chan struct{})

	id, err := m.Start(func(_ context.Context, _ EmitFunc) { <-release })
	if err != nil || id == "" {
		t.Fatalf("first Start failed: id=%q err=%v", id, err)
	}
	if !m.Running() {
		t.Fatal("expected Running after Start")
	}

	if _, err := m.Start(func(context.Context, EmitFunc) {}); !errors.Is(err, ErrBusy) {
		t.Fatalf("expected ErrBusy while a job runs, got %v", err)
	}

	close(release)
	waitFor(t, func() bool { return !m.Running() })

	if _, err := m.Start(func(context.Context, EmitFunc) {}); err != nil {
		t.Fatalf("expected Start to succeed after completion, got %v", err)
	}
}

func TestSubscribeReceivesEventsThenCloses(t *testing.T) {
	m := New()
	release := make(chan struct{})

	if _, err := m.Start(func(_ context.Context, emit EmitFunc) {
		<-release
		emit(StatusRunning, "tick", map[string]any{"progress": 0.5})
		emit(StatusCompleted, "done", map[string]any{"finished": true})
	}); err != nil {
		t.Fatalf("Start: %v", err)
	}

	_, ch, ok := m.Subscribe()
	if !ok {
		t.Fatal("expected an active job to subscribe to")
	}
	close(release)

	var got []Event
	for e := range ch { // channel is closed once the job finishes
		got = append(got, e)
	}
	if len(got) != 2 {
		t.Fatalf("expected 2 events, got %d: %+v", len(got), got)
	}
	if got[0].Status != string(StatusRunning) || got[1].Status != string(StatusCompleted) {
		t.Fatalf("unexpected event sequence: %+v", got)
	}
}

func TestSubscribeWhenIdle(t *testing.T) {
	m := New()
	if _, _, ok := m.Subscribe(); ok {
		t.Fatal("expected Subscribe to report no active job")
	}
}
