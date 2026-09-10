package outbox_test

import (
	"context"
	"errors"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/statemachine/outbox"
)

// fakeStreamer records every item Send receives, optionally failing on a
// configured item.
type fakeStreamer struct {
	failOn  int
	failErr error
	sent    []int
}

func (f *fakeStreamer) Send(_ context.Context, item int) error {
	if item == f.failOn && f.failErr != nil {
		return f.failErr
	}
	f.sent = append(f.sent, item)
	return nil
}

func TestSlot_NextReturnsPushedItem(t *testing.T) {
	t.Parallel()

	slot := outbox.NewSlot[int]()
	slot.Push(42)

	ctx, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()

	item, ok := slot.Next(ctx)
	if !ok {
		t.Fatal("Next() ok = false, want true")
	}
	if item != 42 {
		t.Fatalf("Next() item = %d, want 42", item)
	}
}

func TestSlot_PushKeepsOnlyTheLatest(t *testing.T) {
	t.Parallel()

	slot := outbox.NewSlot[int]()
	slot.Push(1)
	slot.Push(2)
	slot.Push(3)

	ctx, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()

	item, ok := slot.Next(ctx)
	if !ok {
		t.Fatal("Next() ok = false, want true")
	}
	if item != 3 {
		t.Fatalf("Next() item = %d, want 3 (the latest push)", item)
	}
}

func TestSlot_PushNeverBlocks(t *testing.T) {
	t.Parallel()

	slot := outbox.NewSlot[int]()

	done := make(chan struct{})
	go func() {
		defer close(done)
		for i := range 1000 {
			slot.Push(i)
		}
	}()

	select {
	case <-done:
	case <-time.After(5 * time.Second):
		t.Fatal("1000 pushes with no consumer took over 5s -- Push is blocking")
	}
}

func TestSlot_NextReturnsFalseWhenContextDone(t *testing.T) {
	t.Parallel()

	slot := outbox.NewSlot[int]()

	ctx, cancel := context.WithCancel(context.Background())
	cancel()

	_, ok := slot.Next(ctx)
	if ok {
		t.Fatal("Next() ok = true, want false after ctx was already canceled")
	}
}

func TestDrain_ForwardsPushedItemsInOrder(t *testing.T) {
	t.Parallel()

	slot := outbox.NewSlot[int]()
	streamer := &fakeStreamer{}

	ctx, cancel := context.WithCancel(context.Background())
	drainErr := make(chan error, 1)
	go func() {
		drainErr <- outbox.Drain(ctx, slot, streamer)
	}()

	// Push one at a time with a short pause so each is individually
	// collected by Drain rather than coalesced by Slot's keep-latest
	// behavior (already covered by TestSlot_PushKeepsOnlyTheLatest).
	for _, v := range []int{1, 2, 3} {
		slot.Push(v)
		time.Sleep(20 * time.Millisecond)
	}

	cancel()
	if err := <-drainErr; !errors.Is(err, context.Canceled) {
		t.Fatalf("Drain() error = %v, want context.Canceled", err)
	}

	if len(streamer.sent) != 3 || streamer.sent[0] != 1 || streamer.sent[1] != 2 || streamer.sent[2] != 3 {
		t.Fatalf("streamer.sent = %v, want [1 2 3]", streamer.sent)
	}
}

func TestDrain_ReturnsSendErrorImmediately(t *testing.T) {
	t.Parallel()

	slot := outbox.NewSlot[int]()
	wantErr := errors.New("stream closed")
	streamer := &fakeStreamer{failOn: 1, failErr: wantErr}

	slot.Push(1)

	err := outbox.Drain(context.Background(), slot, streamer)
	if !errors.Is(err, wantErr) {
		t.Fatalf("Drain() error = %v, want %v", err, wantErr)
	}
}
