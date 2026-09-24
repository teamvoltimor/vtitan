package boardsim

import (
	"context"
	"errors"
	"net"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
)

// A slow wheel still advances: the sub-count remainder carries over
// between steps instead of being truncated away each time.
func TestEncoder_AdvanceCarriesTheFraction(t *testing.T) {
	t.Parallel()

	var e Encoder
	for range 10 {
		e.advance(100, 0.001) // 0.1 count per step
	}
	if got := e.Counts(); got != 1 {
		t.Fatalf("Counts after 10 x 0.1 = %d, want 1", got)
	}
	for range 10 {
		e.advance(-100, 0.001)
	}
	if got := e.Counts(); got != 0 {
		t.Fatalf("Counts after reversing = %d, want 0", got)
	}
}

// An unconfigured board announces itself with Hello, and Run ends with
// ErrLinkClosed when the host side goes away.
func TestRun_HelloThenLinkClosed(t *testing.T) {
	t.Parallel()

	hostEnd, boardEnd := net.Pipe()
	b, err := New(boardEnd, Options{BootID: 42})
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	done := make(chan error, 1)
	go func() { done <- b.Run(context.Background()) }()

	var (
		dec boardlink.Decoder
		pkt boardlink.Packet
	)
	buf := make([]byte, boardlink.MaxEncodedLen)
	_ = hostEnd.SetReadDeadline(time.Now().Add(time.Second))
	for hello := false; !hello; {
		n, readErr := hostEnd.Read(buf)
		if readErr != nil {
			t.Fatalf("reading Hello: %v", readErr)
		}
		for _, c := range buf[:n] {
			if ok, _ := dec.Feed(c, &pkt); ok && pkt.Type == boardlink.TypeHello {
				hello = true
			}
		}
	}
	if pkt.Hello.BootID != 42 {
		t.Errorf("Hello BootID = %d, want 42", pkt.Hello.BootID)
	}

	_ = hostEnd.Close()
	select {
	case runErr := <-done:
		if !errors.Is(runErr, ErrLinkClosed) {
			t.Fatalf("Run = %v, want ErrLinkClosed", runErr)
		}
	case <-time.After(time.Second):
		t.Fatal("Run did not return after the host closed the link")
	}
}
