package supervise_test

import (
	"context"
	"os"
	"strconv"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/supervise"
)

func TestWatchdogTimeout(t *testing.T) {
	otherPID := os.Getpid() + 1

	tests := []struct {
		name        string
		watchdogSec string
		watchdogPID string
		wantTimeout time.Duration
		wantOK      bool
	}{
		{
			name:   "unset",
			wantOK: false,
		},
		{
			name:        "unparseable",
			watchdogSec: "not-a-number",
			wantOK:      false,
		},
		{
			name:        "zero",
			watchdogSec: "0",
			wantOK:      false,
		},
		{
			name:        "negative",
			watchdogSec: "-1000000",
			wantOK:      false,
		},
		{
			name:        "enabled, no pid filter",
			watchdogSec: "30000000",
			wantTimeout: 30 * time.Second,
			wantOK:      true,
		},
		{
			name:        "enabled, pid matches this process",
			watchdogSec: "30000000",
			watchdogPID: strconv.Itoa(os.Getpid()),
			wantTimeout: 30 * time.Second,
			wantOK:      true,
		},
		{
			name:        "enabled, pid belongs to a different process",
			watchdogSec: "30000000",
			watchdogPID: strconv.Itoa(otherPID),
			wantOK:      false,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Setenv("WATCHDOG_USEC", tt.watchdogSec)
			t.Setenv("WATCHDOG_PID", tt.watchdogPID)

			gotTimeout, gotOK := supervise.WatchdogTimeout()
			if gotOK != tt.wantOK {
				t.Fatalf("WatchdogTimeout() ok = %v, want %v", gotOK, tt.wantOK)
			}
			if gotOK && gotTimeout != tt.wantTimeout {
				t.Errorf("WatchdogTimeout() timeout = %v, want %v", gotTimeout, tt.wantTimeout)
			}
		})
	}
}

func TestRunWatchdog_PingsAtHalfTheConfiguredTimeout(t *testing.T) {
	socketPath, listener := listenNotifySocket(t)
	t.Setenv("NOTIFY_SOCKET", socketPath)

	const timeout = 20 * time.Millisecond

	ctx, cancel := context.WithCancel(t.Context())
	defer cancel()

	go supervise.RunWatchdog(ctx, discardLogger(), timeout)

	first := readOneDatagram(t, listener)
	if first != supervise.WatchdogState {
		t.Fatalf("first datagram = %q, want %q", first, supervise.WatchdogState)
	}

	started := time.Now()
	second := readOneDatagram(t, listener)
	elapsed := time.Since(started)
	if second != supervise.WatchdogState {
		t.Fatalf("second datagram = %q, want %q", second, supervise.WatchdogState)
	}

	// The ping interval is timeout/2 (10ms here); assert it's clearly
	// shorter than timeout itself rather than pinning an exact duration,
	// which would make this test flaky under CI scheduling jitter.
	if elapsed >= timeout {
		t.Errorf(
			"elapsed between pings = %v, want < timeout (%v), i.e. pinging faster than once per timeout",
			elapsed, timeout,
		)
	}
}

func TestRunWatchdog_StopsOnContextCancel(t *testing.T) {
	socketPath, listener := listenNotifySocket(t)
	t.Setenv("NOTIFY_SOCKET", socketPath)

	ctx, cancel := context.WithCancel(context.Background())

	done := make(chan struct{})
	go func() {
		supervise.RunWatchdog(ctx, discardLogger(), 10*time.Millisecond)
		close(done)
	}()

	readOneDatagram(t, listener) // wait for at least one ping so the goroutine is definitely running
	cancel()

	select {
	case <-done:
	case <-time.After(notifyReadTimeout):
		t.Fatal("RunWatchdog did not return after ctx cancellation")
	}
}
