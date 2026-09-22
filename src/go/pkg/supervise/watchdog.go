package supervise

import (
	"context"
	"log/slog"
	"os"
	"strconv"
	"time"
)

// watchdogUsecEnv and watchdogPidEnv are the fixed sd_watchdog_enabled(3)
// protocol env vars systemd sets on a unit with WatchdogSec= configured --
// protocol facts, not deployment tunables, so package constants rather than
// Config fields.
const (
	watchdogUsecEnv = "WATCHDOG_USEC"
	watchdogPidEnv  = "WATCHDOG_PID"
)

// watchdogPingDivisor halves the WATCHDOG_USEC-derived timeout to get the
// ping interval: sd_watchdog_enabled(3) documents that a well-behaved
// service should call sd_notify "WATCHDOG=1" at roughly half the configured
// timeout, so a single missed wakeup (scheduling jitter, a slow GC pause)
// doesn't by itself cross systemd's dead-man's-switch threshold.
const watchdogPingDivisor = 2

// WatchdogTimeout reads $WATCHDOG_USEC and returns it as a time.Duration
// -- this is the raw timeout systemd expects a "WATCHDOG=1" ping within,
// not yet halved into a ping interval (see RunWatchdog). ok is false when
// the watchdog isn't configured for this process at all: WATCHDOG_USEC is
// unset or unparseable, or, per sd_watchdog_enabled(3), WATCHDOG_PID is set
// and doesn't match os.Getpid() -- that second case means the env vars
// were inherited from a supervisor chain and describe watchdog config for
// a different process, not this one.
func WatchdogTimeout() (timeout time.Duration, ok bool) {
	usecStr := os.Getenv(watchdogUsecEnv)
	if usecStr == "" {
		return 0, false
	}

	usec, err := strconv.ParseInt(usecStr, 10, 64)
	if err != nil || usec <= 0 {
		return 0, false
	}

	if pidStr := os.Getenv(watchdogPidEnv); pidStr != "" {
		pid, pidErr := strconv.Atoi(pidStr)
		if pidErr != nil || pid != os.Getpid() {
			return 0, false
		}
	}

	return time.Duration(usec) * time.Microsecond, true
}

// RunWatchdog sends a "WATCHDOG=1" sd_notify ping on a ticker until ctx is
// done. timeout is the raw watchdog timeout (e.g. from WatchdogTimeout,
// which is exactly WATCHDOG_USEC as systemd configured it) -- RunWatchdog
// itself divides by watchdogPingDivisor to get the ping interval, so
// callers must pass the timeout, not an already-halved interval, or pings
// will arrive twice as slowly as systemd expects. A failed Notify is
// logged and the loop keeps going -- one dropped ping is not a reason to
// stop trying, systemd's own timeout is what decides whether the process
// gets killed.
func RunWatchdog(ctx context.Context, logger *slog.Logger, timeout time.Duration) {
	ticker := time.NewTicker(timeout / watchdogPingDivisor)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			if err := Notify(ctx, WatchdogState); err != nil {
				logger.Error("supervise: sending watchdog ping", "error", err)
			}
		}
	}
}
