package motor

import "time"

// DefaultCommandTimeout is the motor loop's own deadline watchdog: how long
// it will keep driving the last commanded speed after the most recent
// AckermannCmd before treating the command stream as stale and safety-
// stopping. It is a first-derived value for the Go port, exposed as a
// caller-supplied duration rather than hardcoded so it can be tuned on real
// hardware. adr:0068-go-parallel-track-single-cutover
//
// It lives in a file with no build tag, unlike the rest of the loop, so the
// non-Linux build (cmd/pi5's picolink, internal/node/picolink) reads the same
// value instead of a copy that can drift.
const DefaultCommandTimeout = 500 * time.Millisecond
