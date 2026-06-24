// Package telemetry is the robot telemetry bounded context. It owns the
// snapshot pipeline: writing frames, querying history, and distributing
// live updates to subscribers.
package telemetry

import "errors"

var ErrNoSnapshot = errors.New("telemetry: no snapshot received yet")
