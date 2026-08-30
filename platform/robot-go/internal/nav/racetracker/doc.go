// Package racetracker counts laps and tracks race progress, porting
// src/navigation/race_tracker.py.
//
// Two independent things live here because they do in Python, and
// corridor_estimator/start_conditions import TRAVEL_DIRS from this module:
//
//   - LapDetector, the geometric start/finish-line crossing detector.
//   - RaceTracker, the bookkeeping for elapsed time, distance and splits.
//
// # Why lap detection is not just a waypoint wrap
//
// The navigator currently counts a lap whenever the waypoint index runs off
// the end of the path. Python treats that as the FALLBACK and prefers this
// detector, which requires a geometric crossing of the start/finish line AND
// a waypoint wrap since the last confirmed lap. Requiring both is what stops
// overshoot, a stuck loop on the line, or a waypoint skip near the finish
// from counting twice -- failure modes with real history on this robot (see
// the 2026-08-06 counterclockwise round, where the lap count stuck at zero
// for seven minutes).
//
// # Time is injected
//
// RaceTracker reads the clock on nearly every call, so it takes one through
// WithClock rather than calling time.Now directly. Python's tests sleep to
// advance real time; injecting instead keeps the Go tests deterministic and
// fast, and costs one option at construction.
package racetracker
