// Package smoke exercises the board composition the way cmd/pi5 assembles it:
// several internal/node run loops supervised in one process, talking to each
// other over a real NATS server, with synthetic sensor input going in and
// drive commands expected out.
//
// It exists because nothing else covers that seam. Unit tests cover each node
// package, and test/bagreplay drives the navigator domain packages directly
// from a recorded bag -- neither one connects a publisher to a subscriber, so
// both would pass with the NATS wiring disconnected. When cmd/pi5 grew from
// one supervised target to six, the only evidence the wiring worked was a
// manual run on a dev machine, which is not evidence anyone else can re-check.
//
// # What it deliberately does not cover
//
// No hardware: the drivers are absent, and the IMU here is a level synthetic
// one, so nothing here says the robot would drive correctly -- only that the
// path from a scan to a drive command is connected. The scans are synthesized
// rather than replayed from a bag so the test is hermetic: bags are gitignored
// and CI has none.
//
// It also composes the node packages itself rather than importing cmd/pi5's
// target list, which is package main and not importable. The two must be kept
// in step by hand; if that drifts, move the target list into a package both
// can call.
package smoke
