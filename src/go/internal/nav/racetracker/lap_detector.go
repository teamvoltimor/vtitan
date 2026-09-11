package racetracker

import (
	"errors"
	"fmt"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// LapDetector is a geometric start/finish-line detector with waypoint-index
// corroboration, matching race_tracker.py's LapDetector.
//
// A lap counts only when BOTH hold:
//
//  1. Geometric: the robot crosses the start/finish line traveling forward,
//     i.e. the dot product against the travel normal goes from negative to
//     non-negative, while in the starting section.
//  2. Waypoint: the waypoint sequence has wrapped at least once since the
//     last confirmed lap.
//
// Either alone over-counts. The geometric test fires again on any jitter
// across the line; the waypoint test fires on a path the robot never
// physically completed.
type LapDetector struct {
	origin       trackmodel.Waypoint
	normal       TravelNormal
	startSection trackmodel.Section

	// prevDot is nil until the first Update, matching Python's `float | None`
	// -- the first sample cannot be a crossing because there is nothing to
	// have crossed FROM.
	prevDot         *float64
	waypointPending bool
}

// ErrNoTravelNormal is returned by NewLapDetector when the
// (section, direction) pair has no travel vector. Python indexes TRAVEL_DIRS
// directly and would raise KeyError; surfacing it at construction means a bad
// pair fails once, at wiring time, instead of on the first tick.
var ErrNoTravelNormal = errors.New("racetracker: no travel normal for section/direction")

// NewLapDetector builds a detector for a start zone at startPos in
// startSection, traveling in direction.
func NewLapDetector(
	startPos trackmodel.Waypoint,
	startSection trackmodel.Section,
	direction trackmodel.Direction,
) (*LapDetector, error) {
	normal, ok := TravelNormalFor(startSection, direction)
	if !ok {
		return nil, fmt.Errorf(
			"%w: section %v, direction %v",
			ErrNoTravelNormal,
			startSection,
			direction,
		)
	}
	return &LapDetector{origin: startPos, normal: normal, startSection: startSection}, nil
}

// NotifyWaypointWrapped records that the waypoint index has wrapped to 0,
// arming the waypoint half of the confirmation.
func (d *LapDetector) NotifyWaypointWrapped() {
	d.waypointPending = true
}

// Update reports whether a confirmed lap crossing occurred at this position.
//
// The section guard matters as much as the dot-product sign: on a closed
// loop the robot's dot against the finish-line normal goes positive once per
// lap ANYWHERE the geometry happens to line up, so without it a crossing
// could be recorded on the far side of the mat.
func (d *LapDetector) Update(robotPos trackmodel.Waypoint, currentSection trackmodel.Section) bool {
	dot := (robotPos.X-d.origin.X)*d.normal.NX + (robotPos.Y-d.origin.Y)*d.normal.NY

	geometricCross := d.prevDot != nil &&
		*d.prevDot < 0.0 &&
		dot >= 0.0 &&
		currentSection == d.startSection

	// Python branches here on geometric_cross and assigns the same value in
	// both arms, so the branch is a no-op and this single assignment is
	// exactly equivalent. Its comment describes an intent ("keep current
	// positive value") that the code does not implement differently -- noted
	// so a reader does not think behavior was dropped in the port.
	d.prevDot = &dot

	if geometricCross && d.waypointPending {
		d.waypointPending = false
		return true
	}
	return false
}

// ApproachingFinish reports whether robotPos is inside withinM of the
// finish line, still short of it, matching race_tracker.py's
// LapDetector.approaching_finish.
//
// Lives here because this type already owns where the finish line is and
// which way the round is driven; a caller reconstructing that from the
// origin and the travel normal would be a second copy of the geometry, free
// to drift from the one that counts laps.
//
// currentSection is NOT redundant with the distance test: dot is a
// projection onto a single travel normal, so the opposite straight
// projects onto the same window (for a SOUTH start driven CCW the normal is
// +x and dot = x - originX, which sweeps the same -withinM..0 range while
// the robot drives the NORTH straight). Distance alone would slow the
// robot on the far side of the track, one straight early, every lap.
//
// Returns true only while the robot is short of the line (dot < 0), so
// this stops applying the moment the crossing is registered.
func (d *LapDetector) ApproachingFinish(
	robotPos trackmodel.Waypoint, currentSection trackmodel.Section, withinM float64,
) bool {
	if currentSection != d.startSection {
		return false
	}
	dot := (robotPos.X-d.origin.X)*d.normal.NX + (robotPos.Y-d.origin.Y)*d.normal.NY
	return -withinM <= dot && dot < 0.0
}
