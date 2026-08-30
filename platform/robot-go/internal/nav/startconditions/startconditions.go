// Package startconditions computes where the robot believes it starts,
// without being handed a scenario file. Ports src/navigation/start_conditions.py.
//
// The starting pose is pure track geometry -- corridor centerline, biased
// away from the inner block, aligned with travel -- so it can be computed
// from the layout the robot believes it is on rather than read from
// metadata. That is what lets the deployed navigator run with no scenario
// file at all.
//
// # Why the section can be assumed but the direction cannot
//
// Section is free: the robot defines its own world frame by declaring its
// starting corridor to be the south one, and everything else follows. If it
// is physically in the east corridor, its entire map is the true map
// rotated 90 degrees, and since it learns corridor widths against its own
// labels the map stays self-consistent. The path it drives is correct in
// its own frame, which is the only frame it acts in.
//
// Direction is not. Assuming south-clockwise also asserts the inner block
// is on the robot's right. If the robot is actually traveling the other
// way round the loop the block is on its left, the assumption is wrong by a
// REFLECTION rather than a rotation, and no amount of width learning
// recovers it -- it steers toward the wall it thinks is the far one.
// Reflection is not a symmetry of the labeled track, so travel direction
// has to come from outside: a launch parameter or a jumper, the same way
// the challenge mode does.
package startconditions

import (
	"math"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/racetracker"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/waypoints"
)

// Config is start-conditions' tuning: the track geometry StartPose needs
// plus the waypoint generator's centerline-bias split, which StartPose
// reuses rather than reimplementing so a spawn pose can never drift from
// the planner's own centerline.
type Config struct {
	// TrackMaxCoordM is the track's outer boundary
	// (profile.TrackConfig.Track.MaxCoord).
	TrackMaxCoordM float64
	// NarrowWidthM is the corridor width AssumedStartConditions believes
	// every side is, when the caller supplies no widths of its own -- the
	// safe prior blind operation starts from.
	NarrowWidthM float64
	// Waypoints parameterizes the narrow/wide centerline-bias split; see
	// waypoints.CenterBiasForCorridor.
	Waypoints waypoints.Config
}

// StartingConditions is a starting_conditions mapping in scenario-metadata
// shape, matching assumed_start_conditions' return value.
type StartingConditions struct {
	Direction trackmodel.Direction
	Section   trackmodel.Section
	X, Y, Yaw float64
}

// CanonicalSection is the section a robot assumes when it hasn't been told
// which one it is in. Any choice works -- see the package doc -- so this is
// a label, not a claim about where the robot physically is. Matches
// shared.domain.enums.Section.canonical().
const CanonicalSection = trackmodel.South

// Default track-geometry values, matching track.toml's [track]/[corridor]
// sections.
const (
	// DefaultTrackMaxCoordM matches track.toml's [track] max_coord.
	DefaultTrackMaxCoordM = 3.0
	// DefaultNarrowWidthM matches track.toml's [corridor] narrow.
	DefaultNarrowWidthM = 0.6
)

// DefaultConfig returns the Config matching the shipped TOML defaults.
func DefaultConfig() Config {
	return Config{
		TrackMaxCoordM: DefaultTrackMaxCoordM,
		NarrowWidthM:   DefaultNarrowWidthM,
		Waypoints:      waypoints.DefaultConfig(),
	}
}

// StartPose returns the spawn pose on the biased corridor centerline,
// aligned with travel, and ok=false when section/direction is not one of
// the four known corridors/travel directions.
//
// widthsM must have all four trackmodel.Section keys populated, in meters.
// centerBiasM, when non-nil, pins EVERY corridor to this magnitude and
// skips the narrow/wide split -- see waypoints.CenterBiasForCorridor for
// when a caller wants that (the Obstacles Challenge does).
func StartPose(
	section trackmodel.Section,
	direction trackmodel.Direction,
	widthsM map[trackmodel.Section]float64,
	cfg Config,
	centerBiasM *float64,
) (x, y, yaw float64, ok bool) {
	bias := func(widthM float64) float64 {
		return waypoints.CenterBiasForCorridor(widthM, cfg.Waypoints, centerBiasM)
	}

	trackCenter := cfg.TrackMaxCoordM / 2
	var cx, cy float64
	switch section {
	case trackmodel.South:
		cx, cy = trackCenter, widthsM[trackmodel.South]/2+bias(widthsM[trackmodel.South])
	case trackmodel.North:
		cx, cy = trackCenter, cfg.TrackMaxCoordM-widthsM[trackmodel.North]/2-bias(
			widthsM[trackmodel.North],
		)
	case trackmodel.East:
		cx, cy = cfg.TrackMaxCoordM-widthsM[trackmodel.East]/2-bias(
			widthsM[trackmodel.East],
		), trackCenter
	case trackmodel.West:
		cx, cy = widthsM[trackmodel.West]/2+bias(widthsM[trackmodel.West]), trackCenter
	default:
		return 0, 0, 0, false
	}

	normal, ok := racetracker.TravelNormalFor(section, direction)
	if !ok {
		return 0, 0, 0, false
	}
	return cx, cy, math.Atan2(normal.NY, normal.NX), true
}

// AssumedStartConditions returns the starting conditions for a robot that
// was told nothing but the direction, and ok=false when section/direction
// don't resolve to a pose (see StartPose).
//
// direction is the one thing that cannot be assumed -- see the package doc.
// widthsM is the layout the robot believes it is on; nil defaults to every
// corridor at cfg.NarrowWidthM, which interacts with the narrow/wide bias
// split: the default belief is narrow, so a blind robot assumes the narrow
// bias until it measures otherwise, keeping the assumed start on the path
// it will actually be given.
func AssumedStartConditions(
	direction trackmodel.Direction,
	widthsM map[trackmodel.Section]float64,
	section trackmodel.Section,
	cfg Config,
	centerBiasM *float64,
) (StartingConditions, bool) {
	believed := widthsM
	if believed == nil {
		believed = map[trackmodel.Section]float64{
			trackmodel.North: cfg.NarrowWidthM,
			trackmodel.South: cfg.NarrowWidthM,
			trackmodel.East:  cfg.NarrowWidthM,
			trackmodel.West:  cfg.NarrowWidthM,
		}
	}
	x, y, yaw, ok := StartPose(section, direction, believed, cfg, centerBiasM)
	if !ok {
		return StartingConditions{}, false
	}
	return StartingConditions{Direction: direction, Section: section, X: x, Y: y, Yaw: yaw}, true
}
