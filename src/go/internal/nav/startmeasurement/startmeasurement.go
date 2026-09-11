// Package startmeasurement measures where the robot actually is, instead of
// assuming where it was put. Ports src/navigation/start_measurement.py.
//
// startconditions.StartPose computes a starting pose from geometry alone:
// the corridor centerline, at the middle of the mat's side. Nothing
// measures it, so it is an assertion about the operator's placement rather
// than an observation of it -- and the assertion is wrong even when it is
// close. The marked starting square spans one meter along the corridor,
// split into two half-meter cells centered at 1.25 and 1.75, so the assumed
// 1.5 sits exactly on the boundary between them: the one along-corridor
// position the robot can never legally occupy.
//
// Measured on real hardware 2026-08-05: two CCW rounds were set down near
// the far end of the corridor with 0.69 m of clear track ahead while the
// plan, built from the assumed start, expected roughly 1.5 m. The robot
// drove into the wall in six seconds with the steering barely off center,
// because nothing downstream can discover a starting error the localizer
// is not looking for -- its search is local (see internal/nav/localization),
// so an error of that size is permanently outside its reach.
//
// The scan already contains the answer. With the chassis aligned to the
// corridor, the four cardinal rays give the distance to the wall ahead, the
// wall behind and each side, and those ARE the position, expressed relative
// to the corridor the robot is standing in. Measured against the recorded
// rounds' true poses the four rays agreed to within 1-4 cm.
//
// Which side of the mat the robot is on is neither knowable nor needed:
// with equal corridors the track is symmetric under 90 degree rotation, so
// the four candidate sides score identically on any scan, and the robot
// declares its own starting section anyway (see internal/nav/startconditions).
// What is knowable, and what actually matters, is how far along that
// corridor it stands and how far it has before the corner it is driving at.
package startmeasurement

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// Config is start-measurement's tuning, matching StartMeasurementParams
// plus the track/LIDAR geometry the rays are read against.
type Config struct {
	// RayHalfWidthDeg is the half-angle of each cardinal wedge (forward,
	// back, left, right) the medians are taken over.
	RayHalfWidthDeg float64
	// ClosingToleranceM is how far forward+back may fall short of
	// TrackMaxCoordM before the reading is rejected -- see
	// MeasureStartPose's doc comment for why 0.15 m is generous on
	// purpose.
	ClosingToleranceM float64

	// TrackMaxCoordM is the track's outer boundary
	// (profile.TrackConfig.Track.MaxCoord).
	TrackMaxCoordM float64
	LidarMinRangeM float64
	LidarMaxRangeM float64
}

// MeasuredStart is a starting pose read off the track rather than assumed.
type MeasuredStart struct {
	X, Y float64
	// DistanceAheadM is the clear track between the robot and the wall it
	// faces -- the number whose absence caused the 2026-08-05 failures.
	DistanceAheadM float64
	// OuterWallDistanceM is the distance to the outer wall, across the
	// corridor.
	OuterWallDistanceM float64
	// CorridorWidthM is the width of the corridor the robot stands in, and
	// meaningful only when CorridorWidthKnown is true. Known is false when
	// the robot is level with a corner rather than the inner block and both
	// side rays reach outer walls, which measures the mat and not a
	// corridor -- matches Python's corridor_width_m: float | None.
	CorridorWidthM     float64
	CorridorWidthKnown bool
}

// Shipped defaults, matching
// src/config/navigation/sensors/start_measurement.toml,
// track.toml's [track] section and robot.toml's [lidar] section.
const (
	// DefaultRayHalfWidthDeg matches start_measurement.toml's
	// ray_half_width_deg.
	DefaultRayHalfWidthDeg = 4.0
	// DefaultClosingToleranceM matches start_measurement.toml's
	// closing_tolerance_m.
	DefaultClosingToleranceM = 0.15

	// DefaultTrackMaxCoordM matches track.toml's [track] max_coord.
	DefaultTrackMaxCoordM = 3.0
	// DefaultLidarMinRangeM matches robot.toml's [lidar] min_range.
	DefaultLidarMinRangeM = 0.045
	// DefaultLidarMaxRangeM matches robot.toml's [lidar] max_range.
	DefaultLidarMaxRangeM = 12.0

	// lidarMaxRangeMargin matches RobotSpecs.LIDAR_MAX_RANGE * 0.99: the C1
	// reports its own max range on a no-return, so the valid window is
	// pinned just under it rather than at it.
	lidarMaxRangeMargin = 0.99
)

// sectionRotations are the sections in 90-degree rotation order, starting
// from the frame poses are built in, matching Python's _SECTION_ROTATIONS.
var sectionRotations = [4]trackmodel.Section{
	trackmodel.South,
	trackmodel.East,
	trackmodel.North,
	trackmodel.West,
}

// DefaultConfig returns the Config matching the shipped TOML defaults.
func DefaultConfig() Config {
	return Config{
		RayHalfWidthDeg:   DefaultRayHalfWidthDeg,
		ClosingToleranceM: DefaultClosingToleranceM,
		TrackMaxCoordM:    DefaultTrackMaxCoordM,
		LidarMinRangeM:    DefaultLidarMinRangeM,
		LidarMaxRangeM:    DefaultLidarMaxRangeM,
	}
}

// rotateInto rotates a pose built in the SOUTH frame into section's frame.
//
// A quarter turn about the mat's center maps each section onto the next,
// and with equal corridors the track is invariant under it -- which is
// exactly why the section is a free choice rather than something to be
// measured.
func rotateInto(
	section trackmodel.Section,
	trackMaxCoordM, x, y float64,
) (rotatedX, rotatedY float64) {
	turns := 0
	for i, s := range sectionRotations {
		if s == section {
			turns = i
			break
		}
	}
	rotatedX, rotatedY = x, y
	for range turns {
		rotatedX, rotatedY = trackMaxCoordM-rotatedY, rotatedX
	}
	return rotatedX, rotatedY
}

// MeasureStartPose reads the robot's pose out of a scan, assuming only its
// corridor and direction, and ok=false when the scan cannot support one --
// a ray with no valid return, or opposite rays that do not span the mat,
// which means something is standing in one of them. ok=false is a refusal
// to guess, and callers should treat it as "do not race", not as "use the
// old assumption".
//
// rangesM/anglesRad must already be sanitized (no NaN/inf); anglesRad is in
// the robot frame, 0 = forward. direction decides which way along the
// corridor "ahead" points, and which side the outer wall is on --
// clockwise keeps the inner block on the robot's right, counterclockwise on
// its left, so the outer wall is opposite it.
//
// cfg.ClosingToleranceM is how far forward+back may fall short of the mat
// before the reading is rejected. Opposite rays along a corridor must span
// the mat, so their sum is a free validity check -- it needs no knowledge
// of where the robot is. Sized from real scans, not nominally: two recorded
// rounds on a properly set-up track summed to 2.978 m and 2.971 m against a
// nominal 3.0, so the honest error on good data is already 2-3 cm before
// LIDAR noise, mat seams, or walls that are not quite square. The 0.15 m
// default is five times that, while the failure this rejects -- a hand, a
// bystander, or a sign standing in one of the rays -- misses by a meter or
// more. The margin is deliberately generous: a false rejection costs a
// re-run, and a false acceptance costs the round.
func MeasureStartPose(
	scan controllers.LidarScan,
	direction trackmodel.Direction,
	section trackmodel.Section,
	cfg Config,
) (MeasuredStart, bool) {
	rayHalfWidthRad := cfg.RayHalfWidthDeg * math.Pi / navutil.DegreesPerHalfTurn
	maxValidRangeM := cfg.LidarMaxRangeM * lidarMaxRangeMargin

	forward, ok := navutil.WedgeMedian(
		scan, 0, rayHalfWidthRad, cfg.LidarMinRangeM, 0, maxValidRangeM,
	)
	if !ok {
		return MeasuredStart{}, false
	}
	back, ok := navutil.WedgeMedian(
		scan, math.Pi, rayHalfWidthRad, cfg.LidarMinRangeM, 0, maxValidRangeM,
	)
	if !ok {
		return MeasuredStart{}, false
	}
	left, ok := navutil.WedgeMedian(
		scan, math.Pi/2, rayHalfWidthRad, cfg.LidarMinRangeM, 0, maxValidRangeM,
	)
	if !ok {
		return MeasuredStart{}, false
	}
	right, ok := navutil.WedgeMedian(
		scan, -math.Pi/2, rayHalfWidthRad, cfg.LidarMinRangeM, 0, maxValidRangeM,
	)
	if !ok {
		return MeasuredStart{}, false
	}

	if math.Abs((forward+back)-cfg.TrackMaxCoordM) > cfg.ClosingToleranceM {
		return MeasuredStart{}, false
	}

	// Clockwise travel keeps the inner block to the right, so the outer
	// wall is the left ray; counterclockwise is the mirror of that.
	clockwise := direction == trackmodel.Clockwise
	outer, inner := left, right
	if !clockwise {
		outer, inner = right, left
	}

	// Both side rays reaching outer walls measures the mat across, not a
	// corridor: the robot is level with a corner, past the inner block.
	// Only a sum that falls short of the mat is a corridor width.
	span := outer + inner
	widthKnown := span < cfg.TrackMaxCoordM-cfg.ClosingToleranceM

	// back is the distance to the wall behind, so it IS the along-corridor
	// coordinate when traveling in the axis' positive direction, and the
	// mat less that when traveling against it.
	along := back
	if clockwise {
		along = cfg.TrackMaxCoordM - back
	}
	x, y := rotateInto(section, cfg.TrackMaxCoordM, along, outer)

	return MeasuredStart{
		X: x, Y: y,
		DistanceAheadM:     forward,
		OuterWallDistanceM: outer,
		CorridorWidthM:     span,
		CorridorWidthKnown: widthKnown,
	}, true
}
