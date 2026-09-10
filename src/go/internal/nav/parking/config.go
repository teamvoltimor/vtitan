package parking

import (
	"log/slog"
	"math"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/config/profile"
)

// BlockPosition is a single parking block position in world coordinates,
// matching shared.domain.models.BlockPosition.
type BlockPosition struct {
	X, Y float64
}

// ParkingLot is the parking maneuver's geometry input: the two magenta
// blocks that define the bay between them, matching
// shared.domain.models.ParkingLot. Both blocks are required; a lot with
// either missing is a caller bug, since an empty lot would aim the
// controller at a bay that is not there.
type ParkingLot struct {
	Block1 BlockPosition
	Block2 BlockPosition
}

// ParkingLotSpecs mirrors the [parking] section of track.toml
// (shared.config.track_constants.Parking): the dimensions of the magenta
// parking blocks. The bay is the gap between two fins standing
// perpendicular to the wall, so WIDTH is the fin thickness along the wall and
// LENGTH is the fin depth out from it.
type ParkingLotSpecs struct {
	Length float64
	Width  float64
	// WallOffsetM is half of Length: the block is placed by its centre,
	// flush to the wall.
	WallOffsetM float64
	// BlockSpacingFactor is the bay length as a multiple of the robot's
	// LENGTH -- the dimension that has to fit inside the bay. Scaling it by
	// width instead once produced a bay exactly one chassis long, which the
	// robot could never enter.
	BlockSpacingFactor float64
}

// TrackDimensions mirrors the [track] section of track.toml
// (shared.config.track_constants.Track): the mat's coordinate bounds.
type TrackDimensions struct {
	MinCoord float64
	MaxCoord float64
}

// Config is the parallel-park maneuver's tuning, matching
// ParkingParams plus the handful of values it reads from neighboring
// sections (waypoints.ARC_RADIUS for the staging clearance, escape's
// reverse speed/steer for the reposition burst, and robot.toml's
// wheelbase for the yaw tolerance).
type Config struct {
	// ParallelToleranceM is the WRO wheel-to-wall distance-difference rule
	// (2 cm). The yaw tolerance is derived from it as atan(tol / wheelbase),
	// so a wider chassis with the same rule tolerates a smaller angle.
	ParallelToleranceM float64
	// YawTolerance is the wall-parallel tolerance derived from
	// ParallelToleranceM and the wheelbase.
	YawTolerance float64
	// ApproachClearanceM is the staging-point standoff in front of the bay
	// opening, taken from waypoints.ARC_RADIUS.
	ApproachClearanceM float64
	// PosReachDistM is the "reached staging position" threshold.
	PosReachDistM float64
	// DefaultMaxFrames is the frame budget before the maneuver gives up.
	DefaultMaxFrames int
	// SaturatedSteerThreshold is the normalized steering magnitude counted as
	// "at lock".
	SaturatedSteerThreshold float64
	// SaturationStuckTicks is the number of consecutive saturated ticks before
	// a reverse-reorient recovery is latched.
	SaturationStuckTicks int
	// RepositionSpeedMPS is the reverse speed during a recovery burst (m/s,
	// negative). Sourced from escape.REV_SPEED.
	RepositionSpeedMPS float64
	// RepositionSteerMag is the magnitude of the normalized steer held during
	// a recovery burst, biased toward whichever side the target bears.
	// Sourced from escape.REV_STEER_DEG via the wheelbase-derived max steer.
	RepositionSteerMag float64
	// MinLookaheadDistM is the floor distance for the pure-pursuit curvature
	// formula, avoiding a blow-up at the target.
	MinLookaheadDistM float64
	// WheelbaseM is the chassis wheelbase (m), used to derive YawTolerance.
	WheelbaseM float64
	// WallStandoffM is the closest approach to the field wall before ENTER
	// gives up.
	WallStandoffM float64
	// AttemptAfterFinalLap reports whether the navigator should pursue the
	// bay at all once the final lap is banked. False holds position in the
	// finish section instead, which is what ships.
	AttemptAfterFinalLap bool

	// MarkerStandoffM is the closest approach to a parking-bay marker fin
	// before ENTER gives up.
	MarkerStandoffM float64
	// DefaultSpeedMPS is the constant driving speed during the maneuver (m/s).
	DefaultSpeedMPS float64
	// ChassisLengthM/ChassisWidthM are the robot body box dimensions (m),
	// used to lay out the chassis footprint corners.
	ChassisLengthM float64
	ChassisWidthM  float64
	// MaxSteeringAngleRad is the road-wheel angle the navigator may command
	// (rad), fed to the pure-pursuit steering formula. Sourced from
	// robot.toml's effective steering limit.
	MaxSteeringAngleRad float64
}

// Shipped defaults, matching
// src/config/navigation/parking/parking.toml and the
// neighboring sections each cross-referenced value comes from.
const (
	// DefaultParallelToleranceM matches PARALLEL_TOLERANCE_M.
	DefaultParallelToleranceM = 0.02
	// DefaultPosReachDistM matches POS_REACH_DIST_M.
	DefaultPosReachDistM = 0.04
	// DefaultDefaultMaxFrames matches DEFAULT_MAX_FRAMES.
	DefaultDefaultMaxFrames = 400
	// DefaultSaturatedSteerThreshold matches SATURATED_STEER_THRESHOLD.
	DefaultSaturatedSteerThreshold = 0.999
	// DefaultSaturationStuckTicks matches SATURATION_STUCK_TICKS.
	DefaultSaturationStuckTicks = 20
	// DefaultMinLookaheadDistM matches MIN_LOOKAHEAD_DIST_M.
	DefaultMinLookaheadDistM = 0.02
	// DefaultWallStandoffM matches WALL_STANDOFF_M.
	DefaultWallStandoffM = 0.05
	// DefaultMarkerStandoffM matches MARKER_STANDOFF_M.
	DefaultMarkerStandoffM = 0.01
	// DefaultAttemptAfterFinalLap matches ATTEMPT_AFTER_FINAL_LAP.
	DefaultAttemptAfterFinalLap = false

	// DefaultApproachClearanceM matches waypoints.ARC_RADIUS -- the staging
	// standoff in front of the bay opening.
	DefaultApproachClearanceM = 0.45
	// DefaultRepositionSpeedMPS matches escape.REV_SPEED (reverse, negative).
	DefaultRepositionSpeedMPS = -0.20
	// defaultMaxSteeringAngleRad matches the shipped road-wheel limit (55 deg),
	// used to normalize the reposition steer magnitude.
	defaultMaxSteeringAngleRad = 1.2252
	// DefaultRepositionSteerMag matches escape.REV_STEER_DEG (44 deg) at the
	// shipped 55 deg road-wheel limit (1.2252 rad).
	DefaultRepositionSteerMag = 44.0 * math.Pi / 180.0 / defaultMaxSteeringAngleRad
	// DefaultWheelbaseM matches robot.toml's [ackermann] wheelbase (0.19 m).
	DefaultWheelbaseM = 0.19
	// DefaultDefaultSpeedMPS matches parking.SPEED.
	DefaultDefaultSpeedMPS = 0.12
	// DefaultChassisLengthM/DefaultChassisWidthM match robot.toml's
	// [chassis] length/width; the Python RobotSpecs.LENGTH/WIDTH the
	// footprint corners are laid out from.
	DefaultChassisLengthM = 0.30
	DefaultChassisWidthM  = 0.194
	// DefaultMaxSteeringAngleRad matches the shipped road-wheel limit
	// (55 deg), the steering angle the pure-pursuit formula clamps to.
	DefaultMaxSteeringAngleRad = 1.2252

	// DefaultParkingLotLengthM/DefaultParkingLotWidthM mirror track.toml's
	// [parking] length/width.
	DefaultParkingLotLengthM = 0.20
	DefaultParkingLotWidthM  = 0.02
	// DefaultParkingLotWallOffsetM mirrors track.toml's [parking]
	// wall_offset -- half of DefaultParkingLotLengthM, the block being
	// placed by its centre, flush to the wall.
	DefaultParkingLotWallOffsetM = 0.10
	// DefaultParkingLotBlockSpacingFactor mirrors track.toml's [parking]
	// spacing_factor.
	DefaultParkingLotBlockSpacingFactor = 1.5
	// DefaultTrackMinCoordM/DefaultTrackMaxCoordM mirror track.toml's
	// [track] min_coord/max_coord.
	DefaultTrackMinCoordM = 0.0
	DefaultTrackMaxCoordM = 3.0
)

// DefaultParkingLotSpecs matches track.toml's [parking] section.
var DefaultParkingLotSpecs = ParkingLotSpecs{
	Length:             DefaultParkingLotLengthM,
	Width:              DefaultParkingLotWidthM,
	WallOffsetM:        DefaultParkingLotWallOffsetM,
	BlockSpacingFactor: DefaultParkingLotBlockSpacingFactor,
}

// DefaultTrackDimensions matches track.toml's [track] min_coord/max_coord.
var DefaultTrackDimensions = TrackDimensions{
	MinCoord: DefaultTrackMinCoordM,
	MaxCoord: DefaultTrackMaxCoordM,
}

// DefaultConfig returns the Config matching the shipped TOML defaults.
func DefaultConfig() Config {
	yawTolerance := math.Atan2(DefaultParallelToleranceM, DefaultWheelbaseM)
	return Config{
		ParallelToleranceM:      DefaultParallelToleranceM,
		YawTolerance:            yawTolerance,
		ApproachClearanceM:      DefaultApproachClearanceM,
		PosReachDistM:           DefaultPosReachDistM,
		DefaultMaxFrames:        DefaultDefaultMaxFrames,
		SaturatedSteerThreshold: DefaultSaturatedSteerThreshold,
		SaturationStuckTicks:    DefaultSaturationStuckTicks,
		RepositionSpeedMPS:      DefaultRepositionSpeedMPS,
		RepositionSteerMag:      DefaultRepositionSteerMag,
		MinLookaheadDistM:       DefaultMinLookaheadDistM,
		WheelbaseM:              DefaultWheelbaseM,
		WallStandoffM:           DefaultWallStandoffM,
		MarkerStandoffM:         DefaultMarkerStandoffM,
		AttemptAfterFinalLap:    DefaultAttemptAfterFinalLap,
		DefaultSpeedMPS:         DefaultDefaultSpeedMPS,
		ChassisLengthM:          DefaultChassisLengthM,
		ChassisWidthM:           DefaultChassisWidthM,
		MaxSteeringAngleRad:     DefaultMaxSteeringAngleRad,
	}
}

// ConfigFor resolves the Config to run with: DefaultConfig's literals,
// overlaid with parking.toml, escape.toml, waypoints.toml, and robot.toml
// (the last for the wheelbase that feeds the yaw tolerance and the
// road-wheel limit that feeds the reposition steer magnitude) if configRoot
// is non-empty and loading succeeds; otherwise, or on any load failure, the
// literal defaults, logging why. Each file loads and falls back
// independently, since they are unrelated failure domains.
//
// hardwareProfileNames names the active robot hardware profile(s) (e.g.
// servo/motor overlays); robot.toml requires them for the wheelbase and
// steering-limit fields, so omitting them falls back to the literal
// defaults for those two derived values rather than erroring.
func ConfigFor(logger *slog.Logger, configRoot string, hardwareProfileNames []string) Config {
	cfg := DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	parkingPath := filepath.Join(configRoot, profile.DefaultParkingTOMLPath)
	if loaded, err := profile.Load[profile.ParkingConfig](parkingPath, nil); err != nil {
		logger.Warn("parking: loading parking.toml, falling back to defaults", "error", err)
	} else {
		cfg.ParallelToleranceM = loaded.ParallelToleranceM
		cfg.PosReachDistM = loaded.PosReachDistM
		cfg.DefaultMaxFrames = loaded.DefaultMaxFrames
		cfg.SaturatedSteerThreshold = loaded.SaturatedSteerThreshold
		cfg.SaturationStuckTicks = loaded.SaturationStuckTicks
		cfg.DefaultSpeedMPS = loaded.Speed
		cfg.MinLookaheadDistM = loaded.MinLookaheadDistM
		cfg.WallStandoffM = loaded.WallStandoffM
		cfg.MarkerStandoffM = loaded.MarkerStandoffM
		cfg.AttemptAfterFinalLap = loaded.AttemptAfterFinalLap
		// Yaw tolerance tracks the wheelbase that feeds it; default keeps the
		// literal value until robot.toml is applied below.
		cfg.YawTolerance = math.Atan2(loaded.ParallelToleranceM, cfg.WheelbaseM)
	}

	escapePath := filepath.Join(configRoot, profile.DefaultEscapeTOMLPath)
	var escape *profile.EscapeConfig
	if loaded, err := profile.Load[profile.EscapeConfig](escapePath, nil); err != nil {
		logger.Warn("parking: loading escape.toml, falling back to defaults", "error", err)
	} else {
		cfg.RepositionSpeedMPS = loaded.RevSpeed
		escape = loaded
	}

	waypointsPath := filepath.Join(configRoot, profile.DefaultWaypointsTOMLPath)
	if loaded, err := profile.Load[profile.WaypointsConfig](waypointsPath, nil); err != nil {
		logger.Warn("parking: loading waypoints.toml, falling back to defaults", "error", err)
	} else {
		cfg.ApproachClearanceM = loaded.ArcRadius
	}

	robotPath := filepath.Join(configRoot, profile.DefaultRobotTOMLPath)
	if loaded, err := profile.LoadRobotConfig(robotPath, hardwareProfileNames); err != nil {
		logger.Warn("parking: loading robot.toml, falling back to defaults", "error", err)
	} else {
		cfg.WheelbaseM = loaded.Ackermann.Wheelbase
		cfg.ChassisLengthM = loaded.Chassis.Length
		cfg.ChassisWidthM = loaded.Chassis.Width
		cfg.MaxSteeringAngleRad = loaded.MaxSteeringAngle()
		maxSteer := loaded.MaxSteeringAngle()
		if maxSteer > 0 && escape != nil {
			cfg.RepositionSteerMag = escape.RevSteerNorm(maxSteer)
		}
		// Yaw tolerance follows the now-known wheelbase.
		cfg.YawTolerance = math.Atan2(cfg.ParallelToleranceM, cfg.WheelbaseM)
	}

	return cfg
}
