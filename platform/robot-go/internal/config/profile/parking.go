package profile

// ParkingConfig mirrors
// platform/config/navigation/parking/parking.toml
// (shared.config.navigation_tuning.parking.ParkingParams), the tuning the
// parallel-park maneuver (internal/nav/parking) reads. Only the fields the
// Go maneuver actually consumes are mirrored; the rest of the Python
// tuning tree's parking section is omitted rather than mirrored unused.
type ParkingConfig struct {
	// ParallelToleranceM matches PARALLEL_TOLERANCE_M -- the WRO wheel-to-wall
	// distance-difference rule, used to derive the yaw tolerance.
	ParallelToleranceM float64 `mapstructure:"parallel_tolerance_m"`
	// PosReachDistM matches POS_REACH_DIST_M -- "reached staging position"
	// threshold.
	PosReachDistM float64 `mapstructure:"pos_reach_dist_m"`
	// DefaultMaxFrames matches DEFAULT_MAX_FRAMES -- give up after this many
	// control ticks.
	DefaultMaxFrames int `mapstructure:"default_max_frames"`
	// SaturatedSteerThreshold matches SATURATED_STEER_THRESHOLD -- normalized
	// steering magnitude counted as "at lock".
	SaturatedSteerThreshold float64 `mapstructure:"saturated_steer_threshold"`
	// SaturationStuckTicks matches SATURATION_STUCK_TICKS -- saturated ticks
	// before reverse-reorient.
	SaturationStuckTicks int `mapstructure:"saturation_stuck_ticks"`
	// Speed matches SPEED -- constant driving speed during the maneuver (m/s).
	Speed float64 `mapstructure:"speed"`
	// MinLookaheadDistM matches MIN_LOOKAHEAD_DIST_M -- floor distance to avoid
	// curvature blow-up.
	MinLookaheadDistM float64 `mapstructure:"min_lookahead_dist_m"`
	// WallStandoffM matches WALL_STANDOFF_M -- closest approach to the field
	// wall before the maneuver gives up.
	WallStandoffM float64 `mapstructure:"wall_standoff_m"`
	// MarkerStandoffM matches MARKER_STANDOFF_M -- closest approach to a
	// parking-bay marker fin before the maneuver gives up.
	MarkerStandoffM float64 `mapstructure:"marker_standoff_m"`
}

// DefaultParkingTOMLPath is
// platform/config/navigation/parking/parking.toml, relative to the
// repo root. No per-component profile overlays -- pass nil profileNames to
// Load.
const DefaultParkingTOMLPath = "platform/config/navigation/parking/parking.toml"
