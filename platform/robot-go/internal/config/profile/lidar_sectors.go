package profile

// LidarSectorsConfig mirrors the subset of
// platform/shared/config/navigation/sensors/lidar_sectors.toml
// (shared.config.navigation_tuning.LidarSectorParams) that
// internal/nav/directionestimator.DirectionFromParkingBay currently
// consumes via its forward-clearance check. internal/telemetry/diag has
// its own hardcoded equivalents of the collision-avoidance-facing fields
// (FrontHalfFOVRad, MinValidRangeM, etc.) predating this profile
// migration -- not touched here, since that package's own ConfigFor-style
// migration is a separate, later step, not part of this one.
type LidarSectorsConfig struct {
	// DirectionArcHalfFovDeg matches DIRECTION_ARC_HALF_FOV_DEG -- the
	// forward-clearance cone utils.py's _forward_clearance uses, NOT the
	// collision-avoidance front sector's own (wider) FOV.
	DirectionArcHalfFovDeg float64 `mapstructure:"direction_arc_half_fov_deg"`
	// MinValidRangeM matches MIN_VALID_RANGE_M.
	MinValidRangeM float64 `mapstructure:"min_valid_range_m"`
}

// DefaultLidarSectorsTOMLPath is
// platform/shared/config/navigation/sensors/lidar_sectors.toml, relative
// to the repo root. No per-component profile overlays -- pass nil
// profileNames to Load.
const DefaultLidarSectorsTOMLPath = "platform/shared/config/navigation/sensors/lidar_sectors.toml"
