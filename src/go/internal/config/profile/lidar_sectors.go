package profile

// LidarSectorsConfig mirrors
// platform/shared/config/navigation/sensors/lidar_sectors.toml
// (shared.config.navigation_tuning.LidarSectorParams) in full. Originally
// covered only the two fields internal/nav/directionestimator needed
// (DirectionArcHalfFovDeg, MinValidRangeM); expanded to the full schema
// for internal/nav/controllers' CollisionAvoidanceController/sectors.go,
// which also need the front/threat FOVs, self-detection/no-data
// sentinels, and the four blind-wedge bounds. internal/telemetry/diag has
// its own hardcoded equivalents of the collision-avoidance-facing fields
// predating this profile migration -- not touched here, since that
// package's own ConfigFor-style migration is a separate, later step.
type LidarSectorsConfig struct {
	// FrontHalfFovDeg matches FRONT_HALF_FOV_DEG --
	// compute_forward_clearance's cone.
	FrontHalfFovDeg float64 `mapstructure:"front_half_fov_deg"`
	// ThreatHalfFovDeg matches THREAT_HALF_FOV_DEG --
	// detect_threat_direction's sectors.
	ThreatHalfFovDeg float64 `mapstructure:"threat_half_fov_deg"`
	// SelfDetectionThresholdM matches SELF_DETECTION_THRESHOLD_M.
	SelfDetectionThresholdM float64 `mapstructure:"self_detection_threshold_m"`
	// DirectionArcHalfFovDeg matches DIRECTION_ARC_HALF_FOV_DEG -- the
	// forward-clearance cone utils.py's _forward_clearance uses, NOT the
	// collision-avoidance front sector's own (wider) FOV.
	DirectionArcHalfFovDeg float64 `mapstructure:"direction_arc_half_fov_deg"`
	// MinValidRangeM matches MIN_VALID_RANGE_M.
	MinValidRangeM float64 `mapstructure:"min_valid_range_m"`
	// ThreatNoDetectionRangeM matches THREAT_NO_DETECTION_RANGE_M.
	ThreatNoDetectionRangeM float64 `mapstructure:"threat_no_detection_range_m"`
	// NoDataRangeM matches NO_DATA_RANGE_M.
	NoDataRangeM float64 `mapstructure:"no_data_range_m"`
	// BlindWedgeLeftMinDeg/BlindWedgeLeftMaxDeg match
	// BLIND_WEDGE_LEFT_MIN_DEG/BLIND_WEDGE_LEFT_MAX_DEG.
	BlindWedgeLeftMinDeg float64 `mapstructure:"blind_wedge_left_min_deg"`
	BlindWedgeLeftMaxDeg float64 `mapstructure:"blind_wedge_left_max_deg"`
	// BlindWedgeRightMinDeg/BlindWedgeRightMaxDeg match
	// BLIND_WEDGE_RIGHT_MIN_DEG/BLIND_WEDGE_RIGHT_MAX_DEG.
	BlindWedgeRightMinDeg float64 `mapstructure:"blind_wedge_right_min_deg"`
	BlindWedgeRightMaxDeg float64 `mapstructure:"blind_wedge_right_max_deg"`
}

// DefaultLidarSectorsTOMLPath is
// platform/shared/config/navigation/sensors/lidar_sectors.toml, relative
// to the repo root. No per-component profile overlays -- pass nil
// profileNames to Load.
const DefaultLidarSectorsTOMLPath = "platform/shared/config/navigation/sensors/lidar_sectors.toml"
