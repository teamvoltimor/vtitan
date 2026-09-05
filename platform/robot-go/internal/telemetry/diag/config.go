package diag

import (
	"fmt"
	"math"

	"github.com/go-playground/validator/v10"
)

// AngleWedge is a bearing range (radians, 0 = forward, +pi/2 = left) that a
// sector query excludes outright, matching sector_ranges' blind-wedge mask
// in platform/robot/src/navigation/control/controllers/collision_avoidance/
// sectors.py — the two rear-corner mount-occlusion wedges where LIDAR
// self-collision reads as a real close range at every distance, so they
// must be filtered by angle rather than distance. Disabled by default
// (Enabled=false) rather than a real-but-guessed default: the concrete
// wedge geometry lives in the not-yet-ported NavigationTuning system, so a
// wedge is only applied once a caller explicitly supplies one from real
// tuning data.
type AngleWedge struct {
	MinRad  float64
	MaxRad  float64
	Enabled bool
}

// Config parameterizes Aggregator's sector-clearance computation. Every
// field here is config-driven rather than a literal buried in sector.go,
// mirroring the source of truth these values come from on the Python side
// (platform/shared/src/shared/config/navigation_tuning/sensors.py's
// LidarSectorsTuning) — DefaultConfig ships that file's own default values
// so behavior matches out of the box, but nothing here is hardcoded past
// that default; a caller with real hardware-profile tuning overrides it.
type Config struct {
	// FrontHalfFOVRad is the half-width of the front/left/right sectors,
	// matching LidarSectorsTuning.FRONT_HALF_FOV_DEG.
	FrontHalfFOVRad float64
	// MinValidRangeM is the lower bound below which a range reading is
	// treated as invalid/no-return, matching LidarSectorsTuning.MIN_VALID_RANGE_M.
	// Used for the front sector, which does not filter self-detection.
	MinValidRangeM float64
	// SelfDetectionThresholdM is the lower bound used instead of
	// MinValidRangeM for sectors that filter chassis self-detection
	// (left/right), matching LidarSectorsTuning.SELF_DETECTION_THRESHOLD_M.
	SelfDetectionThresholdM float64
	// MaxValidRangeM is the upper bound above which a range reading is
	// treated as a fabricated no-return substitute rather than a genuine
	// long reading, matching RobotSpecs.LIDAR_MAX_RANGE minus the
	// no-return margin sectors.py applies (see DefaultMaxValidRangeM).
	MaxValidRangeM float64
	// BlindWedgeLeft and BlindWedgeRight exclude the two rear-corner
	// mount-occlusion wedges by angle. Disabled by default — see
	// AngleWedge's doc comment.
	BlindWedgeLeft  AngleWedge
	BlindWedgeRight AngleWedge
}

const (
	// DefaultFrontHalfFOVRad matches LidarSectorsTuning.FRONT_HALF_FOV_DEG's
	// default (30 deg), converted to radians.
	DefaultFrontHalfFOVRad = 30.0 * math.Pi / 180.0
	// DefaultMinValidRangeM matches LidarSectorsTuning.MIN_VALID_RANGE_M's default.
	DefaultMinValidRangeM = 0.05
	// DefaultSelfDetectionThresholdM matches
	// LidarSectorsTuning.SELF_DETECTION_THRESHOLD_M's default.
	DefaultSelfDetectionThresholdM = 0.08
	// lidarMaxRangeM matches RobotSpecs.LIDAR_MAX_RANGE's default
	// (platform/shared/config/robot.toml's lidar.max_range, the Slamtec
	// C1's spec ceiling).
	lidarMaxRangeM = 12.0
	// noReturnMarginM matches sectors.py's _NO_RETURN_MARGIN_M: how far
	// below lidarMaxRangeM still counts as the hardware gateway's
	// fabricated no-return substitute, not a genuine long reading.
	noReturnMarginM = 0.05
	// DefaultMaxValidRangeM is lidarMaxRangeM minus noReturnMarginM.
	DefaultMaxValidRangeM = lidarMaxRangeM - noReturnMarginM
)

// DefaultConfig returns a Config seeded with LidarSectorsTuning's shipped
// defaults and blind wedges disabled (see AngleWedge).
//
// There is deliberately no LIDAR mount correction here. Scans reach this
// package already in the robot frame -- lidar.ConfigFor resolves
// robot.toml's [lidar].inverted/mount_yaw_offset_deg into the driver, so
// the correction is applied once, at the source. This package used to
// re-apply it from its own copy of the config, which is how it came to use
// a constant rotation after the driver had established that the mount
// needs a mirror.
func DefaultConfig() Config {
	return Config{
		FrontHalfFOVRad:         DefaultFrontHalfFOVRad,
		MinValidRangeM:          DefaultMinValidRangeM,
		SelfDetectionThresholdM: DefaultSelfDetectionThresholdM,
		MaxValidRangeM:          DefaultMaxValidRangeM,
		BlindWedgeLeft:          AngleWedge{},
		BlindWedgeRight:         AngleWedge{},
	}
}

// Validate reports whether c is usable, backed by go-playground/validator
// tags on an internal mirror struct (validator needs struct tags, and
// Config's fields are documentation-heavy enough that inlining tags here
// would hurt readability more than a small mirror costs).
func (c Config) Validate() error {
	type constraints struct {
		FrontHalfFOVRad         float64 `validate:"gt=0,lte=3.141592653589793"`
		MinValidRangeM          float64 `validate:"gt=0"`
		SelfDetectionThresholdM float64 `validate:"gt=0"`
		MaxValidRangeM          float64 `validate:"gtfield=MinValidRangeM"`
	}
	v := validator.New()
	if err := v.Struct(constraints{
		FrontHalfFOVRad:         c.FrontHalfFOVRad,
		MinValidRangeM:          c.MinValidRangeM,
		SelfDetectionThresholdM: c.SelfDetectionThresholdM,
		MaxValidRangeM:          c.MaxValidRangeM,
	}); err != nil {
		return fmt.Errorf("diag: invalid config: %w", err)
	}
	return nil
}
