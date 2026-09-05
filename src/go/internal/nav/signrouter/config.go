package signrouter

import (
	"fmt"
	"math"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
)

// Config bundles every tuning- and geometry-derived value the routing,
// deformation and lane-planning functions in this package need, matching
// (merged, see doc.go) shared.config.navigation_tuning.SignRouterParams /
// src.navigation.planning.sign_router.config's SignRouterConfig and
// SignRouterConstants.
type Config struct {
	// CameraHFOVRad is the camera's horizontal field of view, from
	// robot.toml's [camera] hfov. The shipped sensor is a Raspberry Pi
	// Camera Module 3 Wide at 1.7802 rad (102 deg).
	CameraHFOVRad float64
	// CameraWidthPX is the sensor pixel width, from robot.toml's
	// [camera] width.
	CameraWidthPX float64
	// CameraFarClipM is the LIDAR-fusion validity ceiling, from
	// robot.toml's [camera] far_clip.
	CameraFarClipM float64
	// SensorMountXOffsetM is how far forward of the chassis centre the
	// camera/LIDAR sit, from robot.toml's [camera] mount_x_offset, so a
	// projection starts at the sensor rather than the body origin.
	SensorMountXOffsetM float64
	// SignHeightM is the real-world height of a WRO traffic sign, from
	// track.toml's [sign] height -- the only dimension the pinhole range
	// model needs.
	SignHeightM float64
	// LateralOffsetM is the lateral deformation magnitude (m), matching
	// SignRouterConfig.lateral_offset. Not a raw tunable: it is chassis
	// half-diagonal + sign half-width + SIGN_CLEARANCE_MARGIN_M, computed
	// by DefaultConfig/ConfigFor rather than restated as a literal (see
	// TestLateralOffsetTracksChassis in test_sign_router.py -- a 0.28mm
	// chassis-width change was enough to flip a corpus scenario).
	LateralOffsetM float64
	// ActivationDistM matches SignRouterConfig.activation_dist: deformation
	// activates when the robot is within this distance of a sign (m).
	ActivationDistM float64
	// PassedDistM matches SignRouterConfig.passed_dist: a sign is marked
	// passed once the robot moves further than this from it (m). MUST stay
	// above ActivationDistM -- see NewConfig.
	PassedDistM float64
	// DepthPin matches SignRouterConfig.depth_pin: hold the commanded point
	// abeam the sign instead of letting it recede.
	DepthPin bool
	// DetectionMatchDistM matches SignRouterConfig.detection_match_dist.
	DetectionMatchDistM float64
	// MinConfidence matches SignRouterConfig.min_confidence.
	MinConfidence float64
	// CommitHysteresis matches SignRouterConfig.commit_hysteresis.
	CommitHysteresis bool
	// CorridorFlipTicks matches SignRouterConfig.corridor_flip_ticks.
	CorridorFlipTicks int
	// SettleTicks matches SignRouterConfig.settle_ticks.
	SettleTicks int
	// RelabelUnsatisfiable matches
	// SignRouterParams.SIGN_LANE_RELABEL_UNSATISFIABLE: move a sign to the
	// corner's other face when this one's clamped lane target lands on the
	// forbidden side of it.
	RelabelUnsatisfiable bool
	// DepthConsistentCorridor matches
	// SignRouterParams.SIGN_LANE_DEPTH_CONSISTENT_CORRIDOR: resolve a
	// corner sign by which face its depth lies along, not which is nearest.
	DepthConsistentCorridor bool

	// WallClearanceMarginM matches SignRouterConstants.wall_clearance_margin_m.
	WallClearanceMarginM float64
	// DeformDepthBufferM matches SignRouterConstants.deform_depth_buffer_m.
	DeformDepthBufferM float64
	// PinCornerGuard matches SignRouterConstants.pin_corner_guard.
	PinCornerGuard bool
	// PinHeadingGuard matches SignRouterConstants.pin_heading_guard.
	PinHeadingGuard bool
	// PinHeadingGuardRad matches SignRouterConstants.pin_heading_guard_rad
	// (SignRouterParams.PIN_HEADING_GUARD_DEG converted once, in radians).
	PinHeadingGuardRad float64

	// TrackMinCoordM/TrackMaxCoordM/TrackCornerMinM/TrackCornerMaxM mirror
	// shared.config.constants.TrackDimensions.MIN_COORD/MAX_COORD/
	// CORNER_MIN/CORNER_MAX, which routing.py's pure functions read as
	// module globals. Threaded through Config here instead: this package
	// has no generated-constants file to source them from the way
	// TrackDimensions sources them from track.toml, and passing them
	// explicitly is the same choice internal/nav/waypoints.CorridorForPosition
	// already made for the same numbers.
	TrackMinCoordM  float64
	TrackMaxCoordM  float64
	TrackCornerMinM float64
	TrackCornerMaxM float64

	// ChassisHalfDiagonalM matches geometry.chassis_half_diagonal_m().
	ChassisHalfDiagonalM float64
	// BehindToleranceM matches geometry.behind_tolerance_m(), and replaces
	// router.py's package-level BEHIND_TOLERANCE (computed once at import
	// time there; computed once per Config here for the same reason
	// ChassisHalfDiagonalM is -- see LateralOffsetM's comment).
	BehindToleranceM float64
}

// Default* mirror the shipped literal defaults this package's Python
// counterpart is built from: SignRouterParams' Pydantic field defaults for
// the tuning knobs, and the CURRENT platform/config/robot.toml
// [chassis] / track.toml [track]/[sign] values for geometry. Like
// internal/nav/waypoints.DefaultConfig, these are a fallback for when no
// config root is available -- ConfigFor prefers the live TOML values, which
// is required reading for LateralOffsetM/ChassisHalfDiagonalM/
// BehindToleranceM in particular (see their doc comments).
const (
	DefaultSignClearanceMarginM = 0.075
	DefaultWallClearanceMarginM = 0.04
	DefaultDeformDepthBufferM   = 0.5
	DefaultActivationDistM      = 1.40
	DefaultPassedDistM          = 1.60
	DefaultDetectionMatchDistM  = 0.30
	DefaultMinConfidence        = 0.25
	DefaultSettleTicks          = 150
	DefaultCommitHysteresis     = false
	DefaultCorridorFlipTicks    = 1
	DefaultDepthPin             = true
	DefaultPinCornerGuard       = true
	DefaultPinHeadingGuard      = true
	DefaultPinHeadingGuardDeg   = 35.0
	// DefaultCameraHFOVRad/DefaultCameraWidthPX/DefaultCameraFarClipM/
	// DefaultSensorMountXOffsetM mirror robot.toml's [camera] section, and
	// DefaultSignHeightM mirrors track.toml's [sign] height. They are the
	// fallback for a run with no config root, not a second source of truth.
	DefaultCameraHFOVRad       = 1.7802
	DefaultCameraWidthPX       = 1536.0
	DefaultCameraFarClipM      = 10.0
	DefaultSensorMountXOffsetM = 0.1222
	DefaultSignHeightM         = 0.10

	DefaultRelabelUnsatisfiable = true
	DefaultDepthConsistent      = true
	DefaultTrackMinCoordM       = 0.0
	DefaultTrackMaxCoordM       = 3.0
	DefaultTrackCornerMinM      = 1.0
	DefaultTrackCornerMaxM      = 2.0
	// DefaultChassisLengthM/DefaultChassisWidthM mirror robot.toml's
	// [chassis] length/width -- 0.30/0.194 as of this port. See
	// ChassisHalfDiagonalM's doc comment for why ConfigFor re-derives this
	// from the live file rather than trusting these to stay current.
	DefaultChassisLengthM = 0.30
	DefaultChassisWidthM  = 0.194
	// DefaultSignWidthM mirrors track.toml's [sign] width.
	DefaultSignWidthM = 0.05
)

// chassisHalfDiagonalM matches geometry.chassis_half_diagonal_m(): half the
// chassis diagonal, the clearance radius while mid-turn. Sized on the
// diagonal rather than the half-width because a robot still turning
// presents its corner, not its side -- see the Python docstring for the
// measured collision-count justification.
func chassisHalfDiagonalM(lengthM, widthM float64) float64 {
	return math.Hypot(lengthM/2, widthM/2)
}

// behindToleranceM matches geometry.behind_tolerance_m(): half the chassis
// length, so an object level with the rear bumper still counts as
// alongside rather than cleared.
func behindToleranceM(lengthM float64) float64 {
	return lengthM / 2
}

// DefaultConfig returns the Config matching the Default* literals above.
func DefaultConfig() Config {
	chassisHalfDiagonal := chassisHalfDiagonalM(DefaultChassisLengthM, DefaultChassisWidthM)
	return Config{
		LateralOffsetM:          chassisHalfDiagonal + DefaultSignWidthM/2 + DefaultSignClearanceMarginM,
		ActivationDistM:         DefaultActivationDistM,
		PassedDistM:             DefaultPassedDistM,
		DepthPin:                DefaultDepthPin,
		DetectionMatchDistM:     DefaultDetectionMatchDistM,
		MinConfidence:           DefaultMinConfidence,
		CommitHysteresis:        DefaultCommitHysteresis,
		CorridorFlipTicks:       DefaultCorridorFlipTicks,
		SettleTicks:             DefaultSettleTicks,
		CameraHFOVRad:           DefaultCameraHFOVRad,
		CameraWidthPX:           DefaultCameraWidthPX,
		CameraFarClipM:          DefaultCameraFarClipM,
		SensorMountXOffsetM:     DefaultSensorMountXOffsetM,
		SignHeightM:             DefaultSignHeightM,
		RelabelUnsatisfiable:    DefaultRelabelUnsatisfiable,
		DepthConsistentCorridor: DefaultDepthConsistent,
		WallClearanceMarginM:    DefaultWallClearanceMarginM,
		DeformDepthBufferM:      DefaultDeformDepthBufferM,
		PinCornerGuard:          DefaultPinCornerGuard,
		PinHeadingGuard:         DefaultPinHeadingGuard,
		PinHeadingGuardRad:      DefaultPinHeadingGuardDeg * math.Pi / navutil.DegreesPerHalfTurn,
		TrackMinCoordM:          DefaultTrackMinCoordM,
		TrackMaxCoordM:          DefaultTrackMaxCoordM,
		TrackCornerMinM:         DefaultTrackCornerMinM,
		TrackCornerMaxM:         DefaultTrackCornerMaxM,
		ChassisHalfDiagonalM:    chassisHalfDiagonal,
		BehindToleranceM:        behindToleranceM(DefaultChassisLengthM),
	}
}

// NewConfig validates cfg and returns it unchanged, matching
// SignRouterConfig.__post_init__'s activation_dist/passed_dist check (the
// lateral_offset-defaulting half of __post_init__ has no Go equivalent --
// callers always supply LateralOffsetM explicitly, via DefaultConfig or
// ConfigFor).
//
// activation_dist must stay below passed_dist: _active_sign_candidates
// engages a sign nearer than activation_dist and retires one further than
// passed_dist, in that order, on the same tick. Invert them and every sign
// is engaged and marked passed in the same breath, from a meter away, and
// stays retired for the rest of the run -- deformation never fires at the
// real pass. Measured on the 256-scenario corpus: activation_dist=1.30
// against passed_dist=1.20 took it from 209 collisions to 256/256 with zero
// laps completed, silently. That is the worst shape a config error can take
// in a safety path, so it is an error rather than a clamp.
func NewConfig(cfg Config) (Config, error) {
	if cfg.ActivationDistM >= cfg.PassedDistM {
		return Config{}, fmt.Errorf(
			"signrouter: activation_dist (%v) must be < passed_dist (%v): a sign would be engaged "+
				"and marked passed on the same tick, permanently retiring it before its real pass "+
				"and disabling sign avoidance for the whole run",
			cfg.ActivationDistM, cfg.PassedDistM,
		)
	}
	return cfg, nil
}
