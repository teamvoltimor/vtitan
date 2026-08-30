package navigator

import (
	"math"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
)

// Config aggregates every tuning value the Navigator itself consumes --
// the union of what CoreNavigator and EscapeRecovery read off a
// NavigationTuning instance (self._tuning.clearance/speed/heading/pursuit/
// waypoints/control/lidar_sectors/escape/sign_router) plus the RobotSpecs-,
// TrackDimensions-, TrafficSignSpecs- and CompetitionSpecs-derived
// constants they read as module-level globals.
//
// Deliberately separate from controllers.Config and signrouter.Config
// rather than embedding them: those two are each a faithful mirror of what
// THEIR types consume, and the overlap (contact distance, the escape
// reverse speed, the sign activation distance) is a handful of fields the
// navigator reads for its own decisions. Navigator's constructor takes all
// three, so nothing is inferred from one to another.
type Config struct {
	// ContactDistM/SlowDistM/MediumDistM are the forward-clearance zone
	// bounds of the speed ladder (m), matching ClearanceZones.
	ContactDistM float64
	SlowDistM    float64
	MediumDistM  float64

	// MinMPS/MaxMPS/CreepMPS/SlowMPS/MediumMPS/FastMPS are the raw speed
	// tiers (m/s), matching SpeedControlParams' fields. Read them through
	// the accessors below, never directly: each is CLAMPED by
	// DrivetrainMaxSpeedMPS the way SpeedControlParams' *_mps() methods
	// clamp by RobotSpecs.MAX_SPEED_MPS.
	MinMPS    float64
	MaxMPS    float64
	CreepMPS  float64
	SlowMPS   float64
	MediumMPS float64
	FastMPS   float64
	// DrivetrainMaxSpeedMPS is RobotSpecs.MAX_SPEED_MPS -- a ceiling the
	// tiers clamp against, not a multiplier they scale by.
	DrivetrainMaxSpeedMPS float64

	// CrawlRad is the heading error (rad) at or above which speed drops to
	// the creep floor, matching HeadingErrorZones.CRAWL.
	CrawlRad float64

	// WallMarginSafetyM/MinLookaheadTransitionM parameterize the
	// crosstrack budget handed to WaypointController.SetCrosstrackBudget,
	// matching PurePursuitParams' fields of the same name.
	WallMarginSafetyM       float64
	MinLookaheadTransitionM float64
	// CornerPreviewDistanceM is the path distance previewed for an
	// upcoming turn (m), matching CORNER_PREVIEW_DISTANCE_M.
	CornerPreviewDistanceM float64

	// MainLoopReachedDistanceM is the distance below which this loop (as
	// opposed to WaypointController's own internal test) counts a waypoint
	// as reached (m), matching MAIN_LOOP_REACHED_DISTANCE_M.
	MainLoopReachedDistanceM float64
	// ReplanHeadingTieMarginM is ReplacePath's near-tie band (m), matching
	// REPLAN_HEADING_TIE_MARGIN_M.
	ReplanHeadingTieMarginM float64

	// ControlHz is the control loop's rate, matching CONTROL_HZ. Supplies
	// both WaypointController.ComputeSteering's dt and the frames-to-meters
	// conversion the reverse gates use.
	ControlHz float64

	// NoDataRangeM is the sentinel a sector with no valid rays reports,
	// matching NO_DATA_RANGE_M.
	NoDataRangeM float64

	// PoseTrailMinStepM/PoseTrailLen bound the breadcrumb trail a
	// retrace-reverse follows, matching POSE_TRAIL_MIN_STEP_M/
	// POSE_TRAIL_LEN.
	PoseTrailMinStepM float64
	PoseTrailLen      int
	// RevSpeed is the reverse speed during a stuck escape (m/s, negative),
	// matching REV_SPEED.
	RevSpeed float64
	// RevSteerDeg is the road-wheel angle held while reversing out (deg),
	// matching REV_STEER_DEG; see RevSteerNorm.
	RevSteerDeg float64
	// KTurnMinFrames/EscalateAfterAttempts/EscapeSideCommitAttempts/
	// MaxEscapeFrames/StuckEscalationFramesPerAttempt drive the escalating
	// stuck escape, matching the escape.toml fields of the same names.
	KTurnMinFrames                  int
	EscalateAfterAttempts           int
	EscapeSideCommitAttempts        int
	MaxEscapeFrames                 int
	StuckEscalationFramesPerAttempt int
	// StuckMoveThreshold is the displacement (m) that has to accumulate
	// before an escape sequence's counter resets, matching
	// STUCK_MOVE_THRESHOLD.
	StuckMoveThreshold float64

	// SignClearanceMarginM is the extra margin beyond the chassis and sign
	// half-widths the lane offset is built from, matching
	// SIGN_CLEARANCE_MARGIN_M.
	SignClearanceMarginM float64
	// ActivationDistM is the distance within which a sign counts as
	// "ahead" for the sign-aware lookahead, matching ACTIVATION_DIST_M.
	ActivationDistM float64
	// EscapeMaskRadiusM is how close a LIDAR return must land to a routed
	// sign to be withheld from the escape trigger, matching
	// ESCAPE_MASK_RADIUS_M.
	EscapeMaskRadiusM float64
	// SignLanePlanner/SignLaneSuppressDeform/SignLaneRampM/SignLaneHoldM/
	// SignLaneSplitOverlap/SignLaneSkipUnsatisfiable/SignLaneOffsetFrac/
	// SignLaneCornerEntryM/SignLaneCommitAheadM parameterize the pass-side
	// lane rebuild, matching the SIGN_LANE_* params.
	SignLanePlanner           bool
	SignLaneSuppressDeform    bool
	SignLaneRampM             float64
	SignLaneHoldM             float64
	SignLaneSplitOverlap      bool
	SignLaneSkipUnsatisfiable bool
	SignLaneOffsetFrac        float64
	SignLaneCornerEntryM      float64
	SignLaneCommitAheadM      float64
	// SignAwareLookahead/SignAwareSpeed/SignDeformSpeedThresholdM/
	// StaleTargetRescue are the Obstacles-only extensions to the ordinary
	// lookahead, speed and waypoint-advance pipelines.
	SignAwareLookahead        bool
	SignAwareSpeed            bool
	SignDeformSpeedThresholdM float64
	StaleTargetRescue         bool
	// RetraceEscape/RetraceDistM/RetraceSteerGainDeg parameterize the
	// retrace-reverse (see retraceSteer).
	RetraceEscape       bool
	RetraceDistM        float64
	RetraceSteerGainDeg float64
	// SignContactEvade/SignContactDistM/SignContactSteerDeg parameterize
	// the last-resort geometric guard against clipping a routed sign.
	SignContactEvade    bool
	SignContactDistM    float64
	SignContactSteerDeg float64

	// ChassisWidthM/MaxSteeringAngleRad/LidarToFrontBumperM/
	// LidarToRearBumperM are the RobotSpecs values this package reads
	// directly (see the Default* block).
	ChassisWidthM       float64
	MaxSteeringAngleRad float64
	LidarToFrontBumperM float64
	LidarToRearBumperM  float64

	// TrackMaxCoordM/CornerMinM/CornerMaxM/SignWidthM are the
	// TrackDimensions/TrafficSignSpecs values this package reads directly.
	TrackMaxCoordM float64
	CornerMinM     float64
	CornerMaxM     float64
	SignWidthM     float64
}

// Default* mirror the shipped TOML values this package's Python
// counterpart (CoreNavigator + EscapeRecovery, via NavigationTuning) reads:
// platform/shared/config/navigation/** plus platform/shared/config/robot.toml
// and track.toml. They are the fallback for a caller with no config root at
// all -- ConfigFor prefers the live files.
const (
	// DefaultContactDistM and the next two are the speed ladder's zone
	// bounds (motion/clearance.toml).
	DefaultContactDistM = 0.10
	DefaultSlowDistM    = 0.25
	DefaultMediumDistM  = 0.50

	// DefaultMinMPS and the following tiers are absolute m/s (motion/speed.toml).
	// They mirror the BASE file, not the active motor profile's overlay --
	// ConfigFor loads the overlay when hardware profile names are supplied.
	DefaultMinMPS    = 0.0499
	DefaultMaxMPS    = 0.156
	DefaultCreepMPS  = 0.1014
	DefaultSlowMPS   = 0.117
	DefaultMediumMPS = 0.1326
	DefaultFastMPS   = 0.156

	// DefaultDrivetrainMaxSpeedMPS is NOT in robot.toml's base file --
	// drivetrain.max_speed_mps is deliberately required from an active
	// hardware profile, so there is no true chassis-only default. 1.0 is
	// the value the currently shipped profile (rev-hd-hex-motor-6000rpm)
	// carries; used here only as the literal fallback for a caller with
	// no config root, exactly as controllers.DefaultMaxSteeringAngleRad
	// is. Every speed tier is CLAMPED by it (never scaled), matching
	// SpeedControlParams' *_mps() accessors.
	DefaultDrivetrainMaxSpeedMPS = 1.0

	// DefaultCrawlRad is the one surviving rung of a graduated ladder
	// (motion/heading.toml); see HeadingErrorZones.
	DefaultCrawlRad = 1.0

	// DefaultWallMarginSafetyM and the next two are the pursuit fields
	// CoreNavigator itself reads (motion/pursuit.toml), as opposed to those
	// WaypointController already owns.
	DefaultWallMarginSafetyM      = 0.03
	DefaultMinLookaheadTransition = 0.10
	DefaultCornerPreviewDistanceM = 0.80

	// DefaultMainLoopReachedDistanceM and the next are waypoint tuning
	// (waypoint/waypoints.toml).
	DefaultMainLoopReachedDistanceM = 0.20
	DefaultReplanHeadingTieMarginM  = 0.15

	// DefaultControlHz is the control loop rate (motion/control.toml).
	DefaultControlHz = 20.0

	// DefaultNoDataRangeM is the "no valid reading" sentinel
	// (sensors/lidar_sectors.toml) the stuck-escape branches read as
	// "assume clear".
	DefaultNoDataRangeM = 10.0

	// DefaultPoseTrailMinStepM and the following are the escape.toml fields
	// the core navigator's pose-trail retrace and escalating-escape logic
	// read, which controllers.Config deliberately does not mirror.
	DefaultPoseTrailMinStepM               = 0.01
	DefaultPoseTrailLen                    = 128
	DefaultRevSpeed                        = -0.20
	DefaultRevSteerDeg                     = 44.0
	DefaultKTurnMinFrames                  = 6
	DefaultEscalateAfterAttempts           = 3
	DefaultEscapeSideCommitAttempts        = 2
	DefaultMaxEscapeFrames                 = 20
	DefaultStuckEscalationFramesPerAttempt = 2
	DefaultStuckMoveThreshold              = 0.03

	// DefaultSignClearanceMarginM and the following are the sign-router
	// knobs (signs/sign_router.toml + SignRouterParams' Pydantic defaults
	// for the many that file never sets).
	DefaultSignClearanceMarginM      = 0.075
	DefaultActivationDistM           = 1.40
	DefaultEscapeMaskRadiusM         = 0.12
	DefaultSignLanePlanner           = true
	DefaultSignLaneSuppressDeform    = true
	DefaultSignLaneRampM             = 0.90
	DefaultSignLaneHoldM             = 0.25
	DefaultSignLaneSplitOverlap      = false
	DefaultSignLaneSkipUnsatisfiable = false
	DefaultSignLaneOffsetFrac        = 1.0
	DefaultSignLaneCornerEntryM      = 0.50
	DefaultSignLaneCommitAheadM      = 0.0
	DefaultSignAwareLookahead        = false
	DefaultSignAwareSpeed            = false
	DefaultSignDeformSpeedThresholdM = 0.02
	DefaultStaleTargetRescue         = false
	DefaultRetraceEscape             = false
	DefaultRetraceDistM              = 0.25
	DefaultRetraceSteerGainDeg       = 55.0
	DefaultSignContactEvade          = false
	DefaultSignContactDistM          = 0.60
	DefaultSignContactSteerDeg       = 19.25

	// DefaultChassisWidthM and the following are robot geometry
	// (robot.toml + the active hardware profile), the same values
	// controllers.Config sources; repeated here because this package reads
	// RobotSpecs directly too (chassis half-width for the crosstrack budget
	// and the sign-evade prediction, the steering limit for every
	// *_steer_norm conversion, the bumper offsets for the stuck-escape gate).
	DefaultChassisWidthM       = 0.194
	DefaultMaxSteeringAngleRad = 1.2252
	DefaultLidarToFrontBumperM = 0.30/2.0 - 0.1222
	DefaultLidarToRearBumperM  = 0.30/2.0 + 0.1222

	// DefaultTrackMaxCoordM and the following are track geometry
	// (track.toml).
	DefaultTrackMaxCoordM = 3.0
	DefaultCornerMinM     = 1.0
	DefaultCornerMaxM     = 2.0
	DefaultSignWidthM     = 0.05

	// DefaultOpenChallengeLaps matches competition_specs.toml's
	// open_challenge_laps (CompetitionSpecs.OPEN_CHALLENGE_LAPS), the
	// default num_laps CoreNavigator.__init__ takes.
	DefaultOpenChallengeLaps = 3

	// halfTurnDeg is a half-turn in degrees, the degrees-to-radians factor.
	halfTurnDeg = 180.0
)

// DefaultConfig returns the Config matching the shipped TOML defaults
// listed in the Default* block above.
func DefaultConfig() Config {
	return Config{
		ContactDistM: DefaultContactDistM,
		SlowDistM:    DefaultSlowDistM,
		MediumDistM:  DefaultMediumDistM,

		MinMPS:                DefaultMinMPS,
		MaxMPS:                DefaultMaxMPS,
		CreepMPS:              DefaultCreepMPS,
		SlowMPS:               DefaultSlowMPS,
		MediumMPS:             DefaultMediumMPS,
		FastMPS:               DefaultFastMPS,
		DrivetrainMaxSpeedMPS: DefaultDrivetrainMaxSpeedMPS,

		CrawlRad: DefaultCrawlRad,

		WallMarginSafetyM:       DefaultWallMarginSafetyM,
		MinLookaheadTransitionM: DefaultMinLookaheadTransition,
		CornerPreviewDistanceM:  DefaultCornerPreviewDistanceM,

		MainLoopReachedDistanceM: DefaultMainLoopReachedDistanceM,
		ReplanHeadingTieMarginM:  DefaultReplanHeadingTieMarginM,

		ControlHz:    DefaultControlHz,
		NoDataRangeM: DefaultNoDataRangeM,

		PoseTrailMinStepM:               DefaultPoseTrailMinStepM,
		PoseTrailLen:                    DefaultPoseTrailLen,
		RevSpeed:                        DefaultRevSpeed,
		RevSteerDeg:                     DefaultRevSteerDeg,
		KTurnMinFrames:                  DefaultKTurnMinFrames,
		EscalateAfterAttempts:           DefaultEscalateAfterAttempts,
		EscapeSideCommitAttempts:        DefaultEscapeSideCommitAttempts,
		MaxEscapeFrames:                 DefaultMaxEscapeFrames,
		StuckEscalationFramesPerAttempt: DefaultStuckEscalationFramesPerAttempt,
		StuckMoveThreshold:              DefaultStuckMoveThreshold,

		SignClearanceMarginM:      DefaultSignClearanceMarginM,
		ActivationDistM:           DefaultActivationDistM,
		EscapeMaskRadiusM:         DefaultEscapeMaskRadiusM,
		SignLanePlanner:           DefaultSignLanePlanner,
		SignLaneSuppressDeform:    DefaultSignLaneSuppressDeform,
		SignLaneRampM:             DefaultSignLaneRampM,
		SignLaneHoldM:             DefaultSignLaneHoldM,
		SignLaneSplitOverlap:      DefaultSignLaneSplitOverlap,
		SignLaneSkipUnsatisfiable: DefaultSignLaneSkipUnsatisfiable,
		SignLaneOffsetFrac:        DefaultSignLaneOffsetFrac,
		SignLaneCornerEntryM:      DefaultSignLaneCornerEntryM,
		SignLaneCommitAheadM:      DefaultSignLaneCommitAheadM,
		SignAwareLookahead:        DefaultSignAwareLookahead,
		SignAwareSpeed:            DefaultSignAwareSpeed,
		SignDeformSpeedThresholdM: DefaultSignDeformSpeedThresholdM,
		StaleTargetRescue:         DefaultStaleTargetRescue,
		RetraceEscape:             DefaultRetraceEscape,
		RetraceDistM:              DefaultRetraceDistM,
		RetraceSteerGainDeg:       DefaultRetraceSteerGainDeg,
		SignContactEvade:          DefaultSignContactEvade,
		SignContactDistM:          DefaultSignContactDistM,
		SignContactSteerDeg:       DefaultSignContactSteerDeg,

		ChassisWidthM:       DefaultChassisWidthM,
		MaxSteeringAngleRad: DefaultMaxSteeringAngleRad,
		LidarToFrontBumperM: DefaultLidarToFrontBumperM,
		LidarToRearBumperM:  DefaultLidarToRearBumperM,

		TrackMaxCoordM: DefaultTrackMaxCoordM,
		CornerMinM:     DefaultCornerMinM,
		CornerMaxM:     DefaultCornerMaxM,
		SignWidthM:     DefaultSignWidthM,
	}
}

// MinSpeedMPS is MIN_MPS clamped by the drivetrain ceiling, matching
// SpeedControlParams.min_mps().
func (c Config) MinSpeedMPS() float64 { return math.Min(c.MinMPS, c.DrivetrainMaxSpeedMPS) }

// MaxSpeedMPS is MAX_MPS clamped by the drivetrain ceiling, matching
// SpeedControlParams.max_mps().
func (c Config) MaxSpeedMPS() float64 { return math.Min(c.MaxMPS, c.DrivetrainMaxSpeedMPS) }

// CreepSpeedMPS is CREEP_MPS clamped by the drivetrain ceiling, matching
// SpeedControlParams.creep_mps().
func (c Config) CreepSpeedMPS() float64 { return math.Min(c.CreepMPS, c.DrivetrainMaxSpeedMPS) }

// SlowSpeedMPS is SLOW_MPS clamped by the drivetrain ceiling, matching
// SpeedControlParams.slow_mps().
func (c Config) SlowSpeedMPS() float64 { return math.Min(c.SlowMPS, c.DrivetrainMaxSpeedMPS) }

// MediumSpeedMPS is MEDIUM_MPS clamped by the drivetrain ceiling, matching
// SpeedControlParams.medium_mps().
func (c Config) MediumSpeedMPS() float64 { return math.Min(c.MediumMPS, c.DrivetrainMaxSpeedMPS) }

// FastSpeedMPS is FAST_MPS clamped by the drivetrain ceiling, matching
// SpeedControlParams.fast_mps().
func (c Config) FastSpeedMPS() float64 { return math.Min(c.FastMPS, c.DrivetrainMaxSpeedMPS) }

// RevSteerNorm converts RevSteerDeg into a normalised actuator command,
// matching EscapeManeuverParams.rev_steer_norm(): the stored value is a
// physical road-wheel angle, so a wider servo produces a SMALLER normalised
// command for the same angle rather than the same command meaning a wider
// angle on different hardware.
func (c Config) RevSteerNorm() float64 {
	return navutil.SteeringNormFromAngleRad(degreesToRadians(c.RevSteerDeg), c.MaxSteeringAngleRad)
}

// SignContactSteerNorm converts SignContactSteerDeg into a normalised
// actuator command, matching SignRouterParams.sign_contact_steer_norm().
func (c Config) SignContactSteerNorm() float64 {
	return navutil.SteeringNormFromAngleRad(degreesToRadians(c.SignContactSteerDeg), c.MaxSteeringAngleRad)
}

// RetraceSteerGainNorm is the reverse-pure-pursuit steering command for a
// target lateralOverDistance off-axis, matching
// SignRouterParams.retrace_steer_gain_norm(): the caller passes the
// dimensionless bearing ratio, the gain turns it into a road-wheel angle,
// and only then does the servo's reach enter.
func (c Config) RetraceSteerGainNorm(lateralOverDistance float64) float64 {
	return navutil.SteeringNormFromAngleRad(
		degreesToRadians(c.RetraceSteerGainDeg)*lateralOverDistance,
		c.MaxSteeringAngleRad,
	)
}

// LaneLateralOffsetM is the lateral distance a pass-side lane is laid at,
// matching _refresh_sign_lanes' own expression: (chassis half-diagonal +
// sign half-width + SIGN_CLEARANCE_MARGIN_M) * SIGN_LANE_OFFSET_FRAC.
// chassisHalfDiagonalM comes from the live signrouter.Config, which already
// derives it from robot.toml, rather than being recomputed here.
func (c Config) LaneLateralOffsetM(chassisHalfDiagonalM float64) float64 {
	return (chassisHalfDiagonalM + c.SignWidthM/2.0 + c.SignClearanceMarginM) * c.SignLaneOffsetFrac
}

// degreesToRadians converts an angle in degrees to radians.
func degreesToRadians(deg float64) float64 { return deg * math.Pi / halfTurnDeg }
