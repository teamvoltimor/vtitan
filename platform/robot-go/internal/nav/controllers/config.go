package controllers

import (
	"log/slog"
	"math"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
)

// Config aggregates every tuning value internal/nav/controllers' types
// consume, mirroring what CollisionAvoidanceController.from_tuning/
// WaypointController.from_tuning/StuckDetector.from_tuning each pull from a
// NavigationTuning instance -- plus the RobotSpecs-derived physical
// constants (wheelbase, chassis width, max steering angle, LIDAR mount
// offsets/range) those from_tuning constructors also read directly off
// RobotSpecs rather than the tuning object. DefaultConfig's field values
// mirror the shipped TOML defaults (platform/shared/config/navigation/**
// and platform/shared/config/robot.toml); ConfigFor loads the real values
// via internal/config/profile, falling back to DefaultConfig's literals
// when no config root is supplied or loading fails.
type Config struct {
	// Clearance (motion/clearance.toml).
	ContactDist float64
	// ObstaclesContactDist supersedes ContactDist on the Obstacles Challenge;
	// nil leaves it alone. See ForObstaclesChallenge.
	ObstaclesContactDist *float64
	SlowDist             float64
	FastDist             float64
	PathMargin           float64

	// Control (motion/control.toml).
	ControlHz float64

	// Pursuit (motion/pursuit.toml).
	LookaheadShort         float64
	LookaheadLong          float64
	LookaheadTransition    float64
	LookaheadBlendStart    float64
	SteerKp                float64
	MaxSteeringRate        float64
	CornerTurnThresholdRad float64

	// Lidar sectors (sensors/lidar_sectors.toml).
	FrontHalfFovDeg         float64
	ThreatHalfFovDeg        float64
	SelfDetectionThresholdM float64
	MinValidRangeM          float64
	ThreatNoDetectionRangeM float64
	NoDataRangeM            float64
	BlindWedgeLeftMinDeg    float64
	BlindWedgeLeftMaxDeg    float64
	BlindWedgeRightMinDeg   float64
	BlindWedgeRightMaxDeg   float64

	// Escape (escape/escape.toml -- subset StuckDetector/
	// CollisionAvoidanceController's from_tuning actually read).
	RevSpeed                float64
	RevSteerDeg             float64
	KTurnMinFrames          int
	KTurnMaxFrames          int
	StuckMoveThreshold      float64
	StuckTimeoutFrames      int
	SideCorrectionSteerDeg  float64
	SideCorrectionSpeed     float64
	SideCorrectionFrames    int
	StuckConfirmationChecks int
	StuckHistoryFloor       int
	MinHistoryForDistance   int

	// Waypoints (waypoint/waypoints.toml -- the one field this package
	// consumes; see profile.WaypointsConfig.ControllerReachedDistanceM).
	ControllerReachedDistanceM float64

	// RobotSpecs-derived physical constants (robot.toml + active hardware
	// profile).
	WheelbaseM          float64
	ChassisWidthM       float64
	MaxSteeringAngleRad float64
	LidarToFrontBumperM float64
	LidarToRearBumperM  float64
	LidarMaxRangeM      float64
}

// Default* match the shipped TOML values (platform/shared/config/
// navigation/** and platform/shared/config/robot.toml) as of this port.
const (
	DefaultContactDist = 0.10
	DefaultSlowDist    = 0.25
	DefaultFastDist    = 1.00
	DefaultPathMargin  = 0.10

	DefaultControlHz = 20.0

	DefaultLookaheadShort         = 0.16
	DefaultLookaheadLong          = 0.32
	DefaultLookaheadTransition    = 0.30
	DefaultLookaheadBlendStart    = 0.70
	DefaultSteerKp                = 1.2
	DefaultMaxSteeringRate        = 1.2
	DefaultCornerTurnThresholdRad = 0.35

	DefaultFrontHalfFovDeg         = 30.0
	DefaultThreatHalfFovDeg        = 45.0
	DefaultSelfDetectionThresholdM = 0.08
	DefaultMinValidRangeM          = 0.05
	DefaultThreatNoDetectionRangeM = 1.0
	DefaultNoDataRangeM            = 10.0
	DefaultBlindWedgeLeftMinDeg    = -155.0
	DefaultBlindWedgeLeftMaxDeg    = -120.0
	DefaultBlindWedgeRightMinDeg   = 120.0
	DefaultBlindWedgeRightMaxDeg   = 160.0

	DefaultRevSpeed                = -0.20
	DefaultRevSteerDeg             = 44.0
	DefaultKTurnMinFrames          = 6
	DefaultKTurnMaxFrames          = 12
	DefaultStuckMoveThreshold      = 0.03
	DefaultStuckTimeoutFrames      = 40
	DefaultSideCorrectionSteerDeg  = 16.5
	DefaultSideCorrectionSpeed     = 0.1
	DefaultSideCorrectionFrames    = 4
	DefaultStuckConfirmationChecks = 3
	DefaultStuckHistoryFloor       = 60
	DefaultMinHistoryForDistance   = 2

	DefaultControllerReachedDistanceM = 0.01

	// DefaultWheelbaseM/DefaultChassisWidthM/DefaultLidarToFrontBumperM/
	// DefaultLidarToRearBumperM/DefaultLidarMaxRangeM match robot.toml's
	// base [chassis]/[ackermann]/[lidar] sections (no hardware profile
	// needed -- these are true of the chassis regardless of which motor/
	// servo drives it).
	DefaultWheelbaseM          = 0.19
	DefaultChassisWidthM       = 0.194
	DefaultLidarToFrontBumperM = 0.30/2.0 - 0.1222
	DefaultLidarToRearBumperM  = 0.30/2.0 + 0.1222
	DefaultLidarMaxRangeM      = 12.0

	// DefaultMaxSteeringAngleRad is NOT in robot.toml's base file --
	// steering.max_wheel_angle_deg is deliberately required from an
	// active hardware profile (see profile.RobotConfig's doc comment), so
	// there is no true chassis-only default. 1.2252 rad (~70.2 deg) is
	// cited in waypoints.toml's arc_radius comment as the measured value
	// this codebase currently ships with; used here only as the literal
	// fallback for a caller with no config root at all (see ConfigFor,
	// which loads the real, profile-sourced value whenever a hardware
	// profile is available).
	DefaultMaxSteeringAngleRad = 1.2252
)

// DefaultConfig returns the Config matching the shipped TOML/robot.toml
// defaults.
func DefaultConfig() Config {
	return Config{
		ContactDist: DefaultContactDist,
		SlowDist:    DefaultSlowDist,
		FastDist:    DefaultFastDist,
		PathMargin:  DefaultPathMargin,

		ControlHz: DefaultControlHz,

		LookaheadShort:         DefaultLookaheadShort,
		LookaheadLong:          DefaultLookaheadLong,
		LookaheadTransition:    DefaultLookaheadTransition,
		LookaheadBlendStart:    DefaultLookaheadBlendStart,
		SteerKp:                DefaultSteerKp,
		MaxSteeringRate:        DefaultMaxSteeringRate,
		CornerTurnThresholdRad: DefaultCornerTurnThresholdRad,

		FrontHalfFovDeg:         DefaultFrontHalfFovDeg,
		ThreatHalfFovDeg:        DefaultThreatHalfFovDeg,
		SelfDetectionThresholdM: DefaultSelfDetectionThresholdM,
		MinValidRangeM:          DefaultMinValidRangeM,
		ThreatNoDetectionRangeM: DefaultThreatNoDetectionRangeM,
		NoDataRangeM:            DefaultNoDataRangeM,
		BlindWedgeLeftMinDeg:    DefaultBlindWedgeLeftMinDeg,
		BlindWedgeLeftMaxDeg:    DefaultBlindWedgeLeftMaxDeg,
		BlindWedgeRightMinDeg:   DefaultBlindWedgeRightMinDeg,
		BlindWedgeRightMaxDeg:   DefaultBlindWedgeRightMaxDeg,

		RevSpeed:                DefaultRevSpeed,
		RevSteerDeg:             DefaultRevSteerDeg,
		KTurnMinFrames:          DefaultKTurnMinFrames,
		KTurnMaxFrames:          DefaultKTurnMaxFrames,
		StuckMoveThreshold:      DefaultStuckMoveThreshold,
		StuckTimeoutFrames:      DefaultStuckTimeoutFrames,
		SideCorrectionSteerDeg:  DefaultSideCorrectionSteerDeg,
		SideCorrectionSpeed:     DefaultSideCorrectionSpeed,
		SideCorrectionFrames:    DefaultSideCorrectionFrames,
		StuckConfirmationChecks: DefaultStuckConfirmationChecks,
		StuckHistoryFloor:       DefaultStuckHistoryFloor,
		MinHistoryForDistance:   DefaultMinHistoryForDistance,

		ControllerReachedDistanceM: DefaultControllerReachedDistanceM,

		WheelbaseM:          DefaultWheelbaseM,
		ChassisWidthM:       DefaultChassisWidthM,
		MaxSteeringAngleRad: DefaultMaxSteeringAngleRad,
		LidarToFrontBumperM: DefaultLidarToFrontBumperM,
		LidarToRearBumperM:  DefaultLidarToRearBumperM,
		LidarMaxRangeM:      DefaultLidarMaxRangeM,
	}
}

// ForObstaclesChallenge returns c as the Obstacles Challenge should run it,
// mirroring ClearanceZones.for_obstacles_challenge: ContactDist replaced by
// ObstaclesContactDist when one is set, and c unchanged when it is not, so
// the Open path and an un-overridden Obstacles path stay byte-identical.
//
// The two challenges present different things to escape FROM, which is why
// the zone is per-challenge at all: a wall 0.10 m ahead in Open is a genuine
// emergency, while Obstacles additionally has signs the router deliberately
// routes PAST at ~0.175 m, so the shared value fires on geometry the planner
// chose on purpose.
func (c Config) ForObstaclesChallenge() Config {
	if c.ObstaclesContactDist == nil {
		return c
	}
	c.ContactDist = *c.ObstaclesContactDist
	return c
}

// NewCollisionAvoidanceController builds a CollisionAvoidanceController
// from c, matching CollisionAvoidanceController.from_tuning.
func (c Config) NewCollisionAvoidanceController() *CollisionAvoidanceController {
	pathHalfWidth := c.ChassisWidthM/navutil.Half + c.PathMargin
	return &CollisionAvoidanceController{
		ContactDist:    c.ContactDist,
		SlowDist:       c.SlowDist,
		FastDist:       c.FastDist,
		EscapeRevSpeed: c.RevSpeed,
		EscapeSteerScale: navutil.SteeringNormFromAngleRad(
			c.RevSteerDeg*math.Pi/navutil.DegreesPerHalfTurn,
			c.MaxSteeringAngleRad,
		),
		StuckThreshold: c.StuckMoveThreshold,
		PathHalfWidth:  pathHalfWidth,
		KTurnMinFrames: c.KTurnMinFrames,
		KTurnMaxFrames: c.KTurnMaxFrames,
		SideCorrectionSteer: navutil.SteeringNormFromAngleRad(
			c.SideCorrectionSteerDeg*math.Pi/navutil.DegreesPerHalfTurn,
			c.MaxSteeringAngleRad,
		),
		SideCorrectionSpeed:     c.SideCorrectionSpeed,
		SideCorrectionFrames:    c.SideCorrectionFrames,
		FrontHalfFovRad:         c.FrontHalfFovDeg * math.Pi / navutil.DegreesPerHalfTurn,
		ThreatHalfFovRad:        c.ThreatHalfFovDeg * math.Pi / navutil.DegreesPerHalfTurn,
		Geometry:                c.sectorGeometry(),
		ThreatNoDetectionRangeM: c.ThreatNoDetectionRangeM,
		LidarToFrontBumperM:     c.LidarToFrontBumperM,
	}
}

// NewWaypointController builds a WaypointController from c, matching
// WaypointController.from_tuning.
func (c Config) NewWaypointController() *WaypointController {
	return NewWaypointController(
		c.MaxSteeringAngleRad,
		c.WheelbaseM,
		c.LookaheadShort,
		c.LookaheadLong,
		c.LookaheadTransition,
		c.MaxSteeringRate,
		c.ControllerReachedDistanceM,
		c.CornerTurnThresholdRad,
	)
}

// NewStuckDetector builds a StuckDetector from c, matching
// StuckDetector.from_tuning. A nil logger falls back to slog.Default().
func (c Config) NewStuckDetector(logger *slog.Logger) (*StuckDetector, error) {
	const historyDoubling = 2
	historySize := max(c.StuckTimeoutFrames*historyDoubling, c.StuckHistoryFloor)
	return NewStuckDetector(
		c.StuckMoveThreshold,
		c.StuckTimeoutFrames,
		historySize,
		c.StuckConfirmationChecks,
		c.MinHistoryForDistance,
		logger,
	)
}

// sectorGeometry builds the SectorGeometry shared by every sector query
// from this Config, matching how CollisionAvoidanceController.__init__
// converts its degree-unit lidar_sectors parameters to radians once at
// construction.
func (c Config) sectorGeometry() SectorGeometry {
	return SectorGeometry{
		SelfDetectionThresholdM: c.SelfDetectionThresholdM,
		MinValidRangeM:          c.MinValidRangeM,
		NoDataRangeM:            c.NoDataRangeM,
		LidarMaxRangeM:          c.LidarMaxRangeM,
		BlindWedgeLeftMinRad:    c.BlindWedgeLeftMinDeg * math.Pi / navutil.DegreesPerHalfTurn,
		BlindWedgeLeftMaxRad:    c.BlindWedgeLeftMaxDeg * math.Pi / navutil.DegreesPerHalfTurn,
		BlindWedgeRightMinRad:   c.BlindWedgeRightMinDeg * math.Pi / navutil.DegreesPerHalfTurn,
		BlindWedgeRightMaxRad:   c.BlindWedgeRightMaxDeg * math.Pi / navutil.DegreesPerHalfTurn,
	}
}
