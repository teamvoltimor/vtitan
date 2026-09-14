package navigator

import (
	"log/slog"
	"os"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/escape"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/motion"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/sensors"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/signs"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/waypoint"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
)

// speedTOMLPath/headingTOMLPath/speedProfilesDir are declared here rather
// than in internal/config/profile because neither file has a profile mirror
// there yet, and speed.toml's overlay layout is unlike every path that
// package already models: its per-motor overlay lives at
// src/config/profiles/<name>/motion/speed.toml, NOT at
// <dir-of-base>/profiles/<name>/<base-name>, which is the only shape
// profile.Load's own merge understands (that shape is what robot.toml
// uses). loadSpeedConfig below does the overlay walk itself for that
// reason.
//
// Every source below decodes into the generated DTO for its TOML: the
// navigator reads the same fields the other consumers do, so a separate
// hand-written mirror only risked drifting from the schema.

const (
	speedTOMLPath   = "src/config/navigation/motion/speed.toml"
	headingTOMLPath = "src/config/navigation/motion/heading.toml"
	// speedProfilesDir is the directory holding each hardware profile's
	// overlay tree, relative to the repo root.
	speedProfilesDir = "src/config/profiles"
	// speedProfileRelPath is where a profile's speed overlay sits inside
	// its own directory.
	speedProfileRelPath = "motion/speed.toml"
)

// loadApplyTOML is the profile-less call convention every navigator source
// shares; the load-warn-apply logic itself lives in profile.Apply. The label
// argument is retained at the call sites but no longer used, since Apply logs
// the failing path directly.
func loadApplyTOML[T any](logger *slog.Logger, path, _ string, apply func(T)) {
	profile.Apply[T](logger, path, nil, apply)
}

// ConfigFor resolves the Config to run with: DefaultConfig's literals,
// overlaid with each source TOML file (clearance/speed/heading/pursuit/
// waypoints/control/lidar_sectors/escape/sign_router/robot/track, each
// loaded independently from <configRoot>/<its own path>) plus the active
// hardwareProfileNames for the RobotSpecs-derived fields and the per-motor
// speed ladder, if configRoot is non-empty and loading succeeds;
// otherwise, or on any load failure, the literal defaults, logging why.
// Each file loads and falls back independently, since they're unrelated
// failure domains -- the same contract controllers.ConfigFor and
// signrouter.ConfigFor state.
func ConfigFor(logger *slog.Logger, configRoot string, hardwareProfileNames []string) Config {
	cfg := DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	loadApplyTOML(
		logger, filepath.Join(configRoot, profile.DefaultClearanceTOMLPath), "clearance.toml",
		func(loaded profile.ClearanceConfig) {
			cfg.ContactDistM = loaded.ContactDist
			cfg.SlowDistM = loaded.SlowDist
			cfg.MediumDistM = loaded.MediumDist
			cfg.ForwardNoDataIsDegraded = loaded.ForwardNoDataIsDegraded
		})

	if speed, err := loadSpeedConfig(configRoot, hardwareProfileNames); err != nil {
		logger.Warn("navigator: loading speed.toml, falling back to defaults", "error", err)
	} else {
		cfg.MinMPS = speed.MinMps
		cfg.MaxMPS = speed.MaxMps
		cfg.CreepMPS = speed.CreepMps
		cfg.SlowMPS = speed.SlowMps
		cfg.MediumMPS = speed.MediumMps
		cfg.FastMPS = speed.FastMps
		cfg.Open = ChallengeTiers{
			MaxMPS:    speed.OpenMaxMps,
			SlowMPS:   speed.OpenSlowMps,
			MediumMPS: speed.OpenMediumMps,
			FastMPS:   speed.OpenFastMps,
		}
		cfg.Obstacles = ChallengeTiers{
			MaxMPS:    speed.ObstaclesMaxMps,
			SlowMPS:   speed.ObstaclesSlowMps,
			MediumMPS: speed.ObstaclesMediumMps,
			FastMPS:   speed.ObstaclesFastMps,
		}
		if tierErr := cfg.validateChallengeTiers(); tierErr != nil {
			logger.Warn("navigator: per-challenge speed tiers rejected, dropping them", "error", tierErr)
			cfg.Open, cfg.Obstacles = ChallengeTiers{}, ChallengeTiers{}
		}
	}

	loadApplyTOML(
		logger,
		filepath.Join(configRoot, headingTOMLPath),
		"heading.toml",
		func(loaded motion.NavigationMotionHeading) {
			cfg.CrawlRad = loaded.Crawl
		},
	)

	loadApplyTOML(
		logger, filepath.Join(configRoot, profile.DefaultPursuitTOMLPath), "pursuit.toml",
		func(loaded motion.NavigationMotionPursuit) {
			cfg.WallMarginSafetyM = loaded.WallMarginSafetyM
			cfg.MinLookaheadTransitionM = loaded.MinLookaheadTransitionM
			cfg.CornerPreviewDistanceM = loaded.CornerPreviewDistanceM
		})

	loadApplyTOML(
		logger,
		filepath.Join(configRoot, profile.DefaultWaypointsTOMLPath),
		"waypoints.toml",
		func(loaded waypoint.NavigationWaypointWaypoints) {
			cfg.FirstLapCornerCaution = loaded.FirstLapCornerCaution
			cfg.MainLoopReachedDistanceM = loaded.MainLoopReachedDistanceM
			cfg.ReplanHeadingTieMarginM = loaded.ReplanHeadingTieMarginM
			cfg.ParkEngageDistM = loaded.ArcRadius
			cfg.FinishApproachM = loaded.FinishApproachM
		},
	)

	loadApplyTOML(
		logger,
		filepath.Join(configRoot, profile.DefaultControlTOMLPath),
		"control.toml",
		func(loaded motion.NavigationMotionControl) {
			cfg.ControlHz = loaded.ControlHz
		},
	)

	loadApplyTOML(
		logger,
		filepath.Join(configRoot, profile.DefaultLidarSectorsTOMLPath),
		"lidar_sectors.toml",
		func(loaded sensors.NavigationSensorsLidarSectors) {
			cfg.NoDataRangeM = loaded.NoDataRangeM
		},
	)

	loadApplyTOML(
		logger,
		filepath.Join(configRoot, profile.DefaultEscapeTOMLPath),
		"escape.toml",
		func(loaded escape.NavigationEscapeEscape) {
			applyEscapeTOML(&cfg, loaded)
		},
	)

	loadApplyTOML(
		logger,
		filepath.Join(configRoot, profile.DefaultSignRouterTOMLPath),
		"sign_router.toml",
		func(loaded signs.NavigationSignsSignRouter) {
			applySignRouterTOML(&cfg, loaded)
		},
	)

	// robot.toml goes through LoadRobotConfig, not the generic loadApplyTOML
	// above, for two reasons that both bite silently.
	//
	// It must see hardwareProfileNames: max_speed_mps and max_wheel_angle_deg
	// are DELIBERATELY ABSENT from the base file (they describe a specific
	// motor and servo) and only a profile supplies them.
	//
	// And it must be the checked variant: every field here is applied
	// unconditionally, so a missing key lands as 0 rather than leaving the
	// default in place -- and DrivetrainMaxSpeedMPS=0 makes every
	// Config.*SpeedMPS() method return min(tier, 0), i.e. a robot that
	// cannot move at all. Measured: a native corpus sweep given a config
	// root but no profiles scored 640/640 STUCK at max speed 0.000, which
	// reads as a navigation failure rather than as the config error it is.
	// LoadRobotConfig fails loudly on exactly those keys instead.
	robotPath := filepath.Join(configRoot, profile.DefaultRobotTOMLPath)
	if loaded, err := profile.LoadRobotConfig(robotPath, hardwareProfileNames); err != nil {
		logger.Warn("navigator: loading robot.toml, falling back to defaults", "error", err)
	} else {
		cfg.ChassisWidthM = loaded.Chassis.Width
		cfg.MaxSteeringAngleRad = loaded.MaxSteeringAngle()
		cfg.LidarToFrontBumperM = loaded.LidarToFrontBumper()
		cfg.LidarToRearBumperM = loaded.LidarToRearBumper()
		cfg.DrivetrainMaxSpeedMPS = loaded.Drivetrain.MaxSpeedMPS
	}

	loadApplyTOML(
		logger,
		filepath.Join(configRoot, profile.DefaultTrackTOMLPath),
		"track.toml",
		func(loaded generated.TrackConfig) {
			cfg.TrackMaxCoordM = loaded.Track.MaxCoord
			cfg.CornerMinM = loaded.Track.CornerMin
			cfg.CornerMaxM = loaded.Track.CornerMax
			cfg.SignWidthM = loaded.Sign.Width
		},
	)

	return cfg
}

// applyEscapeTOML copies loaded escape.toml values onto cfg. Split out so
// ConfigFor stays a readable sequence of independent load-or-fall-back
// blocks rather than one function dominated by field assignments.
func applyEscapeTOML(cfg *Config, loaded escape.NavigationEscapeEscape) {
	cfg.PoseTrailMinStepM = loaded.PoseTrailMinStepM
	cfg.PoseTrailLen = loaded.PoseTrailLen
	cfg.RevSpeed = loaded.RevSpeed
	cfg.RevSteerDeg = loaded.RevSteerDeg
	cfg.KTurnMinFrames = profile.Frames(loaded.KTurnMinS, cfg.ControlHz)
	cfg.EscalateAfterAttempts = loaded.EscalateAfterAttempts
	cfg.EscapeSideCommitAttempts = loaded.EscapeSideCommitAttempts
	cfg.MaxEscapeFrames = profile.Frames(loaded.MaxEscapeS, cfg.ControlHz)
	cfg.StuckEscalationFramesPerAttempt = profile.Frames(loaded.StuckEscalationPerAttemptS, cfg.ControlHz)
	cfg.StuckMoveThreshold = loaded.StuckMoveThreshold
	cfg.KTurnFitRearGap = loaded.KTurnFitRearGap
	// The generated DTO carries this as a plain bool, so a successful
	// escape.toml load always yields a concrete override. The shipped file
	// sets it, and DefaultConfig's KTurnFitRearGap is false, so an omitted
	// key resolving to false lands on the same resolved value the old
	// nil-means-leave-alone pointer produced.
	cfg.ObstaclesKTurnFitRearGap = &loaded.ObstaclesKTurnFitRearGap
}

// applySignRouterTOML copies loaded sign_router.toml values onto cfg, for
// the same reason applyEscapeTOML exists.
func applySignRouterTOML(cfg *Config, loaded signs.NavigationSignsSignRouter) {
	cfg.SignClearanceMarginM = loaded.SignClearanceMarginM
	cfg.ActivationDistM = loaded.ActivationDistM
	cfg.EscapeMaskRadiusM = loaded.EscapeMaskRadiusM
	cfg.SignLanePlanner = loaded.SignLanePlanner
	cfg.SignLaneSuppressDeform = loaded.SignLaneSuppressDeform
	cfg.SignLaneRampM = loaded.SignLaneRampM
	cfg.SignLaneHoldM = loaded.SignLaneHoldM
	cfg.SignLaneSplitOverlap = loaded.SignLaneSplitOverlap
	cfg.SignLaneSkipUnsatisfiable = loaded.SignLaneSkipUnsatisfiable
	cfg.SignLaneOffsetFrac = loaded.SignLaneOffsetFrac
	cfg.SignLaneGapCentreFrac = loaded.SignLaneGapCentreFrac
	cfg.SignLaneCornerEntryM = loaded.SignLaneCornerEntryM
	cfg.SignLaneCommitAheadM = loaded.SignLaneCommitAheadM
	cfg.SignAwareLookahead = loaded.SignAwareLookahead
	cfg.SignAwareSpeed = loaded.SignAwareSpeed
	cfg.SignDeformSpeedThresholdM = loaded.SignDeformSpeedThresholdM
	cfg.StaleTargetRescue = loaded.StaleTargetRescue
	cfg.RetraceEscape = loaded.RetraceEscape
	cfg.RetraceDistM = loaded.RetraceDistM
	cfg.RetraceSteerGainDeg = loaded.RetraceSteerGainDeg
	cfg.SignContactEvade = loaded.SignContactEvade
	cfg.SignContactDistM = loaded.SignContactDistM
	cfg.SignContactSteerDeg = loaded.SignContactSteerDeg
}

// loadSpeedConfig reads the base speed ladder, then overlays each hardware
// profile's own motion/speed.toml in order, later names winning.
//
// The overlay walk is done here instead of through profile.Load's
// profileNames parameter because speed.toml's overlays do not follow that
// function's <dir-of-base>/profiles/<name>/<base-name> layout (see the
// comment above speedTOMLPath). Each overlay is merged by handing the
// already-resolved ladder in as viper defaults, so a profile that sets only
// some tiers -- which every shipped profile does -- leaves the rest at the
// base file's values rather than zeroing them.
func loadSpeedConfig(configRoot string, hardwareProfileNames []string) (motion.NavigationMotionSpeed, error) {
	base, err := profile.Load[motion.NavigationMotionSpeed](filepath.Join(configRoot, speedTOMLPath), nil)
	if err != nil {
		return motion.NavigationMotionSpeed{}, err
	}

	resolved := *base
	for _, name := range hardwareProfileNames {
		overlayPath := filepath.Join(configRoot, speedProfilesDir, name, speedProfileRelPath)
		if _, statErr := os.Stat(overlayPath); statErr != nil {
			continue
		}
		overlaid, overlayErr := profile.LoadWithDefaults[motion.NavigationMotionSpeed](
			overlayPath,
			nil,
			speedDefaults(resolved),
		)
		if overlayErr != nil {
			return motion.NavigationMotionSpeed{}, overlayErr
		}
		resolved = *overlaid
	}
	return resolved, nil
}

// speedDefaults turns an already-resolved ladder into the defaults map an
// overlay load falls back to for every tier it does not itself set.
//
// The per-challenge tiers are carried through only when the resolved ladder
// actually has them. Seeding a nil one would turn "this profile declares no
// Open ladder" into an explicit zero, and a zero tier is not a slower robot
// -- it is a stopped one. Omitting the key instead leaves it nil, which is
// what the fallback-to-base-tier path reads.
func speedDefaults(s motion.NavigationMotionSpeed) map[string]any {
	defaults := map[string]any{
		"min_mps":    s.MinMps,
		"max_mps":    s.MaxMps,
		"creep_mps":  s.CreepMps,
		"slow_mps":   s.SlowMps,
		"medium_mps": s.MediumMps,
		"fast_mps":   s.FastMps,
	}
	for key, value := range map[string]*float64{
		"open_max_mps":         s.OpenMaxMps,
		"open_slow_mps":        s.OpenSlowMps,
		"open_medium_mps":      s.OpenMediumMps,
		"open_fast_mps":        s.OpenFastMps,
		"obstacles_max_mps":    s.ObstaclesMaxMps,
		"obstacles_slow_mps":   s.ObstaclesSlowMps,
		"obstacles_medium_mps": s.ObstaclesMediumMps,
		"obstacles_fast_mps":   s.ObstaclesFastMps,
	} {
		if value != nil {
			defaults[key] = *value
		}
	}
	return defaults
}
