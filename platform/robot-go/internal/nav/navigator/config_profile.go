package navigator

import (
	"log/slog"
	"os"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/config/profile"
)

// speedTOMLPath/headingTOMLPath/speedProfilesDir are declared here rather
// than in internal/config/profile because neither file has a profile mirror
// there yet, and speed.toml's overlay layout is unlike every path that
// package already models: its per-motor overlay lives at
// platform/shared/config/profiles/<name>/motion/speed.toml, NOT at
// <dir-of-base>/profiles/<name>/<base-name>, which is the only shape
// profile.Load's own merge understands (that shape is what robot.toml
// uses). loadSpeedConfig below does the overlay walk itself for that
// reason.

// speedTOML mirrors platform/shared/config/navigation/motion/speed.toml
// (shared.config.navigation_tuning.motion.SpeedControlParams' raw fields,
// before the drivetrain clamp its *_mps() accessors apply).
type speedTOML struct {
	MinMPS    float64 `mapstructure:"min_mps"`
	MaxMPS    float64 `mapstructure:"max_mps"`
	CreepMPS  float64 `mapstructure:"creep_mps"`
	SlowMPS   float64 `mapstructure:"slow_mps"`
	MediumMPS float64 `mapstructure:"medium_mps"`
	FastMPS   float64 `mapstructure:"fast_mps"`
}

// headingTOML mirrors platform/shared/config/navigation/motion/heading.toml
// (HeadingErrorZones). One threshold, not a ladder -- see the TOML's own
// comment for the 33%-of-lap-time measurement that deleted the other rungs.
type headingTOML struct {
	Crawl float64 `mapstructure:"crawl"`
}

// navWaypointsTOML mirrors the two waypoints.toml fields CoreNavigator
// itself reads and profile.WaypointsConfig deliberately omits (they belong
// to the navigator, not to waypoint generation).
type navWaypointsTOML struct {
	MainLoopReachedDistanceM float64 `mapstructure:"main_loop_reached_distance_m"`
	ReplanHeadingTieMarginM  float64 `mapstructure:"replan_heading_tie_margin_m"`
	// ArcRadius doubles as ParkEngageDistM, matching Python's
	// _park_engage_dist = tuning.waypoints.ARC_RADIUS.
	ArcRadius float64 `mapstructure:"arc_radius"`
}

// navEscapeTOML mirrors the escape.toml fields the core navigator's
// pose-trail retrace and escalating-escape logic read. profile.EscapeConfig
// deliberately covers only the subset StuckDetector/
// CollisionAvoidanceController consume, so the overlap here is intentional
// rather than a duplicate: these are the fields its doc comment names as
// belonging to this package.
type navEscapeTOML struct {
	PoseTrailMinStepM               float64 `mapstructure:"pose_trail_min_step_m"`
	PoseTrailLen                    int     `mapstructure:"pose_trail_len"`
	RevSpeed                        float64 `mapstructure:"rev_speed"`
	RevSteerDeg                     float64 `mapstructure:"rev_steer_deg"`
	KTurnMinFrames                  int     `mapstructure:"k_turn_min_frames"`
	EscalateAfterAttempts           int     `mapstructure:"escalate_after_attempts"`
	EscapeSideCommitAttempts        int     `mapstructure:"escape_side_commit_attempts"`
	MaxEscapeFrames                 int     `mapstructure:"max_escape_frames"`
	StuckEscalationFramesPerAttempt int     `mapstructure:"stuck_escalation_frames_per_attempt"`
	StuckMoveThreshold              float64 `mapstructure:"stuck_move_threshold"`
}

// navSignRouterTOML mirrors the sign_router.toml / SignRouterParams fields
// the NAVIGATOR reads -- the lane planner, the escape mask, the retrace and
// the sign-contact evade -- which profile.SignRouterConfig omits because
// internal/nav/signrouter itself consumes none of them. Most are absent
// from the checked-in TOML entirely and rely on the Pydantic model's own
// defaults, applied via LoadWithDefaults (see navSignRouterDefaults).
type navSignRouterTOML struct {
	SignClearanceMarginM      float64 `mapstructure:"sign_clearance_margin_m"`
	ActivationDistM           float64 `mapstructure:"activation_dist_m"`
	EscapeMaskRadiusM         float64 `mapstructure:"escape_mask_radius_m"`
	SignLanePlanner           bool    `mapstructure:"sign_lane_planner"`
	SignLaneSuppressDeform    bool    `mapstructure:"sign_lane_suppress_deform"`
	SignLaneRampM             float64 `mapstructure:"sign_lane_ramp_m"`
	SignLaneHoldM             float64 `mapstructure:"sign_lane_hold_m"`
	SignLaneSplitOverlap      bool    `mapstructure:"sign_lane_split_overlap"`
	SignLaneSkipUnsatisfiable bool    `mapstructure:"sign_lane_skip_unsatisfiable"`
	SignLaneOffsetFrac        float64 `mapstructure:"sign_lane_offset_frac"`
	SignLaneCornerEntryM      float64 `mapstructure:"sign_lane_corner_entry_m"`
	SignLaneCommitAheadM      float64 `mapstructure:"sign_lane_commit_ahead_m"`
	SignAwareLookahead        bool    `mapstructure:"sign_aware_lookahead"`
	SignAwareSpeed            bool    `mapstructure:"sign_aware_speed"`
	SignDeformSpeedThresholdM float64 `mapstructure:"sign_deform_speed_threshold_m"`
	StaleTargetRescue         bool    `mapstructure:"stale_target_rescue"`
	RetraceEscape             bool    `mapstructure:"retrace_escape"`
	RetraceDistM              float64 `mapstructure:"retrace_dist_m"`
	RetraceSteerGainDeg       float64 `mapstructure:"retrace_steer_gain_deg"`
	SignContactEvade          bool    `mapstructure:"sign_contact_evade"`
	SignContactDistM          float64 `mapstructure:"sign_contact_dist_m"`
	SignContactSteerDeg       float64 `mapstructure:"sign_contact_steer_deg"`
}

const (
	speedTOMLPath   = "platform/shared/config/navigation/motion/speed.toml"
	headingTOMLPath = "platform/shared/config/navigation/motion/heading.toml"
	// speedProfilesDir is the directory holding each hardware profile's
	// overlay tree, relative to the repo root.
	speedProfilesDir = "platform/shared/config/profiles"
	// speedProfileRelPath is where a profile's speed overlay sits inside
	// its own directory.
	speedProfileRelPath = "motion/speed.toml"
)

// navSignRouterDefaults is the LoadWithDefaults defaults map for
// navSignRouterTOML: every key the checked-in sign_router.toml never sets,
// so an omitted key reads as SignRouterParams' own Pydantic default rather
// than silently as false/0.0.
func navSignRouterDefaults() map[string]any {
	return map[string]any{
		"sign_lane_planner":             DefaultSignLanePlanner,
		"sign_lane_suppress_deform":     DefaultSignLaneSuppressDeform,
		"sign_lane_ramp_m":              DefaultSignLaneRampM,
		"sign_lane_hold_m":              DefaultSignLaneHoldM,
		"sign_lane_split_overlap":       DefaultSignLaneSplitOverlap,
		"sign_lane_skip_unsatisfiable":  DefaultSignLaneSkipUnsatisfiable,
		"sign_lane_offset_frac":         DefaultSignLaneOffsetFrac,
		"sign_lane_corner_entry_m":      DefaultSignLaneCornerEntryM,
		"sign_lane_commit_ahead_m":      DefaultSignLaneCommitAheadM,
		"sign_aware_lookahead":          DefaultSignAwareLookahead,
		"sign_aware_speed":              DefaultSignAwareSpeed,
		"sign_deform_speed_threshold_m": DefaultSignDeformSpeedThresholdM,
		"stale_target_rescue":           DefaultStaleTargetRescue,
		"retrace_escape":                DefaultRetraceEscape,
		"retrace_dist_m":                DefaultRetraceDistM,
		"retrace_steer_gain_deg":        DefaultRetraceSteerGainDeg,
		"sign_contact_evade":            DefaultSignContactEvade,
		"sign_contact_dist_m":           DefaultSignContactDistM,
		"sign_contact_steer_deg":        DefaultSignContactSteerDeg,
	}
}

// loadApplyTOML loads a profile TOML from path and, on success, hands the
// decoded value to apply; on failure it logs a warning and leaves cfg at its
// defaults. Each source loads and falls back independently because they're
// unrelated failure domains -- the same contract controllers.ConfigFor and
// signrouter.ConfigFor state.
func loadApplyTOML[T any](logger *slog.Logger, path, what string, apply func(T)) {
	loaded, err := profile.Load[T](path, nil)
	if err != nil {
		logger.Warn(
			"navigator: loading config file, falling back to defaults",
			"file",
			what,
			"error",
			err,
		)
		return
	}
	apply(*loaded)
}

// loadApplyTOMLWithDefaults is loadApplyTOML for sources whose checked-in
// TOML omits keys that must read as their Pydantic defaults, not as zero.
func loadApplyTOMLWithDefaults[T any](
	logger *slog.Logger,
	path, what string,
	defaults map[string]any,
	apply func(T),
) {
	loaded, err := profile.LoadWithDefaults[T](path, nil, defaults)
	if err != nil {
		logger.Warn(
			"navigator: loading config file, falling back to defaults",
			"file",
			what,
			"error",
			err,
		)
		return
	}
	apply(*loaded)
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
		})

	if speed, err := loadSpeedConfig(configRoot, hardwareProfileNames); err != nil {
		logger.Warn("navigator: loading speed.toml, falling back to defaults", "error", err)
	} else {
		cfg.MinMPS = speed.MinMPS
		cfg.MaxMPS = speed.MaxMPS
		cfg.CreepMPS = speed.CreepMPS
		cfg.SlowMPS = speed.SlowMPS
		cfg.MediumMPS = speed.MediumMPS
		cfg.FastMPS = speed.FastMPS
	}

	loadApplyTOML(
		logger,
		filepath.Join(configRoot, headingTOMLPath),
		"heading.toml",
		func(loaded headingTOML) {
			cfg.CrawlRad = loaded.Crawl
		},
	)

	loadApplyTOML(
		logger, filepath.Join(configRoot, profile.DefaultPursuitTOMLPath), "pursuit.toml",
		func(loaded profile.PursuitConfig) {
			cfg.WallMarginSafetyM = loaded.WallMarginSafetyM
			cfg.MinLookaheadTransitionM = loaded.MinLookaheadTransitionM
			cfg.CornerPreviewDistanceM = loaded.CornerPreviewDistanceM
		})

	loadApplyTOML(
		logger,
		filepath.Join(configRoot, profile.DefaultWaypointsTOMLPath),
		"waypoints.toml",
		func(loaded navWaypointsTOML) {
			cfg.MainLoopReachedDistanceM = loaded.MainLoopReachedDistanceM
			cfg.ReplanHeadingTieMarginM = loaded.ReplanHeadingTieMarginM
			cfg.ParkEngageDistM = loaded.ArcRadius
		},
	)

	loadApplyTOML(
		logger,
		filepath.Join(configRoot, profile.DefaultControlTOMLPath),
		"control.toml",
		func(loaded profile.ControlConfig) {
			cfg.ControlHz = loaded.ControlHz
		},
	)

	loadApplyTOML(
		logger,
		filepath.Join(configRoot, profile.DefaultLidarSectorsTOMLPath),
		"lidar_sectors.toml",
		func(loaded profile.LidarSectorsConfig) {
			cfg.NoDataRangeM = loaded.NoDataRangeM
		},
	)

	loadApplyTOML(
		logger,
		filepath.Join(configRoot, profile.DefaultEscapeTOMLPath),
		"escape.toml",
		func(loaded navEscapeTOML) {
			applyEscapeTOML(&cfg, loaded)
		},
	)

	loadApplyTOMLWithDefaults(
		logger,
		filepath.Join(configRoot, profile.DefaultSignRouterTOMLPath),
		"sign_router.toml",
		navSignRouterDefaults(),
		func(loaded navSignRouterTOML) {
			applySignRouterTOML(&cfg, loaded)
		},
	)

	loadApplyTOML(
		logger,
		filepath.Join(configRoot, profile.DefaultRobotTOMLPath),
		"robot.toml",
		func(loaded profile.RobotConfig) {
			cfg.ChassisWidthM = loaded.Chassis.Width
			cfg.MaxSteeringAngleRad = loaded.MaxSteeringAngle()
			cfg.LidarToFrontBumperM = loaded.LidarToFrontBumper()
			cfg.LidarToRearBumperM = loaded.LidarToRearBumper()
			cfg.DrivetrainMaxSpeedMPS = loaded.Drivetrain.MaxSpeedMPS
		},
	)

	loadApplyTOML(
		logger,
		filepath.Join(configRoot, profile.DefaultTrackTOMLPath),
		"track.toml",
		func(loaded profile.TrackConfig) {
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
func applyEscapeTOML(cfg *Config, loaded navEscapeTOML) {
	cfg.PoseTrailMinStepM = loaded.PoseTrailMinStepM
	cfg.PoseTrailLen = loaded.PoseTrailLen
	cfg.RevSpeed = loaded.RevSpeed
	cfg.RevSteerDeg = loaded.RevSteerDeg
	cfg.KTurnMinFrames = loaded.KTurnMinFrames
	cfg.EscalateAfterAttempts = loaded.EscalateAfterAttempts
	cfg.EscapeSideCommitAttempts = loaded.EscapeSideCommitAttempts
	cfg.MaxEscapeFrames = loaded.MaxEscapeFrames
	cfg.StuckEscalationFramesPerAttempt = loaded.StuckEscalationFramesPerAttempt
	cfg.StuckMoveThreshold = loaded.StuckMoveThreshold
}

// applySignRouterTOML copies loaded sign_router.toml values onto cfg, for
// the same reason applyEscapeTOML exists.
func applySignRouterTOML(cfg *Config, loaded navSignRouterTOML) {
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
// function's <dir-of-base>/profiles/<name>/<base-name> layout (see
// speedTOMLPath's comment). Each overlay is merged by handing the
// already-resolved ladder in as viper defaults, so a profile that sets only
// some tiers -- which every shipped profile does -- leaves the rest at the
// base file's values rather than zeroing them.
func loadSpeedConfig(configRoot string, hardwareProfileNames []string) (speedTOML, error) {
	base, err := profile.Load[speedTOML](filepath.Join(configRoot, speedTOMLPath), nil)
	if err != nil {
		return speedTOML{}, err
	}

	resolved := *base
	for _, name := range hardwareProfileNames {
		overlayPath := filepath.Join(configRoot, speedProfilesDir, name, speedProfileRelPath)
		if _, statErr := os.Stat(overlayPath); statErr != nil {
			continue
		}
		overlaid, overlayErr := profile.LoadWithDefaults[speedTOML](
			overlayPath,
			nil,
			speedDefaults(resolved),
		)
		if overlayErr != nil {
			return speedTOML{}, overlayErr
		}
		resolved = *overlaid
	}
	return resolved, nil
}

// speedDefaults turns an already-resolved ladder into the defaults map an
// overlay load falls back to for every tier it does not itself set.
func speedDefaults(s speedTOML) map[string]any {
	return map[string]any{
		"min_mps":    s.MinMPS,
		"max_mps":    s.MaxMPS,
		"creep_mps":  s.CreepMPS,
		"slow_mps":   s.SlowMPS,
		"medium_mps": s.MediumMPS,
		"fast_mps":   s.FastMPS,
	}
}
