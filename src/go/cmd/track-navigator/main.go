// Command track-navigator wires the nav stack (track model, corridor-width
// belief, navigator) to live NATS topics, for both isolated bench testing
// and as the real Pi 5 navigator process (a sibling of lidar-node/imu-node/
// motor-node under systemd, matching Python's track_navigator_node running
// with no --metadata).
//
// It wires the NATS-backed controllers.HardwareGateway (internal/adapters/natsgw)
// to the navigator composition root (internal/nav/navigator) and drives Step at
// the nav control rate. The gateway subscribes to vtitan.sensor.v1.scan and
// vtitan.sensor.v1.imu (published by cmd/lidar-node and the IMU node) plus
// vtitan.actuation.v1.joint_states (the motor node's encoder feedback, which
// supplies the wheel odometry bay exit bounds its legs by), estimates
// pose in-process via localization.LidarLocalizer, and publishes drive commands
// on vtitan.actuation.v1.ackermann_cmd (consumed by the motor node) — no new
// NATS subjects are introduced here.
//
// No --metadata flag exists, matching what Python's track_navigator_node.py
// says competition requires: WRO randomizes the inner walls before each
// round, so a told layout cannot be right on the day. The corridor geometry
// is always estimated from LIDAR (see newBlindLayout, widthbelief.Layout),
// and only the travel direction is optionally told via --direction.
//
// --challenge selects Open (default) or Obstacles. Obstacles wires a
// signrouter.SignRouter built EMPTY (no told sign layout, for the same
// randomization reason) and subscribes to the vision sidecar's detections
// (src/vision/nats_sidecar.py) over NATS via internal/adapters/natsvision, so
// signs are discovered from the camera during the creep exactly as
// newBlindLayout discovers the corridor widths from LIDAR.
package main

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"os"
	"os/signal"
	"path/filepath"
	"syscall"
	"time"

	natsio "github.com/nats-io/nats.go"
	"github.com/spf13/cobra"
	"golang.org/x/sync/errgroup"

	"github.com/teamvoltimor/vtitan/src/go/internal/adapters/natsgw"
	"github.com/teamvoltimor/vtitan/src/go/internal/adapters/natsvision"
	"github.com/teamvoltimor/vtitan/src/go/internal/cmdkit"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/bayexit"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/corridorestimator"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/localization"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navigator"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/startconditions"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/waypoints"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/widthbelief"
	"github.com/teamvoltimor/vtitan/src/go/internal/recording"
	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
	navv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/nav/v1"
	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
	visionv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/vision/v1"
	"github.com/teamvoltimor/vtitan/src/go/internal/transport/nats"
)

// cliConfig holds every flag track-navigator accepts.
type cliConfig struct {
	cmdkit.Common

	direction string
	challenge string
	rateHz    float64
	record    bool
}

// navSubscriptions holds the four NATS subscriptions run wires into the nav
// stack, so they can be opened and closed as one unit.
type navSubscriptions struct {
	scan       *nats.Subscriber[*sensorv1.Scan]
	imu        *nats.Subscriber[*sensorv1.Imu]
	joint      *nats.Subscriber[*actuationv1.JointStates]
	detections *nats.Subscriber[*visionv1.Detections]
}

// runtimeConfig is the set of robot/track values run loads once and hands to
// every consumer, so the gateway and the planner cannot disagree about which
// robot they describe.
type runtimeConfig struct {
	wheelRadiusM   float64
	chassisWidthM  float64
	trackMaxCoordM float64
	wpCfg          waypoints.Config
	startCfg       startconditions.Config
	estCfg         corridorestimator.Config
}

// blindNarrowWidthM is the corridor width assumed before anything has been
// measured on the Open Challenge -- the narrow (fail-safe) end of the 60/100 cm
// pair the rules allow. Believing narrow and finding wide leaves the robot with
// room; the reverse puts the planned line inside a wall. Mirrors
// CorridorDimensions.NARROW and internal/sim/scenario/native_blind.go's
// blindNarrowWidthM.
const blindNarrowWidthM = corridorestimator.DefaultNarrowWidthM

// obstaclesCorridorWidthM is what a blind OBSTACLES round assumes: every
// corridor is 1.0 m by rule, so this is prior KNOWLEDGE, not a guess. Mirrors
// CorridorDimensions.OBSTACLES_WIDTH and native_blind.go's
// obstaclesCorridorWidthM.
const obstaclesCorridorWidthM = 1.0

// challengeOpen/challengeObstacles are --challenge's legal values.
const (
	challengeOpen      = "open"
	challengeObstacles = "obstacles"
)

// defaultWheelRadiusM/defaultChassisWidthM are the fallbacks used when
// robot.toml cannot be loaded (a bench run outside the repo, or with no
// hardware profile active). They restate src/config/robot.toml's
// shipped values so a fallback run behaves like the real robot rather than
// like a zero-sized one; the warning names them so a wrong number is visible
// in the log rather than silently believed.
const (
	defaultWheelRadiusM  = 0.035
	defaultChassisWidthM = 0.194
)

// directionCW/directionCCW/directionUndetermined are --direction's legal
// values, matching track_navigator_node.py's own choices. "undetermined" is
// a real third answer, not a missing one: cw/ccw mean the operator KNOWS, so
// LIDAR direction inference is skipped outright, while undetermined means
// nobody said and the robot creeps and infers.
const (
	directionCW           = "cw"
	directionCCW          = "ccw"
	directionUndetermined = "undetermined"
)

// defaultRateHz is the navigator Step rate used when --rate-hz is not
// supplied, matching the Python track_navigator_node's control rate.
const defaultRateHz = 20.0

// exit codes: 0 means track-navigator ran and shut down cleanly (including via
// SIGINT/SIGTERM). 1 means it could not start or hit an unrecoverable runtime
// error.
const (
	exitOK    = 0
	exitError = 1
)

var _ controllers.HardwareGateway = (*natsgw.Gateway)(nil)

// parseDirection resolves --direction into the PROVISIONAL direction a first
// path is planned from (never nil -- a plan needs an axis even when nobody
// gave one) and the direction actually handed to navigator.Params (nil for
// "undetermined", so Navigator runs its own LIDAR bootstrap instead of
// trusting a guess).
func parseDirection(s string) (provisional trackmodel.Direction, known *trackmodel.Direction, err error) {
	switch s {
	case directionCW:
		d := trackmodel.Clockwise
		return d, &d, nil
	case directionCCW:
		d := trackmodel.Counterclockwise
		return d, &d, nil
	case directionUndetermined:
		return trackmodel.Clockwise, nil, nil
	default:
		return 0, nil, fmt.Errorf(
			"track-navigator: unknown --direction %q (want %s, %s, or %s)",
			s, directionCW, directionCCW, directionUndetermined,
		)
	}
}

func newRootCmd(cfg *cliConfig, logger *slog.Logger) *cobra.Command {
	cmd := &cobra.Command{
		Use:   "track-navigator",
		Short: "Run the nav stack (navigator + controllers) against live NATS topics",
		Long: "track-navigator wires the NATS-backed controllers.HardwareGateway to the\n" +
			"navigator composition root, subscribes to scan/IMU, estimates pose in-process,\n" +
			"and steps the navigator (publishing AckermannCmd) at --rate-hz.",
		SilenceUsage:  true,
		SilenceErrors: true,
		RunE: func(cmd *cobra.Command, _ []string) error {
			return run(cmd.Context(), logger, *cfg)
		},
	}

	flags := cmd.Flags()
	cfg.RegisterNATSURL(flags)
	cfg.RegisterNodeName(flags, "track-navigator")
	flags.Float64Var(&cfg.rateHz, "rate-hz", defaultRateHz, "navigator Step rate")
	flags.BoolVar(
		&cfg.record,
		"record",
		false,
		"record the run to data/live/runs as a run_<stamp>/ (MCAP bag of /scan + /nav_debug); video/photos are captured separately by cmd/capture-node",
	)
	cfg.RegisterRunsRoot(flags, "runs root dir for --record (default: repo-root data/live/runs)")
	cfg.RegisterConfigRoot(flags,
		"repo root to read the shipped TOML tree from; empty runs on Go literal defaults")
	cfg.RegisterProfiles(flags)
	flags.StringVar(&cfg.direction, "direction", directionUndetermined,
		"travel direction for the round: cw, ccw, or undetermined (default). "+
			"undetermined means the robot creeps and infers it from LIDAR, matching "+
			"what competition requires -- WRO draws the direction on the day.")
	flags.StringVar(&cfg.challenge, "challenge", challengeOpen,
		"which challenge this round runs: open (default) or obstacles. obstacles "+
			"wires a SignRouter (blind sign discovery from the vision sidecar's "+
			"detections, corridor width fixed at 1.0 m by rule) and switches the "+
			"collision/speed/pursuit tuning to their Obstacles overrides.")

	return cmd
}

// parseChallenge resolves --challenge into isObstacles, or an error for
// anything else, matching parseDirection's shape.
func parseChallenge(s string) (isObstacles bool, err error) {
	switch s {
	case challengeOpen:
		return false, nil
	case challengeObstacles:
		return true, nil
	default:
		return false, fmt.Errorf(
			"track-navigator: unknown --challenge %q (want %s or %s)",
			s, challengeOpen, challengeObstacles,
		)
	}
}

// loadTrackMaxCoordM reads track.toml's outer boundary, falling back to
// navigator.DefaultTrackMaxCoordM (the same literal ConfigFor callers get
// with no config root) on any load failure.
func loadTrackMaxCoordM(logger *slog.Logger, configRoot string) float64 {
	if configRoot == "" {
		return navigator.DefaultTrackMaxCoordM
	}
	loaded, err := profile.Load[profile.TrackConfig](
		filepath.Join(configRoot, profile.DefaultTrackTOMLPath), nil,
	)
	if err != nil {
		logger.Warn("track-navigator: loading track.toml, using default",
			"error", err, "track_max_coord_m", navigator.DefaultTrackMaxCoordM)
		return navigator.DefaultTrackMaxCoordM
	}
	return loaded.Track.MaxCoord
}

// newBlindLayout builds the initial path and the per-tick belief loop that
// corrects it, matching internal/sim/scenario/native_blind.go's
// newBlindSetup -- reproduced rather than imported, since that helper lives
// in an internal sim package.
//
// provisional is the direction the FIRST path is planned from -- never nil,
// even when the round's real direction is still undetermined (see
// parseDirection) -- and is unrelated to what navigator.Params.Direction
// receives; the navigator infers its own answer independently, and the
// belief loop attributes each tick's reading by whichever direction it
// settles on (see widthbelief.Layout.Update).
//
// isObstacles selects the prior and switches off both the estimator's voting
// and the deferral gate, matching newBlindSetup's own isObstacles branch --
// see that function's doc comment for why.
func newBlindLayout(
	logger *slog.Logger,
	base waypoints.PlannerInput,
	provisional trackmodel.Direction,
	isObstacles bool,
	wpCfg waypoints.Config,
	startCfg startconditions.Config,
	estCfg corridorestimator.Config,
) (path []trackmodel.Waypoint, priorGeometry trackmodel.CorridorGeometry, layout *widthbelief.Layout, err error) {
	priorWidthM := blindNarrowWidthM
	if isObstacles {
		priorWidthM = obstaclesCorridorWidthM
	}
	prior := map[trackmodel.Section]float64{
		trackmodel.North: priorWidthM,
		trackmodel.South: priorWidthM,
		trackmodel.East:  priorWidthM,
		trackmodel.West:  priorWidthM,
	}
	priorGeometry = trackmodel.CorridorGeometryFromWidths(prior, base.MaxCoordM)

	// The assumed START takes a different bias from the PLAN, and only on
	// Obstacles -- see newBlindSetup's own comment on assumedBiasM for why
	// this reproduces a pre-existing inconsistency in the Python original
	// rather than fixing it in passing.
	var assumedBiasM *float64
	if isObstacles {
		bias := wpCfg.WideCenterBiasM
		assumedBiasM = &bias
	}
	assumed, ok := startconditions.AssumedStartConditions(
		provisional, prior, startconditions.CanonicalSection, startCfg, assumedBiasM,
	)
	if !ok {
		return nil, trackmodel.CorridorGeometry{}, nil, fmt.Errorf(
			"track-navigator: no assumed start pose for %v from the canonical section", provisional,
		)
	}

	centerBiasM := blindCenterBiasM(isObstacles, wpCfg)
	planned := base
	planned.Geometry = priorGeometry
	planned.Starting = base.Starting.ReplannedAt(
		&provisional, assumed.Section, trackmodel.Waypoint{X: assumed.X, Y: assumed.Y}, assumed.Yaw,
	)
	path, err = waypoints.CalculateWaypoints(planned, 1, wpCfg, centerBiasM, waypoints.AllUnconfirmed())
	if err != nil {
		return nil, trackmodel.CorridorGeometry{}, nil, fmt.Errorf(
			"track-navigator: planning the prior layout: %w", err,
		)
	}

	estimatorOpts := []corridorestimator.Option{}
	if isObstacles {
		estimatorOpts = append(estimatorOpts, corridorestimator.WithFixedWidth())
	}
	layout = widthbelief.NewLayout(widthbelief.Params{
		Logger:    logger,
		Estimator: corridorestimator.New(priorWidthM, estCfg, estimatorOpts...),
		// Deferral is OPEN-ONLY -- see newBlindSetup's own comment: on
		// Obstacles the estimator is fixed and the bias is an explicit
		// override, so the gate's confirmed-ness trigger would rebuild a
		// byte-identical path and re-seek the waypoint index for nothing.
		Defer:       !isObstacles && wpCfg.DeferCurrentCorridorReplan,
		Base:        base,
		Config:      wpCfg,
		CenterBiasM: centerBiasM,
		MaxCoordM:   base.MaxCoordM,
	})
	return path, priorGeometry, layout, nil
}

// blindCenterBiasM is the planning bias for a blind round, matching
// native_blind.go's blindCenterBiasM: nil on Open, which takes the
// narrow/wide split, and wpCfg.ObstaclesCenterBiasM otherwise -- see that
// function's doc comment for the full rationale.
func blindCenterBiasM(isObstacles bool, wpCfg waypoints.Config) *float64 {
	if !isObstacles {
		return nil
	}
	bias := wpCfg.ObstaclesCenterBiasM
	return &bias
}

func run(ctx context.Context, logger *slog.Logger, cfg cliConfig) error {
	provisionalDirection, knownDirection, err := parseDirection(cfg.direction)
	if err != nil {
		return err
	}
	isObstacles, err := parseChallenge(cfg.challenge)
	if err != nil {
		return err
	}
	profiles := profile.ParseNames(cfg.Profiles)
	conn, err := nats.Connect(ctx, nats.DefaultConfig(cfg.NATSURL, cfg.NodeName))
	if err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "nats: ..." context
	}
	defer conn.Close()

	subs, err := openNavSubscriptions(conn, logger)
	if err != nil {
		return err
	}
	defer subs.close(logger)

	rt := loadRuntimeConfig(logger, cfg, profiles)

	path, priorGeometry, layout, err := newBlindLayout(
		logger,
		waypoints.PlannerInput{MaxCoordM: rt.trackMaxCoordM, ChassisWidthM: rt.chassisWidthM},
		provisionalDirection,
		isObstacles,
		rt.wpCfg,
		rt.startCfg,
		rt.estCfg,
	)
	if err != nil {
		return err
	}

	gw, err := natsgw.New(
		conn,
		trackmodel.NewTrackWalls(priorGeometry, -rt.trackMaxCoordM, rt.trackMaxCoordM),
		localization.DefaultConfig(),
		rt.wheelRadiusM,
	)
	if err != nil {
		return err //nolint:wrapcheck // main-level wiring; the cmd prints and exits
	}
	defer func() {
		if closeErr := gw.Close(); closeErr != nil {
			logger.Error("track-navigator: closing gateway", "error", closeErr)
		}
	}()

	// Built regardless of --challenge: the camera/mount constants
	// signrouter.Config carries are meaningful for VisionGateway either way,
	// and building them once here means the SignRouter constructed below (on
	// Obstacles) needs no second config-loading pass.
	srCfg := signrouter.ConfigFor(logger, cfg.ConfigRoot)
	visionGW, err := natsvision.New(
		srCfg,
		signrouter.DefaultMinReliableBBoxHeightPX,
		signrouter.DefaultMinValidLidarRangeM,
		gw,
	)
	if err != nil {
		return err //nolint:wrapcheck // main-level wiring; the cmd prints and exits
	}

	// bayexit config-root wiring matches every other config loaded above:
	// empty --config-root falls back to the Go literal defaults rather than
	// failing the run.
	bxCfg := bayexit.ConfigFor(logger, cfg.ConfigRoot, profiles)

	// A non-nil SignRouter is what identifies the Obstacles Challenge to
	// Navigator itself (see navigator.Params.SignRouter's doc comment) --
	// built empty (no signs) and on the PROVISIONAL direction, matching the
	// native sim runner's own blind construction: WRO randomizes the sign
	// layout before every round, so there is no told layout to start from,
	// only the discovery map ObservedSignMap accumulates as the camera
	// confirms signs during the creep. Navigator.adoptDirection re-keys the
	// router once the real direction settles (see SignRouter.AdoptDirection).
	var signRouter *signrouter.SignRouter
	var discCfg *signrouter.DiscoveryConfig
	if isObstacles {
		signRouter, err = signrouter.NewSignRouter(nil, srCfg, provisionalDirection)
		if err != nil {
			return err //nolint:wrapcheck // main-level wiring; the cmd prints and exits
		}
		dc := signrouter.DiscoveryConfigFor(logger, cfg.ConfigRoot)
		discCfg = &dc
	}

	nav, err := navigator.New(navigator.Params{
		Gateway:                 gw,
		Vision:                  visionGW,
		Waypoints:               path,
		Direction:               knownDirection,
		Config:                  navigator.ConfigFor(logger, cfg.ConfigRoot, profiles),
		ControllersConfig:       controllers.ConfigFor(logger, cfg.ConfigRoot, profiles),
		SignRouterConfig:        srCfg,
		SignRouter:              signRouter,
		SignDiscoveryConfig:     discCfg,
		BayExitConfig:           &bxCfg,
		CorridorEstimatorConfig: &rt.estCfg,
		Logger:                  logger,
	})
	if err != nil {
		return err //nolint:wrapcheck // navigator.New already wraps with "navigator: ..." context
	}

	logger.Info("track-navigator: connected", "nats_url", cfg.NATSURL, "rate_hz", cfg.rateHz)

	rec, closeRec, err := startRecording(cfg, logger)
	if err != nil {
		return err
	}
	defer closeRec()

	group, gctx := errgroup.WithContext(ctx)
	group.Go(func() error { return gw.Run(gctx, subs.scan, subs.imu, subs.joint) })
	group.Go(func() error { return visionGW.Run(gctx, subs.detections) })
	group.Go(func() error { return stepLoop(gctx, logger, nav, gw, layout, rec, cfg.rateHz) })

	if err = group.Wait(); err != nil {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return nil
		}
		return err //nolint:wrapcheck // each goroutine's own error is already package-prefixed
	}
	return nil
}

// openNavSubscriptions opens the four sensor/feedback subscriptions the nav
// stack consumes. On failure it closes any already-opened ones before
// returning the error.
func openNavSubscriptions(conn *natsio.Conn, logger *slog.Logger) (subs *navSubscriptions, err error) {
	scan, err := nats.NewSubscriber[sensorv1.Scan](conn, sensorv1.ScanSubject)
	if err != nil {
		return nil, err
	}
	subs = &navSubscriptions{scan: scan}

	imu, err := nats.NewSubscriber[sensorv1.Imu](conn, sensorv1.ImuSubject)
	if err != nil {
		subs.close(logger)
		return nil, err
	}
	subs.imu = imu

	joint, err := nats.NewSubscriber[actuationv1.JointStates](conn, actuationv1.JointStatesSubject)
	if err != nil {
		subs.close(logger)
		return nil, err
	}
	subs.joint = joint

	// Published by the Python sidecar (src/vision/nats_sidecar.py), never by
	// anything in this Go tree -- absent that process, this subscription
	// simply never receives anything, and GetVisionDetections stays
	// ok=false, exactly like any other sensor this binary has no producer
	// for. Only consulted when --challenge obstacles wires a SignRouter (see
	// run); on Open it is opened and subscribed like every other topic here,
	// but nothing in Navigator ever reads VisionGateway.
	detections, err := nats.NewSubscriber[visionv1.Detections](conn, visionv1.DetectionsSubject)
	if err != nil {
		subs.close(logger)
		return nil, err
	}
	subs.detections = detections

	return subs, nil
}

// close closes every opened subscription, logging (rather than returning)
// any failure the way the original per-subscription defers did.
func (s *navSubscriptions) close(logger *slog.Logger) {
	if s.scan != nil {
		if closeErr := s.scan.Close(); closeErr != nil {
			logger.Error("track-navigator: closing Scan subscription", "error", closeErr)
		}
	}
	if s.imu != nil {
		if closeErr := s.imu.Close(); closeErr != nil {
			logger.Error("track-navigator: closing IMU subscription", "error", closeErr)
		}
	}
	if s.joint != nil {
		if closeErr := s.joint.Close(); closeErr != nil {
			logger.Error("track-navigator: closing JointStates subscription", "error", closeErr)
		}
	}
	if s.detections != nil {
		if closeErr := s.detections.Close(); closeErr != nil {
			logger.Error("track-navigator: closing Detections subscription", "error", closeErr)
		}
	}
}

// loadRuntimeConfig loads robot.toml plus the track/waypoint/start-condition
// configs once, so every consumer in run shares one view of the robot.
func loadRuntimeConfig(logger *slog.Logger, cfg cliConfig, profiles []string) runtimeConfig {
	robotPath := profile.DefaultRobotTOMLPath
	if cfg.ConfigRoot != "" {
		robotPath = filepath.Join(cfg.ConfigRoot, profile.DefaultRobotTOMLPath)
	}
	robotCfg, robotCfgErr := profile.LoadRobotConfig(robotPath, profiles)

	rt := runtimeConfig{
		wheelRadiusM:  defaultWheelRadiusM,
		chassisWidthM: defaultChassisWidthM,
	}
	if robotCfgErr == nil {
		rt.wheelRadiusM = robotCfg.Wheel.Radius
		rt.chassisWidthM = robotCfg.Chassis.Width
	} else {
		logger.Warn("track-navigator: loading robot.toml, using defaults",
			"error", robotCfgErr,
			"wheel_radius_m", rt.wheelRadiusM,
			"chassis_width_m", rt.chassisWidthM)
	}

	rt.trackMaxCoordM = loadTrackMaxCoordM(logger, cfg.ConfigRoot)
	rt.wpCfg = waypoints.ConfigFor(logger, cfg.ConfigRoot)
	rt.startCfg = startconditions.ConfigFor(logger, cfg.ConfigRoot)
	rt.estCfg = corridorestimator.ConfigFor(logger, cfg.ConfigRoot)
	return rt
}

// startRecording opens a run recorder when --record is set, returning the
// recorder and a cleanup function that closes it. With --record off it
// returns a no-op cleanup and a nil recorder.
func startRecording(cfg cliConfig, logger *slog.Logger) (rec *recording.RunRecorder, closeRec func(), err error) {
	if !cfg.record {
		return nil, func() {}, nil
	}
	rec, err = recording.NewRun(cfg.RunsRoot, recording.RunOptions{Video: false})
	if err != nil {
		return nil, nil, fmt.Errorf("track-navigator: creating run: %w", err)
	}
	if err = rec.Open(); err != nil {
		return nil, nil, fmt.Errorf("track-navigator: opening run: %w", err)
	}
	logger.Info("track-navigator: recording run", "dir", rec.Dir())
	return rec, func() {
		if closeErr := rec.Close(); closeErr != nil {
			logger.Error("track-navigator: closing run", "error", closeErr)
		}
	}, nil
}

// stepLoop drives nav.Step at rateHz until ctx is done. When rec is non-nil it
// also writes each tick's /scan and /nav_debug into the run's MCAP bag, so a Go
// run matches the Python robot's run_<stamp>/ shape for bagreplay parity.
//
// layout.Update runs every tick right after Step, matching
// native_runner.go's own loop: a deferred belief is released by the robot
// LEAVING a corridor, so the tick that applies it is usually one the
// estimator had nothing new to say about, not only the tick a fresh reading
// arrived on.
func stepLoop(
	ctx context.Context,
	logger *slog.Logger,
	nav *navigator.Navigator,
	gw *natsgw.Gateway,
	layout *widthbelief.Layout,
	rec *recording.RunRecorder,
	rateHz float64,
) error {
	ticker := time.NewTicker(time.Duration(float64(time.Second) / rateHz))
	defer ticker.Stop()

	steps := 0
	for {
		select {
		case <-ctx.Done():
			logger.Info("track-navigator: stepped", "steps", steps)
			return nil
		case <-ticker.C:
			nav.Step()
			layout.Update(nav, gw, nav.Direction())
			steps++
			if rec != nil {
				if scan := gw.LatestScan(); scan != nil {
					if err := rec.WriteMessage(sensorv1.ScanSubject, scan, logTimeNow()); err != nil {
						logger.Error("track-navigator: writing scan", "error", err)
					}
				}
				if err := rec.WriteMessage(
					navv1.NavigatorDebugSubject,
					nav.DebugSnapshot().ToProto(),
					logTimeNow(),
				); err != nil {
					logger.Error("track-navigator: writing nav_debug", "error", err)
				}
			}
		}
	}
}

// logTimeNow returns a monotonic-ish MCAP log time in nanoseconds (wall clock),
// good enough to order messages within a single run.
func logTimeNow() uint64 {
	return uint64(time.Now().UnixNano())
}

func main() {
	os.Exit(mainWithExitCode())
}

// mainWithExitCode does the real work of main and returns the process exit
// code instead of calling os.Exit directly, so deferred cleanup (closing NATS
// subscriptions/connection, releasing the signal.NotifyContext) actually runs
// before the process exits.
func mainWithExitCode() int {
	logger := slog.New(slog.NewTextHandler(os.Stderr, nil))

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	var cfg cliConfig
	cmd := newRootCmd(&cfg, logger)
	cmd.SetArgs(os.Args[1:])

	if err := cmd.ExecuteContext(ctx); err != nil {
		logger.Error("track-navigator run failed", "error", err)
		return exitError
	}
	return exitOK
}
