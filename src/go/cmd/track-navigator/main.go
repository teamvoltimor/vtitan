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
package main

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"os"
	"os/signal"
	"path/filepath"
	"strings"
	"syscall"
	"time"

	"github.com/spf13/cobra"
	"golang.org/x/sync/errgroup"

	"github.com/teamvoltimor/vtitan/src/go/internal/adapters/natsgw"
	"github.com/teamvoltimor/vtitan/src/go/internal/adapters/natsvision"
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
	"github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/nav/v1"
	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
	visionv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/vision/v1"
	"github.com/teamvoltimor/vtitan/src/go/internal/transport/nats"
)

// blindNarrowWidthM is the corridor width assumed before anything has been
// measured -- the narrow (fail-safe) end of the 60/100 cm pair the Open
// Challenge rules allow. Believing narrow and finding wide leaves the robot
// with room; the reverse puts the planned line inside a wall. Mirrors
// CorridorDimensions.NARROW and internal/sim/scenario/native_blind.go's
// blindNarrowWidthM (Obstacles is not handled here -- see newBlindLayout).
const blindNarrowWidthM = corridorestimator.DefaultNarrowWidthM

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

// cliConfig holds every flag track-navigator accepts.
type cliConfig struct {
	natsURL    string
	nodeName   string
	rateHz     float64
	record     bool
	runsRoot   string
	configRoot string
	profiles   string
	direction  string
}

// splitProfiles parses a comma-separated hardware-profile list, matching
// cmd/pi5's own helper of the same name (each cmd/* binary that takes
// --profiles has its own copy rather than sharing one -- there is no shared
// CLI package for a two-line string split).
func splitProfiles(s string) []string {
	if s == "" {
		return nil
	}
	var out []string
	for _, p := range strings.Split(s, ",") {
		if p = strings.TrimSpace(p); p != "" {
			out = append(out, p)
		}
	}
	return out
}

// exit codes: 0 means track-navigator ran and shut down cleanly (including via
// SIGINT/SIGTERM). 1 means it could not start or hit an unrecoverable runtime
// error.
const (
	exitOK    = 0
	exitError = 1
)

var _ controllers.HardwareGateway = (*natsgw.Gateway)(nil)

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
	flags.StringVar(&cfg.natsURL, "nats-url", nats.DefaultURL(), "nats-server URL")
	flags.StringVar(
		&cfg.nodeName,
		"name",
		"track-navigator",
		"NATS client name, visible in nats-server's connz output",
	)
	flags.Float64Var(&cfg.rateHz, "rate-hz", 20.0, "navigator Step rate")
	flags.BoolVar(&cfg.record, "record", false,
		"record the run to data/live/runs as a run_<stamp>/ (MCAP bag of /scan + /nav_debug); video/photos are captured separately by cmd/capture-node")
	flags.StringVar(&cfg.runsRoot, "runs-root", "", "runs root dir for --record (default: repo-root data/live/runs)")
	flags.StringVar(&cfg.configRoot, "config-root", "",
		"repo root to read the shipped TOML tree from; empty runs on Go literal defaults")
	flags.StringVar(&cfg.profiles, "profiles", "",
		"comma-separated hardware profiles (overrides VTITAN_HARDWARE_PROFILE)")
	flags.StringVar(&cfg.direction, "direction", directionUndetermined,
		"travel direction for the round: cw, ccw, or undetermined (default). "+
			"undetermined means the robot creeps and infers it from LIDAR, matching "+
			"what competition requires -- WRO draws the direction on the day.")

	return cmd
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
// in an internal sim package and is Obstacles-aware in a way this Open-only
// production entry point deliberately is not (no SignRouter is wired here,
// so isObstacles is never true; see the package doc comment).
//
// provisional is the direction the FIRST path is planned from -- never nil,
// even when the round's real direction is still undetermined (see
// parseDirection) -- and is unrelated to what navigator.Params.Direction
// receives; the navigator infers its own answer independently, and the
// belief loop attributes each tick's reading by whichever direction it
// settles on (see widthbelief.Layout.Update).
func newBlindLayout(
	logger *slog.Logger,
	base waypoints.PlannerInput,
	provisional trackmodel.Direction,
	wpCfg waypoints.Config,
	startCfg startconditions.Config,
	estCfg corridorestimator.Config,
) (path []trackmodel.Waypoint, priorGeometry trackmodel.CorridorGeometry, layout *widthbelief.Layout, err error) {
	prior := map[trackmodel.Section]float64{
		trackmodel.North: blindNarrowWidthM,
		trackmodel.South: blindNarrowWidthM,
		trackmodel.East:  blindNarrowWidthM,
		trackmodel.West:  blindNarrowWidthM,
	}
	priorGeometry = trackmodel.CorridorGeometryFromWidths(prior, base.MaxCoordM)

	assumed, ok := startconditions.AssumedStartConditions(
		provisional, prior, startconditions.CanonicalSection, startCfg, nil,
	)
	if !ok {
		return nil, trackmodel.CorridorGeometry{}, nil, fmt.Errorf(
			"track-navigator: no assumed start pose for %v from the canonical section", provisional,
		)
	}

	planned := base
	planned.Geometry = priorGeometry
	planned.Starting = base.Starting.ReplannedAt(
		&provisional, assumed.Section, trackmodel.Waypoint{X: assumed.X, Y: assumed.Y}, assumed.Yaw,
	)
	path, err = waypoints.CalculateWaypoints(planned, 1, wpCfg, nil, waypoints.AllUnconfirmed())
	if err != nil {
		return nil, trackmodel.CorridorGeometry{}, nil, fmt.Errorf(
			"track-navigator: planning the prior layout: %w", err,
		)
	}

	layout = widthbelief.NewLayout(widthbelief.Params{
		Logger:      logger,
		Estimator:   corridorestimator.New(blindNarrowWidthM, estCfg),
		Defer:       wpCfg.DeferCurrentCorridorReplan,
		Base:        base,
		Config:      wpCfg,
		CenterBiasM: nil,
		MaxCoordM:   base.MaxCoordM,
	})
	return path, priorGeometry, layout, nil
}

func run(ctx context.Context, logger *slog.Logger, cfg cliConfig) error {
	provisionalDirection, knownDirection, err := parseDirection(cfg.direction)
	if err != nil {
		return err
	}
	profiles := splitProfiles(cfg.profiles)
	conn, err := nats.Connect(ctx, nats.DefaultConfig(cfg.natsURL, cfg.nodeName))
	if err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "nats: ..." context
	}
	defer conn.Close()

	scanSub, err := nats.NewSubscriber(
		conn,
		sensorv1.ScanSubject,
		func() *sensorv1.Scan { return &sensorv1.Scan{} },
	)
	if err != nil {
		return err //nolint:wrapcheck // NewSubscriber already wraps with "nats: ..." context
	}
	defer func() {
		if closeErr := scanSub.Close(); closeErr != nil {
			logger.Error("track-navigator: closing Scan subscription", "error", closeErr)
		}
	}()

	imuSub, err := nats.NewSubscriber(
		conn,
		sensorv1.ImuSubject,
		func() *sensorv1.Imu { return &sensorv1.Imu{} },
	)
	if err != nil {
		return err //nolint:wrapcheck // NewSubscriber already wraps with "nats: ..." context
	}
	defer func() {
		if closeErr := imuSub.Close(); closeErr != nil {
			logger.Error("track-navigator: closing IMU subscription", "error", closeErr)
		}
	}()

	jointSub, err := nats.NewSubscriber(
		conn,
		actuationv1.JointStatesSubject,
		func() *actuationv1.JointStates { return &actuationv1.JointStates{} },
	)
	if err != nil {
		return err //nolint:wrapcheck // NewSubscriber already wraps with "nats: ..." context
	}
	defer func() {
		if closeErr := jointSub.Close(); closeErr != nil {
			logger.Error("track-navigator: closing JointStates subscription", "error", closeErr)
		}
	}()

	// Published by the Python sidecar (src/vision/nats_sidecar.py), never by
	// anything in this Go tree -- absent that process, this subscription
	// simply never receives anything, and GetVisionDetections stays
	// ok=false, exactly like any other sensor this binary has no producer
	// for. Currently inert regardless: no SignRouter is wired below (Open
	// Challenge only), and VisionGateway is only consulted when one is.
	detectionsSub, err := nats.NewSubscriber(
		conn,
		visionv1.DetectionsSubject,
		func() *visionv1.Detections { return &visionv1.Detections{} },
	)
	if err != nil {
		return err //nolint:wrapcheck // NewSubscriber already wraps with "nats: ..." context
	}
	defer func() {
		if closeErr := detectionsSub.Close(); closeErr != nil {
			logger.Error("track-navigator: closing Detections subscription", "error", closeErr)
		}
	}()

	// robot.toml supplies both the wheel radius the gateway scales joint
	// angles by and the chassis width the planner sizes its corridor with.
	// Loaded once, before either consumer, so the two cannot disagree about
	// which robot they are describing.
	robotPath := profile.DefaultRobotTOMLPath
	if cfg.configRoot != "" {
		robotPath = filepath.Join(cfg.configRoot, profile.DefaultRobotTOMLPath)
	}
	robotCfg, robotCfgErr := profile.LoadRobotConfig(robotPath, profiles)
	wheelRadiusM := defaultWheelRadiusM
	chassisWidthM := defaultChassisWidthM
	if robotCfgErr == nil {
		wheelRadiusM = robotCfg.Wheel.Radius
		chassisWidthM = robotCfg.Chassis.Width
	} else {
		logger.Warn("track-navigator: loading robot.toml, using defaults",
			"error", robotCfgErr,
			"wheel_radius_m", wheelRadiusM,
			"chassis_width_m", chassisWidthM)
	}

	trackMaxCoordM := loadTrackMaxCoordM(logger, cfg.configRoot)
	wpCfg := waypoints.ConfigFor(logger, cfg.configRoot)
	startCfg := startconditions.ConfigFor(logger, cfg.configRoot)
	estCfg := corridorestimator.ConfigFor(logger, cfg.configRoot)

	path, priorGeometry, layout, err := newBlindLayout(
		logger,
		waypoints.PlannerInput{MaxCoordM: trackMaxCoordM, ChassisWidthM: chassisWidthM},
		provisionalDirection,
		wpCfg,
		startCfg,
		estCfg,
	)
	if err != nil {
		return err
	}

	gw, err := natsgw.New(
		conn,
		trackmodel.NewTrackWalls(priorGeometry, -trackMaxCoordM, trackMaxCoordM),
		localization.DefaultConfig(),
		wheelRadiusM,
	)
	if err != nil {
		return err //nolint:wrapcheck
	}
	defer func() {
		if closeErr := gw.Close(); closeErr != nil {
			logger.Error("track-navigator: closing gateway", "error", closeErr)
		}
	}()

	// Built regardless of whether a SignRouter is wired below (see
	// detectionsSub's own comment): the camera/mount constants
	// signrouter.Config carries are meaningful independent of that, and
	// building them once here means a future SignRouter wire-in needs no
	// second config-loading pass.
	srCfg := signrouter.ConfigFor(logger, cfg.configRoot)
	visionGW, err := natsvision.New(srCfg, signrouter.DefaultMinReliableBBoxHeightPX, signrouter.DefaultMinValidLidarRangeM, gw)
	if err != nil {
		return err //nolint:wrapcheck
	}

	// bayexit config-root wiring matches every other config loaded above:
	// empty --config-root falls back to the Go literal defaults rather than
	// failing the run.
	bxCfg := bayexit.ConfigFor(logger, cfg.configRoot, profiles)

	nav, err := navigator.New(navigator.Params{
		Gateway:                 gw,
		Vision:                  visionGW,
		Waypoints:               path,
		Direction:               knownDirection,
		Config:                  navigator.ConfigFor(logger, cfg.configRoot, profiles),
		ControllersConfig:       controllers.ConfigFor(logger, cfg.configRoot, profiles),
		BayExitConfig:           &bxCfg,
		CorridorEstimatorConfig: &estCfg,
		Logger:                  logger,
	})
	if err != nil {
		return err //nolint:wrapcheck // navigator.New already wraps with "navigator: ..." context
	}

	logger.Info("track-navigator: connected", "nats_url", cfg.natsURL, "rate_hz", cfg.rateHz)

	var rec *recording.RunRecorder
	if cfg.record {
		r, err := recording.NewRun(cfg.runsRoot, recording.RunOptions{Video: false})
		if err != nil {
			return fmt.Errorf("track-navigator: creating run: %w", err)
		}
		if err = r.Open(); err != nil {
			return fmt.Errorf("track-navigator: opening run: %w", err)
		}
		rec = r
		defer func() {
			if closeErr := rec.Close(); closeErr != nil {
				logger.Error("track-navigator: closing run", "error", closeErr)
			}
		}()
		logger.Info("track-navigator: recording run", "dir", rec.Dir())
	}

	group, gctx := errgroup.WithContext(ctx)
	group.Go(func() error { return gw.Run(gctx, scanSub, imuSub, jointSub) })
	group.Go(func() error { return visionGW.Run(gctx, detectionsSub) })
	group.Go(func() error { return stepLoop(gctx, logger, nav, gw, layout, rec, cfg.rateHz) })

	if err = group.Wait(); err != nil {
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return nil
		}
		return err //nolint:wrapcheck // each goroutine's own error is already package-prefixed
	}
	return nil
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
				if err := rec.WriteMessage(navv1.NavigatorDebugSubject, nav.DebugSnapshot().ToProto(), logTimeNow()); err != nil {
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
