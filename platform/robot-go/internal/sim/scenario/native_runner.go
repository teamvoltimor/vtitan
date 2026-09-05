package scenario

import (
	"context"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"slices"
	"strings"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navigator"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/parking"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/startconditions"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/waypoints"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/widthbelief"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/collision"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/corpus"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/harness"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/kinematics"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/sensorerrors"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/visionsim"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/simgen/generate"
)

// NativeRunner implements Runner with a fully Go-native closed-loop
// simulation, replacing SubprocessRunner (the frozen Python oracle). It builds
// the harness + collision.TrackModel from scenario metadata, drives the real
// navigator.Navigator (sighted — direction known), and scores the outcome
// into a Result.
//
// Obstacles Challenge scenarios (metadata's sign_positions) are supported:
// signs become both LIDAR-visible collision obstacles
// (collision.ObstacleBox) and SignRouter targets, with a
// internal/sim/visionsim-emulated camera feeding SignRouter's per-tick
// deformation the same way a real detection would.
//
// Parking-lot scenarios (metadata's parking_lot) ARE acted on: the two
// marker fins become LIDAR-visible, unforgivable (collision.SurfaceParkingLot,
// WRO 9.24.7) collision obstacles, and a parking.ParkController drives the
// post-final-lap maneuver into the bay -- see parkBlocksFromMetadata,
// parkControllerFromMetadata, and navigator.Navigator's own handleFinish/
// shouldEngageParking. The final pose is scored against the WRO 15/7/0 point
// tiers (parking.ScorePark) into Result.ParkPoints. A boxed-in-bay START
// (BayExit) is a separate maneuver from parking IN, and is not driven by
// this runner -- see internal/nav/bayexit's own doc comment.
//
// BLIND mode (NativeRunnerConfig.Blind) withholds the scenario's truth from
// the navigator the way a real round does: Direction is nil, so the blind
// bootstrap infers it from LIDAR, and the initial path is planned from the
// challenge's WIDTH PRIOR rather than the true corridor widths, then
// corrected by widthbelief.Layout as the estimator measures each corridor.
// Sighted mode (the default) keeps taking both from metadata.
type NativeRunner struct {
	cfg harness.Config
	// Resolved once at construction rather than per scenario: a corpus sweep
	// runs hundreds of scenarios, and re-reading the same TOML tree for each
	// would be both wasteful and a source of per-scenario divergence if a
	// file changed mid-sweep.
	navCfg     navigator.Config
	ctrlCfg    controllers.Config
	wpCfg      waypoints.Config
	srCfg      signrouter.Config
	startCfg   startconditions.Config
	kinParams  kinematics.Params
	collCfg    collision.Config
	recordRoot string
	// recGeom is chassis geometry only the BAG needs -- wheel size, steering
	// limit, LIDAR mount. None of it belongs in kinematics.Params: a bicycle
	// model turns on wheelbase, not wheel size, and knows nothing of where a
	// sensor is bolted.
	recGeom  recorderGeometry
	seed     uint64
	maxSteps int
	blind    bool
}

// NativeRunnerConfig configures a NativeRunner.
type NativeRunnerConfig struct {
	// Harness overrides the harness Config; nil uses harness.DefaultConfig().
	Harness *harness.Config
	// Seed is the RNG seed for LIDAR noise/dropout (parity default 0, matching
	// the Python np.random.default_rng(0)).
	Seed uint64
	// MaxSteps bounds a single run, in TICKS. Zero derives it from
	// DefaultMaxRunS at the resolved control rate, which is what a caller
	// almost always wants: a step budget is a frame count, and a frame count
	// silently means a different amount of DRIVING at a different loop rate
	// -- 4000 ticks is 200 s at 20 Hz and 80 s at 50 Hz, short enough to
	// time out runs that were finishing comfortably. Set it explicitly only
	// to bound ticks as such.
	MaxSteps int
	// Blind withholds the scenario's direction and corridor widths from the
	// navigator, which must then infer both from LIDAR -- the way a real
	// round works. Defaults false (sighted), which is what every existing
	// corpus sweep measured, so turning this on is an explicit A/B rather
	// than a silent change to what "the native runner" means.
	Blind bool
	// ConfigRoot is the repo root the shipped TOML tree is read from, and
	// HardwareProfiles names one profile per component (drive motor,
	// steering servo) to overlay on it -- the two inputs every nav package's
	// own ConfigFor already takes.
	//
	// An empty ConfigRoot keeps every package on its Go literal defaults.
	// That is NOT the shipped robot: the base navigation tree caps speed at
	// max_mps 0.156 and carries no Open speed ladder at all, while the
	// ladder that Open's results were measured against (0.26/0.38/0.50)
	// lives only in profiles/rev-hd-hex-motor-6000rpm/motion/speed.toml. A
	// sweep run without these is measuring a robot that drives a third as
	// fast as the one the numbers describe, so any comparison against a
	// Python baseline must set both.
	ConfigRoot       string
	HardwareProfiles []string
	// RecordRoot, when non-empty, writes each scenario's run to an MCAP bag
	// under <RecordRoot>/<scenario ID>/. Off by default: a 640-case sweep
	// records 640 bags, which is worth it when debugging a specific failure
	// and pure overhead when scoring.
	RecordRoot string
	// SensorErrors perturbs what the robot knows about ITSELF -- its start
	// pose and its heading -- on top of whatever Blind withholds about the
	// track. Zero (a perfect robot) is what every corpus number here was
	// measured on, and is Python's default too.
	SensorErrors sensorerrors.Errors
}

// ControlDt returns the simulation timestep (s). It resolves the effective
// harness Config (same precedence as NewNativeRunner) and delegates to its
// ControlDt, so a caller-supplied ControlHz is honoured instead of hardcoded.
func (c NativeRunnerConfig) ControlDt() float64 {
	hc := harness.DefaultConfig()
	if c.Harness != nil {
		hc = *c.Harness
	}
	return hc.ControlDt()
}

// NewNativeRunner builds a NativeRunner.
func NewNativeRunner(cfg NativeRunnerConfig) *NativeRunner {
	hc := harness.DefaultConfig()
	if cfg.Harness != nil {
		hc = *cfg.Harness
	}
	// Applied after the harness override so --sensor-error flags compose
	// with a caller-supplied Config rather than being erased by it.
	if cfg.SensorErrors.Any() {
		hc.SensorErrors = cfg.SensorErrors
	}
	// Every ConfigFor already treats an empty root as "use the literal
	// defaults" and logs its own reason on a load failure, so there is no
	// branch here: passing "" reproduces the previous all-defaults runner
	// exactly. The logger is discarded for the same reason newBlindSetup
	// discards its own -- a sweep runs hundreds of scenarios and a
	// per-package load line from each would bury the report.
	logger := discardingLogger()
	kinParams := kinematics.ParamsFor(logger, cfg.ConfigRoot, cfg.HardwareProfiles)
	navCfg := navigator.ConfigFor(logger, cfg.ConfigRoot, cfg.HardwareProfiles)

	// The SIMULATED gateway must step at the same rate the navigator thinks
	// it is running at. harness.Config carried its own hardcoded 20.0, which
	// is the exact divergence control.toml warns about: the controller
	// rate-limits steering with dt, so the two drifting apart would not raise
	// anything -- it would tune the robot against a cadence the simulator
	// never ran at. Taking it from the same loaded config makes that
	// impossible rather than merely unlikely.
	if cfg.ConfigRoot != "" && navCfg.ControlHz > 0 {
		hc.ControlHz = navCfg.ControlHz
	}

	// AFTER the rate is resolved: the budget is a duration, so the tick count
	// it becomes depends on the rate the run will actually step at.
	maxSteps := cfg.MaxSteps
	if maxSteps <= 0 {
		maxSteps = int(math.Round(DefaultMaxRunS * hc.ControlHz))
	}

	return &NativeRunner{
		cfg:        hc,
		recordRoot: cfg.RecordRoot,
		recGeom:    recorderGeometryFor(logger, cfg.ConfigRoot, cfg.HardwareProfiles, kinParams.MaxSteerRad),
		navCfg:     navigator.ConfigFor(logger, cfg.ConfigRoot, cfg.HardwareProfiles),
		ctrlCfg:    controllers.ConfigFor(logger, cfg.ConfigRoot, cfg.HardwareProfiles),
		wpCfg:      waypoints.ConfigFor(logger, cfg.ConfigRoot),
		srCfg:      signrouter.ConfigFor(logger, cfg.ConfigRoot),
		startCfg:   startconditions.ConfigFor(logger, cfg.ConfigRoot),
		kinParams:  kinematics.ParamsFor(logger, cfg.ConfigRoot, cfg.HardwareProfiles),
		collCfg:    collision.ConfigFor(logger, cfg.ConfigRoot),
		seed:       cfg.Seed,
		maxSteps:   maxSteps,
		blind:      cfg.Blind,
	}
}

// DefaultMaxRunS is the wall-clock budget a single scenario gets before it
// is scored as timed out. 200 s, comfortably past the WRO round limit of
// 180 s, so a run that would have been over time on the mat is still driven
// far enough to see what it did rather than cut off mid-recovery.
const DefaultMaxRunS = 200.0

// Run builds and drives one scenario, returning a Result.
func (r *NativeRunner) Run(_ context.Context, sc corpus.Scenario) (Result, error) {
	meta, err := loadMetadata(sc.MetadataPath)
	if err != nil {
		return Result{}, fmt.Errorf("native runner: loading %s: %w", sc.MetadataPath, err)
	}

	geom, startPose, path, err := r.buildScenario(meta)
	if err != nil {
		return Result{}, fmt.Errorf("native runner: building %s: %w", sc.ID, err)
	}

	signs := signsFromMetadata(meta)
	axisAlignTolerance := r.collCfg.AxisAlignTolerance
	obstacleSpecs := make([]collision.ObstacleSpec, len(signs))
	for i, sign := range signs {
		obstacleSpecs[i] = collision.ObstacleSpec{
			CX: sign.X, CY: sign.Y, Length: signObstacleWidthM, Width: signObstacleDepthM,
		}
	}
	obstacleSpecs = append(obstacleSpecs, parkBlocksFromMetadata(meta)...)

	track := collision.NewTrackModel(collision.NewTrackModelParams{
		Geometry:           geom,
		MinCoordM:          0.0,
		MaxCoordM:          r.cfg.TrackMaxCoordM,
		Obstacles:          collision.ObstaclesFromSpecs(obstacleSpecs, axisAlignTolerance),
		LidarSeesObstacles: len(obstacleSpecs) > 0,
		CollisionMarginM:   r.cfg.CollisionMarginM,
	})

	kin := kinematics.NewAckermannKinematics(r.kinParams)

	gw := harness.NewSimHardwareGateway(
		r.cfg, track,
		kinematics.AckermannState{X: startPose.X, Y: startPose.Y, Yaw: startPose.Yaw},
		kin, r.seed,
	)

	// A non-nil SignRouter is what identifies the Obstacles Challenge to
	// Navigator itself (widens the collision-avoidance contact zone, enables
	// the sign-lane path planner) -- see navigator.New's own doc comment on
	// p.SignRouter -- so it is only built when the scenario actually has
	// signs, never as an always-present-but-empty router.
	var signRouter *signrouter.SignRouter
	var vision navigator.VisionGateway
	if len(signs) > 0 {
		signRouter, err = signrouter.NewSignRouter(signs, r.srCfg, startPose.Direction)
		if err != nil {
			return Result{}, fmt.Errorf("native runner: building sign router %s: %w", sc.ID, err)
		}
		vision = &simVisionGateway{gw: gw, signs: signs, cfg: visionConfigFor(r.cfg)}
	}

	targetLaps := defaultLaps(meta)
	pc := parkControllerFromMetadata(meta, startPose.Section, startPose.Direction)

	// Blind withholds BOTH the direction and the layout. The direction goes
	// to nil so navigator's own bootstrap infers it from LIDAR; the path is
	// replaced with one planned from the challenge's width prior and the
	// assumed start, which widthbelief.Layout then corrects each tick.
	//
	// The blind round's DIRECTION is still the scenario's for planning
	// purposes: newBlindSetup needs one to lay out a lap, and the real robot
	// likewise plans only once its own inference has settled. What the
	// navigator is not told is the answer -- it has to reach it itself before
	// it follows this path at all.
	var layout *widthbelief.Layout
	direction := func() *trackmodel.Direction { d := startPose.Direction; return &d }()
	if r.blind {
		blind, blindErr := newBlindSetup(
			plannerBaseFor(meta, r.cfg),
			startPose.Direction,
			len(signs) > 0,
			r.cfg,
			r.wpCfg,
			r.startCfg,
			blindCenterBiasM(len(signs) > 0),
		)
		if blindErr != nil {
			return Result{}, fmt.Errorf("native runner: %s: %w", sc.ID, blindErr)
		}
		path = blind.Waypoints
		layout = blind.Layout
		direction = nil
	}

	nav, err := navigator.New(navigator.Params{
		Gateway:           gw,
		Vision:            vision,
		Waypoints:         path,
		Direction:         direction,
		NumLaps:           targetLaps,
		Config:            r.navCfg,
		ControllersConfig: r.ctrlCfg,
		SignRouterConfig:  r.srCfg,
		SignRouter:        signRouter,
		ParkController:    pc,
	})
	if err != nil {
		return Result{}, fmt.Errorf("native runner: building navigator %s: %w", sc.ID, err)
	}

	rec, err := newSimRecorder(r.recordRoot, sc.ID, r.recGeom)
	if err != nil {
		return Result{}, fmt.Errorf("native runner: %s: %w", sc.ID, err)
	}
	defer func() {
		if closeErr := rec.close(); closeErr != nil && err == nil {
			err = closeErr
		}
	}()

	return r.loop(sc, gw, nav, track, targetLaps, startPose, layout, rec)
}

// simVisionGateway implements navigator.VisionGateway by emulating sign
// detections from the simulation's TRUE chassis pose each tick, matching
// how ScenarioSimulator wires vision_emulator.emulate_sign_observations
// into the Python gateway. Ground-truth pose only (no believed-pose
// reprojection): the native runner's Localize is unsupported (see
// harness.Config.Localize's doc comment), so there is no separate believed
// pose to diverge from the true one yet.
type simVisionGateway struct {
	gw    simGateway
	signs []signrouter.SignSpec
	cfg   visionsim.Config
}

func (v *simVisionGateway) GetVisionDetections() ([]signrouter.TrafficSignObservation, bool) {
	st := v.gw.State()
	obs := visionsim.EmulateSignObservations(v.signs, st.X, st.Y, st.Yaw, v.cfg, nil)
	return obs, len(obs) > 0
}

// visionConfigFor derives visionsim.Config from the harness Config's own
// DetectionConfidence, keeping the camera HFOV/range defaults (visionsim
// owns those; harness.Config does not mirror RobotSpecs' camera constants).
func visionConfigFor(cfg harness.Config) visionsim.Config {
	vc := visionsim.DefaultConfig()
	vc.DetectionConfidence = cfg.DetectionConfidence
	return vc
}

// signObstacleWidthM/signObstacleDepthM mirror track.toml's [sign]
// width/depth (TrafficSignSpecs.WIDTH/DEPTH) -- both 0.05 m. No Go mirror
// of TrafficSignSpecs exists yet beyond signrouter.DefaultSignWidthM
// (the lane-offset consumer of the same width value); depth has no
// existing home, so both are named here where the sim-obstacle geometry
// that needs them lives.
const (
	signObstacleWidthM = signrouter.DefaultSignWidthM
	signObstacleDepthM = 0.05

	// signPlacementCircleDiameterM mirrors track.toml's [sign]
	// placement_circle_diameter (TrafficSignSpecs.PLACEMENT_CIRCLE_DIAMETER):
	// the circle a pillar is placed within on the mat. Touching a pillar is
	// NOT a failure (WRO 9.20) -- the run stays valid as long as any corner
	// of its square is still inside this circle.
	signPlacementCircleDiameterM = 0.085
)

// maxLegalSignDisplacementM is how far a pillar may be pushed and still have
// a corner in its placement circle, matching
// TrafficSignSpecs.MAX_LEGAL_DISPLACEMENT_M. Derived, not measured: the
// corner that survives longest is the one trailing the push, so the bound is
// the displacement at which even that corner leaves the circle.
var maxLegalSignDisplacementM = math.Sqrt(
	math.Pow(signPlacementCircleDiameterM/2, 2)-math.Pow(signObstacleWidthM/2, 2),
) + signObstacleWidthM/2

// signsFromMetadata builds the ground-truth SignSpec list for an Obstacles
// Challenge scenario, matching the sign half of track_model.py's
// obstacles_from_metadata (the parking-block half is deliberately not
// ported here -- see collision.ObstacleBox's is_parking_lot gap, tracked
// separately). Empty for Open Challenge metadata (no sign_positions key),
// exactly as Python's obstacles_from_metadata returns an empty list for it.
func signsFromMetadata(meta generate.Metadata) []signrouter.SignSpec {
	if len(meta.SignPositions) == 0 {
		return nil
	}
	signs := make([]signrouter.SignSpec, len(meta.SignPositions))
	for i, s := range meta.SignPositions {
		color := signrouter.SignColorGreen
		if s.Color == "red" {
			color = signrouter.SignColorRed
		}
		signs[i] = signrouter.SignSpec{X: s.X, Y: s.Y, Color: color}
	}
	return signs
}

// parkBlocksFromMetadata builds the two parking-lot marker fins as
// collision.ObstacleSpec, matching the parking half of obstacles_from_metadata
// (track_model.py) -- the sign half lives in signsFromMetadata. Empty when
// the scenario has no parking lot.
func parkBlocksFromMetadata(meta generate.Metadata) []collision.ObstacleSpec {
	if meta.ParkingLot == nil {
		return nil
	}
	lot := meta.ParkingLot
	return []collision.ObstacleSpec{
		{
			CX: lot.Block1Position.X, CY: lot.Block1Position.Y,
			Length: parking.DefaultParkingLotLengthM, Width: parking.DefaultParkingLotWidthM,
			Yaw: lot.Block1Yaw, IsParkingLot: true,
		},
		{
			CX: lot.Block2Position.X, CY: lot.Block2Position.Y,
			Length: parking.DefaultParkingLotLengthM, Width: parking.DefaultParkingLotWidthM,
			Yaw: lot.Block2Yaw, IsParkingLot: true,
		},
	}
}

// parkControllerFromMetadata builds a parking.ParkController from scenario
// metadata, matching park_controller_from_metadata. nil when the scenario
// has no parking lot.
func parkControllerFromMetadata(
	meta generate.Metadata, section trackmodel.Section, direction trackmodel.Direction,
) *parking.ParkController {
	if meta.ParkingLot == nil {
		return nil
	}
	lot := &parking.ParkingLot{
		Block1: parking.BlockPosition{X: meta.ParkingLot.Block1Position.X, Y: meta.ParkingLot.Block1Position.Y},
		Block2: parking.BlockPosition{X: meta.ParkingLot.Block2Position.X, Y: meta.ParkingLot.Block2Position.Y},
	}
	return parking.ParkControllerFromMetadata(lot, section, direction, parking.DefaultConfig())
}

// simGateway is the simulation-only hardware surface the native runner's
// closed-loop helpers need: the navigator-facing scan source plus the
// physics advance/state/collision methods of harness.SimHardwareGateway.
// Declared at the point of use (go-architect §4) so NativeRunner stays
// testable against a fake instead of coupled to the concrete gateway.
type simGateway interface {
	controllers.PoseSource
	// Sensors is what the blind layout-belief loop reads and re-seeds --
	// embedded rather than restated so the two cannot drift apart.
	widthbelief.Sensors
	// State returns the current simulated chassis state.
	State() kinematics.AckermannState
	// Advance integrates the simulation by dt seconds.
	Advance(dt float64)
	// Collided reports whether the chassis has hit a wall this step.
	Collided() bool
	// CollisionXY returns the contact point of the latest collision, if any.
	CollisionXY() (float64, float64)
}

// loop runs the control loop until terminal (laps / collision / timeout /
// stuck) and scores the Result.
func (r *NativeRunner) loop(
	sc corpus.Scenario,
	gw simGateway,
	nav *navigator.Navigator,
	track *collision.TrackModel,
	targetLaps int,
	startPose scenarioStart,
	layout *widthbelief.Layout,
	rec *simRecorder,
) (Result, error) {
	dt := r.cfg.ControlDt()

	var steps int
	var contactCount int
	var distanceM float64
	var maxSpeedMPS float64
	var minRangeM = math.Inf(1)
	prevX, prevY := gw.State().X, gw.State().Y
	nudge := newSignNudgeState(prevX, prevY)

	// No-progress bailout (mirrors the Python run's NO_PROGRESS_* policy). The
	// unported NO_PROGRESS_WINDOW_S / NO_PROGRESS_DISPLACEMENT_M constants are
	// hardcoded parity defaults here (plan §2).
	noProgressWindow := int(math.Round(2.0 / dt))
	noProgressDisp := 0.05
	anchorX, anchorY := prevX, prevY
	anchorStep := 0

	for steps < r.maxSteps {
		nav.Step()
		// Driven every tick, not only when the estimator speaks: a deferred
		// belief is released by the robot LEAVING a corridor, so the tick that
		// applies it is usually one with no new reading at all. A nil layout
		// (sighted) is a no-op.
		layout.Update(nav, gw, nav.Direction())
		if rec != nil {
			scan, scanOK := gw.GetLidarScan()
			// The REPORTED yaw, not the true one: /imu/data must carry what
			// the robot believes, so a run with sensor errors shows the belief
			// diverging from the ground-truth transform.
			reportedYaw := gw.State().Yaw
			if pose, poseOK := gw.GetCurrentPose(); poseOK {
				reportedYaw = pose.Yaw
			}
			if err := rec.tick(scan, scanOK, nav, gw.State(), reportedYaw, dt); err != nil {
				return Result{}, err
			}
		}
		gw.Advance(dt)
		steps++

		st := gw.State()
		stepDist := math.Hypot(st.X-prevX, st.Y-prevY)
		distanceM += stepDist
		prevX, prevY = st.X, st.Y
		if st.V > maxSpeedMPS {
			maxSpeedMPS = st.V
		}
		if scan, ok := gw.GetLidarScan(); ok {
			for _, rng := range scan.RangesM {
				if !math.IsInf(rng, 0) && rng < minRangeM {
					minRangeM = rng
				}
			}
		}

		if gw.Collided() {
			contactCount++
		}

		// Terminal surface: any wall contact ends the run. A traffic-sign
		// touch does not -- WRO 9.20 allows the pillar to be nudged, and the
		// run stands as long as no sign's accumulated push exceeds
		// maxLegalSignDisplacementM (see signNudgeState.score). A parking-lot
		// fin carries no such leniency (9.24.7): SurfaceParkingLot never
		// reaches signNudgeState.score's leniency branch (it only special-
		// cases SurfaceObstacle), so it stays terminal here unconditionally.
		surface := track.ContactSurfaceAt(st.X, st.Y, st.Yaw, r.cfg.ChassisLengthM, r.cfg.ChassisWidthM)
		surface = nudge.score(track, surface, st.X, st.Y, st.Yaw, r.cfg.ChassisLengthM, r.cfg.ChassisWidthM)
		if surface != collision.SurfaceNone {
			res := r.score(sc, gw, nav, steps, dt, distanceM, maxSpeedMPS, minRangeM, contactCount, targetLaps, surface, false)
			return res, nil
		}

		// Lap completion alone isn't terminal when a ParkController is
		// attached: the round only ends once it is also done (parked
		// cleanly or gave up), matching the Python run loop's
		// `laps_completed >= num_laps and (pc is None or pc.is_done)`.
		pc := nav.ParkController()
		if nav.LapsCompleted() >= targetLaps && (pc == nil || pc.IsDone()) {
			res := r.score(sc, gw, nav, steps, dt, distanceM, maxSpeedMPS, minRangeM, contactCount, targetLaps, collision.SurfaceNone, false)
			return res, nil
		}

		// No-progress check.
		if math.Hypot(st.X-anchorX, st.Y-anchorY) >= noProgressDisp {
			anchorX, anchorY = st.X, st.Y
			anchorStep = steps
		} else if (steps - anchorStep) >= noProgressWindow {
			res := r.score(sc, gw, nav, steps, dt, distanceM, maxSpeedMPS, minRangeM, contactCount, targetLaps, collision.SurfaceNone, true)
			return res, nil
		}
	}

	// Timed out.
	res := r.score(sc, gw, nav, steps, dt, distanceM, maxSpeedMPS, minRangeM, contactCount, targetLaps, collision.SurfaceNone, false)
	res.TimedOut = nav.LapsCompleted() < targetLaps
	return res, nil
}

func (r *NativeRunner) score(
	sc corpus.Scenario,
	gw simGateway,
	nav *navigator.Navigator,
	steps int,
	dt float64,
	distanceM, maxSpeedMPS, minRangeM float64,
	contactCount int,
	targetLaps int,
	surface collision.ContactSurface,
	stuck bool,
) Result {
	// collided is derived from the surface rather than passed alongside it,
	// so the two can never disagree about whether the run ended in contact.
	collided := surface != collision.SurfaceNone
	laps := nav.LapsCompleted()
	st := gw.State()
	cx, cy := gw.CollisionXY()

	// PassSideViolationSigns/PassSideViolation are only ever populated by an
	// Obstacles Challenge run (nav.SignRouter() nil for Open), matching
	// SimResult's own fields -- an empty sign-router-less run reports zero
	// violations, not "unknown."
	var wrongSideSigns []int
	if sr := nav.SignRouter(); sr != nil {
		for index := range sr.WrongSideViolations() {
			wrongSideSigns = append(wrongSideSigns, index)
		}
		slices.Sort(wrongSideSigns)
	}
	passSideViolation := len(wrongSideSigns) > 0

	success := !collided && !stuck && !passSideViolation && !resTimedOut(steps, r.maxSteps, laps, targetLaps)

	// Parked is nil for a scenario with no parking lot, matching
	// SimResult.parked's None. ParkPoints additionally scores the final
	// pose against the WRO 15/7/0 tiers (parking.ScorePark) -- an addition
	// beyond Python's plain boolean, since that scorer was ported standalone
	// and never wired into SimResult either.
	var parked *bool
	var parkPoints *int
	if pc := nav.ParkController(); pc != nil {
		p := pc.IsDone() && !pc.IsTimedOut()
		parked = &p
		score := parking.ScorePark(st.X, st.Y, st.Yaw, pc.Zone(), parking.DefaultConfig())
		points := score.Points
		parkPoints = &points
	}

	return Result{
		TerminalSurface:        surface.String(),
		Scenario:               sc.ID,
		PassSideViolationSigns: wrongSideSigns,
		CollisionXY:            []float64{cx, cy},
		FinalPose:              []float64{st.X, st.Y, st.Yaw},
		Parked:                 parked,
		ParkPoints:             parkPoints,
		SimTimeS:               float64(steps) * dt,
		DistanceM:              distanceM,
		MaxSpeedMPS:            maxSpeedMPS,
		AvgSpeedMPS:            avgSpeed(distanceM, steps, dt),
		MinLidarRangeM:         orZero(minRangeM),
		TargetLaps:             targetLaps,
		LapsCompleted:          laps,
		Steps:                  steps,
		ContactCount:           contactCount,
		Collided:               collided,
		PassSideViolation:      passSideViolation,
		TimedOut:               resTimedOut(steps, r.maxSteps, laps, targetLaps),
		Stuck:                  stuck,
		Success:                success,
		OverTime:               false,
	}
}

func resTimedOut(steps, maxSteps, laps, target int) bool {
	return steps >= maxSteps && laps < target
}

func avgSpeed(distanceM float64, steps int, dt float64) float64 {
	if steps <= 0 || dt <= 0 {
		return 0
	}
	return distanceM / (float64(steps) * dt)
}

func orZero(v float64) float64 {
	if math.IsInf(v, 0) || math.IsNaN(v) {
		return 0
	}
	return v
}

// signNudgeState accumulates each sign's push-displacement across ticks,
// matching scoring.py's ScenarioSimulator._score_obstacle_contact /
// _sign_push / _prev_contact_xy. One instance per run.
type signNudgeState struct {
	prevX, prevY float64
	push         map[int]float64
}

// newSignNudgeState seeds the reference point at the run's start pose,
// matching _prev_contact_xy's __init__ assignment.
func newSignNudgeState(x, y float64) *signNudgeState {
	return &signNudgeState{prevX: x, prevY: y, push: make(map[int]float64)}
}

// score downgrades a legal pillar nudge (WRO 9.20) to a non-collision surface,
// matching _score_obstacle_contact. Fin/wall surfaces pass through untouched
// -- only SurfaceObstacle (traffic signs; no parking fin ever reaches this
// runner yet, see signsFromMetadata) gets the leniency.
//
// The reference point advances EVERY call, not only while touching: updating
// it only during contact would make the accumulated displacement the
// distance since the last touch, so a sign brushed twice a metre apart would
// accumulate that whole metre as if it had been pushed through it.
func (s *signNudgeState) score(track *collision.TrackModel, surface collision.ContactSurface, x, y, yaw, length, width float64) collision.ContactSurface {
	dx, dy := x-s.prevX, y-s.prevY
	s.prevX, s.prevY = x, y
	if surface != collision.SurfaceObstacle {
		return surface
	}
	for index := range track.ObstacleDisplacements(x, y, yaw, length, width) {
		// Only the component of travel pointing AT the sign moves it: a
		// chassis sliding past a sign it is brushing covers distance
		// without pushing it anywhere.
		center, ok := track.ObstacleCenter(index)
		if !ok {
			continue
		}
		toX, toY := center.X-x, center.Y-y
		norm := math.Hypot(toX, toY)
		if norm <= 0.0 {
			continue
		}
		push := (dx*toX + dy*toY) / norm
		if push > 0.0 {
			s.push[index] += push
		}
	}
	for _, push := range s.push {
		if push > maxLegalSignDisplacementM {
			return surface
		}
	}
	// Touched, but still inside its placement circle: not a collision.
	return collision.SurfaceNone
}

// scenarioStart bundles the parsed spawn pose + travel direction + starting
// section, matching ScenarioSimulator's believed_start (the fields
// park_controller_from_metadata needs to build the parking-lot geometry).
type scenarioStart struct {
	X, Y, Yaw float64
	Direction trackmodel.Direction
	Section   trackmodel.Section
}

// loadMetadata reads and parses a *_metadata.json file into generate.Metadata
// -- the SAME struct cmd/simgen writes (internal/simgen/generate), not a
// separately maintained mirror of its schema. The two used to be independent,
// hand-kept-in-sync definitions (one per Go module); unified once simgen
// joined this module, so a schema change in one can no longer silently drift
// from the other.
func loadMetadata(path string) (generate.Metadata, error) {
	raw, err := os.ReadFile(filepath.Clean(path))
	if err != nil {
		return generate.Metadata{}, fmt.Errorf("reading %s: %w", path, err)
	}
	var meta generate.Metadata
	if err := json.Unmarshal(raw, &meta); err != nil {
		return generate.Metadata{}, fmt.Errorf("parsing %s: %w", path, err)
	}
	return meta, nil
}

// buildScenario derives the track geometry, spawn pose, and the planned path
// from scenario metadata.
//
// The path comes from the REAL planner (waypoints.CalculateWaypoints), the
// same one the deployed navigator and the blind setup use. It used to come
// from centerlineLoop, a rectangle offset half a corridor width from each
// wall -- an approximation with SQUARE corners, no centreline bias and no
// arcs, which is not a path any robot in this project has ever been asked to
// drive. Measured on the full 640-case Open space: the approximation scored
// 37/640 against Python's 638/640, with the failures concentrated at corner
// entry, because a square corner asks for a turn no Ackermann chassis can
// execute.
func (r *NativeRunner) buildScenario(meta generate.Metadata) (trackmodel.CorridorGeometry, scenarioStart, []trackmodel.Waypoint, error) {
	cfg := r.cfg
	sectionsByName := map[string]trackmodel.Section{
		"north": trackmodel.North,
		"south": trackmodel.South,
		"east":  trackmodel.East,
		"west":  trackmodel.West,
	}

	widthsM := map[trackmodel.Section]float64{}
	for name, which := range sectionsByName {
		wm, ok := meta.CorridorWidths[name]
		if !ok {
			return trackmodel.CorridorGeometry{}, scenarioStart{}, nil,
				fmt.Errorf("metadata missing corridor width for %q", name)
		}
		widthsM[which] = float64(wm.WidthMM) / 1000.0
	}

	geom := trackmodel.CorridorGeometryFromWidths(widthsM, cfg.TrackMaxCoordM)

	dir := trackmodel.Counterclockwise
	if meta.StartingConditions.Direction == "clockwise" {
		dir = trackmodel.Clockwise
	}

	section, ok := sectionsByName[strings.ToLower(meta.StartingConditions.Section)]
	if !ok {
		return trackmodel.CorridorGeometry{}, scenarioStart{}, nil,
			fmt.Errorf("metadata has unknown starting section %q", meta.StartingConditions.Section)
	}

	start := scenarioStart{
		X:         meta.StartingConditions.Position.X,
		Y:         meta.StartingConditions.Position.Y,
		Yaw:       meta.StartingConditions.Yaw,
		Direction: dir,
		Section:   section,
	}

	planned := plannerBaseFor(meta, cfg)
	planned.Geometry = geom
	planned.Starting = planned.Starting.ReplannedAt(
		&dir, section, trackmodel.Waypoint{X: start.X, Y: start.Y}, start.Yaw,
	)
	// A SIGHTED round has been handed the true widths, so every corridor is
	// confirmed and none takes the unconfirmed inner bias -- the opposite of
	// newBlindSetup, which plans everything unconfirmed.
	path, err := waypoints.CalculateWaypoints(
		planned, 1, r.wpCfg, sightedCenterBiasM(meta), waypoints.AllConfirmed(),
	)
	if err != nil {
		return trackmodel.CorridorGeometry{}, scenarioStart{}, nil,
			fmt.Errorf("planning the believed path: %w", err)
	}
	return geom, start, path, nil
}

// sightedCenterBiasM is the planning bias for a sighted round: nil on Open,
// which takes waypoints.toml's narrow/wide split, and the uniform Obstacles
// override otherwise. Mirrors blindCenterBiasM, keyed off the same fact
// (does this scenario carry signs) rather than off the metadata's
// challenge_type string, so a mislabeled fixture cannot plan one challenge
// with the other's bias.
func sightedCenterBiasM(meta generate.Metadata) *float64 {
	return blindCenterBiasM(len(meta.SignPositions) > 0)
}

// defaultLaps returns the Open Challenge default lap count.
func defaultLaps(_ generate.Metadata) int { return navigator.DefaultOpenChallengeLaps }

// compile-time assertion that NativeRunner satisfies Runner.
var _ Runner = (*NativeRunner)(nil)
