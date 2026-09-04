package scenario

import (
	"context"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"slices"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navigator"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/collision"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/corpus"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/harness"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/kinematics"
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
// deformation the same way a real detection would. Parking-lot scenarios
// (metadata's parking_lot) are parsed but NOT yet acted on: ParkController
// and BayExit are not wired into navigator.Navigator itself (see
// internal/nav/navigator/doc.go's "Scope and deviations" section and
// internal/nav/bayexit's own doc comment on the missing Gateway
// wheel-odometry accessor), so an in-bay-start or drive-past-the-lot
// scenario still runs, just without any parking behavior.
//
// Scope note: blind mode (direction inference + corridor-width estimation +
// believed-wall relocalization) is intentionally out of scope here --
// Direction is always taken from scenario-truth metadata, never nil.
// SubprocessRunner remains the parity oracle for blind-mode and parking
// scenarios until the native runner is extended further.
type NativeRunner struct {
	cfg      harness.Config
	seed     uint64
	maxSteps int
}

// NativeRunnerConfig configures a NativeRunner.
type NativeRunnerConfig struct {
	// Harness overrides the harness Config; nil uses harness.DefaultConfig().
	Harness *harness.Config
	// Seed is the RNG seed for LIDAR noise/dropout (parity default 0, matching
	// the Python np.random.default_rng(0)).
	Seed uint64
	// MaxSteps bounds a single run (parity default 4000 ≈ 200 s at 20 Hz).
	MaxSteps int
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
	maxSteps := cfg.MaxSteps
	if maxSteps <= 0 {
		maxSteps = 4000
	}
	return &NativeRunner{cfg: hc, seed: cfg.Seed, maxSteps: maxSteps}
}

// Run builds and drives one scenario, returning a Result.
func (r *NativeRunner) Run(_ context.Context, sc corpus.Scenario) (Result, error) {
	meta, err := loadMetadata(sc.MetadataPath)
	if err != nil {
		return Result{}, fmt.Errorf("native runner: loading %s: %w", sc.MetadataPath, err)
	}

	geom, startPose, waypoints, err := buildScenario(meta, r.cfg)
	if err != nil {
		return Result{}, fmt.Errorf("native runner: building %s: %w", sc.ID, err)
	}

	signs := signsFromMetadata(meta)
	axisAlignTolerance := collision.DefaultConfig().AxisAlignTolerance
	obstacleSpecs := make([]collision.ObstacleSpec, len(signs))
	for i, sign := range signs {
		obstacleSpecs[i] = collision.ObstacleSpec{
			CX: sign.X, CY: sign.Y, Length: signObstacleWidthM, Width: signObstacleDepthM,
		}
	}

	track := collision.NewTrackModel(collision.NewTrackModelParams{
		Geometry:           geom,
		MinCoordM:          0.0,
		MaxCoordM:          r.cfg.TrackMaxCoordM,
		Obstacles:          collision.ObstaclesFromSpecs(obstacleSpecs, axisAlignTolerance),
		LidarSeesObstacles: len(signs) > 0,
		CollisionMarginM:   r.cfg.CollisionMarginM,
	})

	kin := kinematics.NewAckermannKinematics(defaultKinematicsParams())

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
		signRouterCfg := signrouter.DefaultConfig()
		signRouter, err = signrouter.NewSignRouter(signs, signRouterCfg, startPose.Direction)
		if err != nil {
			return Result{}, fmt.Errorf("native runner: building sign router %s: %w", sc.ID, err)
		}
		vision = &simVisionGateway{gw: gw, signs: signs, cfg: visionConfigFor(r.cfg)}
	}

	targetLaps := defaultLaps(meta)
	navCfg := navigator.ConfigFor(nil, "", nil)
	nav, err := navigator.New(navigator.Params{
		Gateway:           gw,
		Vision:            vision,
		Waypoints:         waypoints,
		Direction:         func() *trackmodel.Direction { d := startPose.Direction; return &d }(),
		NumLaps:           targetLaps,
		Config:            navCfg,
		ControllersConfig: controllers.DefaultConfig(),
		SignRouterConfig:  signrouter.DefaultConfig(),
		SignRouter:        signRouter,
	})
	if err != nil {
		return Result{}, fmt.Errorf("native runner: building navigator %s: %w", sc.ID, err)
	}

	return r.loop(sc, gw, nav, track, targetLaps, startPose)
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
)

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

// simGateway is the simulation-only hardware surface the native runner's
// closed-loop helpers need: the navigator-facing scan source plus the
// physics advance/state/collision methods of harness.SimHardwareGateway.
// Declared at the point of use (go-architect §4) so NativeRunner stays
// testable against a fake instead of coupled to the concrete gateway.
type simGateway interface {
	controllers.PoseSource
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
) (Result, error) {
	dt := r.cfg.ControlDt()

	var steps int
	var contactCount int
	var distanceM float64
	var maxSpeedMPS float64
	var minRangeM = math.Inf(1)
	prevX, prevY := gw.State().X, gw.State().Y

	// No-progress bailout (mirrors the Python run's NO_PROGRESS_* policy). The
	// unported NO_PROGRESS_WINDOW_S / NO_PROGRESS_DISPLACEMENT_M constants are
	// hardcoded parity defaults here (plan §2).
	noProgressWindow := int(math.Round(2.0 / dt))
	noProgressDisp := 0.05
	anchorX, anchorY := prevX, prevY
	anchorStep := 0

	for steps < r.maxSteps {
		nav.Step()
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

		// Terminal surface for Open is the outer wall; any contact ends the run
		// under the strict (pre-2026-08-01) policy the Python simulator uses by
		// default. FootprintCollides checks the full chassis against both walls.
		if trackContact(gw, track, r.cfg) {
			res := r.score(sc, gw, nav, steps, dt, distanceM, maxSpeedMPS, minRangeM, contactCount, targetLaps, true, false)
			return res, nil
		}

		if nav.LapsCompleted() >= targetLaps {
			res := r.score(sc, gw, nav, steps, dt, distanceM, maxSpeedMPS, minRangeM, contactCount, targetLaps, false, false)
			return res, nil
		}

		// No-progress check.
		if math.Hypot(st.X-anchorX, st.Y-anchorY) >= noProgressDisp {
			anchorX, anchorY = st.X, st.Y
			anchorStep = steps
		} else if (steps - anchorStep) >= noProgressWindow {
			res := r.score(sc, gw, nav, steps, dt, distanceM, maxSpeedMPS, minRangeM, contactCount, targetLaps, false, true)
			return res, nil
		}
	}

	// Timed out.
	res := r.score(sc, gw, nav, steps, dt, distanceM, maxSpeedMPS, minRangeM, contactCount, targetLaps, false, false)
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
	collided, stuck bool,
) Result {
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

	return Result{
		TerminalSurface:        terminalSurfaceName(collided),
		Scenario:               sc.ID,
		PassSideViolationSigns: wrongSideSigns,
		CollisionXY:            []float64{cx, cy},
		FinalPose:              []float64{st.X, st.Y, st.Yaw},
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

func terminalSurfaceName(collided bool) string {
	if collided {
		return "outer_wall"
	}
	return "none"
}

// trackContact reports whether the chassis footprint touches a wall (the Open
// Challenge's terminal surface), matching TrackModel.footprint_collides.
func trackContact(gw simGateway, track *collision.TrackModel, cfg harness.Config) bool {
	st := gw.State()
	return track.FootprintCollides(st.X, st.Y, st.Yaw, cfg.ChassisLengthM, cfg.ChassisWidthM)
}

// scenarioStart bundles the parsed spawn pose + travel direction.
type scenarioStart struct {
	X, Y, Yaw float64
	Direction trackmodel.Direction
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

// buildScenario derives the track geometry, spawn pose, and a centerline
// waypoint loop from scenario metadata.
func buildScenario(meta generate.Metadata, cfg harness.Config) (trackmodel.CorridorGeometry, scenarioStart, []trackmodel.Waypoint, error) {
	widthsM := map[trackmodel.Section]float64{}
	for _, sec := range []struct {
		name  string
		which trackmodel.Section
	}{
		{"north", trackmodel.North},
		{"south", trackmodel.South},
		{"east", trackmodel.East},
		{"west", trackmodel.West},
	} {
		wm, ok := meta.CorridorWidths[sec.name]
		if !ok {
			return trackmodel.CorridorGeometry{}, scenarioStart{}, nil,
				fmt.Errorf("metadata missing corridor width for %q", sec.name)
		}
		widthsM[sec.which] = float64(wm.WidthMM) / 1000.0
	}

	geom := trackmodel.CorridorGeometryFromWidths(widthsM, cfg.TrackMaxCoordM)

	dir := trackmodel.Counterclockwise
	if meta.StartingConditions.Direction == "clockwise" {
		dir = trackmodel.Clockwise
	}

	start := scenarioStart{
		X:         meta.StartingConditions.Position.X,
		Y:         meta.StartingConditions.Position.Y,
		Yaw:       meta.StartingConditions.Yaw,
		Direction: dir,
	}

	waypoints := centerlineLoop(geom, cfg.TrackMaxCoordM, dir)
	return geom, start, waypoints, nil
}

// centerlineLoop builds a rectangular waypoint loop offset by half each
// corridor width from the walls — a focused approximation of the Python
// plan_believed_path centerline for the sighted Open Challenge. Edges are
// subdivided so the navigator's waypoint-reached threshold is well-sampled.
func centerlineLoop(geom trackmodel.CorridorGeometry, maxCoord float64, dir trackmodel.Direction) []trackmodel.Waypoint {
	ib := geom.InnerBlock
	// The centerline sits half a corridor width inside each wall:
	//   south corridor -> y = south/2,  north corridor -> y = maxCoord-north/2
	//   west corridor  -> x = west/2,   east corridor  -> x = maxCoord-east/2
	westCL := ib.XMin / 2.0
	eastCL := maxCoord - (maxCoord-ib.XMax)/2.0
	southCL := ib.YMin / 2.0
	northCL := maxCoord - (maxCoord-ib.YMax)/2.0

	// CCW ordering starting along the south edge (west -> east), so a robot
	// spawned on the south centreline travelling CCW proceeds eastward first.
	corners := []trackmodel.Waypoint{
		{X: westCL, Y: southCL},
		{X: eastCL, Y: southCL},
		{X: eastCL, Y: northCL},
		{X: westCL, Y: northCL},
	}
	if dir == trackmodel.Clockwise {
		// Reverse CCW -> CW.
		for i, j := 0, len(corners)-1; i < j; i, j = i+1, j-1 {
			corners[i], corners[j] = corners[j], corners[i]
		}
	}

	const edgeStep = 0.1
	var pts []trackmodel.Waypoint
	for i := 0; i < len(corners); i++ {
		a := corners[i]
		b := corners[(i+1)%len(corners)]
		edgeLen := a.DistanceTo(b)
		n := int(math.Ceil(edgeLen / edgeStep))
		if n < 1 {
			n = 1
		}
		for k := 0; k < n; k++ {
			t := float64(k) / float64(n)
			pts = append(pts, trackmodel.Waypoint{
				X: a.X + (b.X-a.X)*t,
				Y: a.Y + (b.Y-a.Y)*t,
			})
		}
	}
	return pts
}

// defaultLaps returns the Open Challenge default lap count.
func defaultLaps(_ generate.Metadata) int { return navigator.DefaultOpenChallengeLaps }

// defaultKinematicsParams returns the Ackermann integrator parameters for the
// shipped robot. Delegates to kinematics.DefaultParams, the single source of
// truth for these RobotSpecs/RobotDrivetrain constants (plan §2: the profile
// loader is not yet wired, so this is the hardcoded fallback).
func defaultKinematicsParams() kinematics.Params {
	return kinematics.DefaultParams()
}

// compile-time assertion that NativeRunner satisfies Runner.
var _ Runner = (*NativeRunner)(nil)
