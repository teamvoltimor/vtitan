package scenario

import (
	"context"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navigator"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/collision"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/kinematics"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/corpus"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/harness"
)

// NativeRunner implements Runner with a fully Go-native closed-loop
// simulation, replacing SubprocessRunner (the frozen Python oracle). It builds
// the harness + collision.TrackModel from scenario metadata, drives the real
// navigator.Navigator (sighted — direction known, no vision), and scores the
// outcome into a Result.
//
// Scope note: this is the sighted Open-Challenge path. Blind mode (direction
// inference + corridor-width estimation + believed-wall relocalization) and
// the Obstacles pass-side/parking logic are intentionally out of scope here;
// SubprocessRunner remains the parity oracle for those until the native runner
// is extended.
type NativeRunner struct {
	cfg    harness.Config
	seed   uint64
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

// ControlDt returns the simulation timestep (s), matching CONTROL_DT (1/20 Hz).
// TODO: source from the live motion/control.toml once the native runner's
// config is wired to a profile root.
func (c NativeRunnerConfig) ControlDt() float64 { return 1.0 / 20.0 }

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

	track := collision.NewTrackModel(collision.NewTrackModelParams{
		Geometry:           geom,
		MinCoordM:          0.0,
		MaxCoordM:          r.cfg.TrackMaxCoordM,
		Obstacles:          nil,
		LidarSeesObstacles: false,
		CollisionMarginM:   r.cfg.CollisionMarginM,
	})

	kin := kinematics.NewAckermannKinematics(defaultKinematicsParams())

	gw := harness.NewSimHardwareGateway(
		r.cfg, track,
		kinematics.AckermannState{X: startPose.X, Y: startPose.Y, Yaw: startPose.Yaw},
		kin, r.seed,
	)

	targetLaps := defaultLaps(meta)
	navCfg := navigator.ConfigFor(nil, "", nil)
	nav, err := navigator.New(navigator.Params{
		Gateway:           gw,
		Vision:            nil,
		Waypoints:         waypoints,
		Direction:         func() *trackmodel.Direction { d := startPose.Direction; return &d }(),
		NumLaps:           targetLaps,
		Config:            navCfg,
		ControllersConfig: controllers.DefaultConfig(),
		SignRouterConfig:  signrouter.DefaultConfig(),
		SignRouter:        nil,
	})
	if err != nil {
		return Result{}, fmt.Errorf("native runner: building navigator %s: %w", sc.ID, err)
	}

	return r.loop(sc, gw, nav, track, targetLaps, startPose)
}

// loop runs the control loop until terminal (laps / collision / timeout /
// stuck) and scores the Result.
func (r *NativeRunner) loop(
	sc corpus.Scenario,
	gw *harness.SimHardwareGateway,
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
	gw *harness.SimHardwareGateway,
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
	success := !collided && !stuck && !resTimedOut(steps, r.maxSteps, laps, targetLaps)

	return Result{
		TerminalSurface:  terminalSurfaceName(collided),
		Scenario:         sc.ID,
		CollisionXY:      []float64{cx, cy},
		FinalPose:        []float64{st.X, st.Y, st.Yaw},
		SimTimeS:         float64(steps) * dt,
		DistanceM:        distanceM,
		MaxSpeedMPS:      maxSpeedMPS,
		AvgSpeedMPS:      avgSpeed(distanceM, steps, dt),
		MinLidarRangeM:   orZero(minRangeM),
		TargetLaps:       targetLaps,
		LapsCompleted:    laps,
		Steps:            steps,
		ContactCount:     contactCount,
		Collided:         collided,
		TimedOut:         resTimedOut(steps, r.maxSteps, laps, targetLaps),
		Stuck:            stuck,
		Success:          success,
		OverTime:         false,
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
func trackContact(gw *harness.SimHardwareGateway, track *collision.TrackModel, cfg harness.Config) bool {
	st := gw.State()
	return track.FootprintCollides(st.X, st.Y, st.Yaw, cfg.ChassisLengthM, cfg.ChassisWidthM)
}

// scenarioStart bundles the parsed spawn pose + travel direction.
type scenarioStart struct {
	X, Y, Yaw  float64
	Direction  trackmodel.Direction
}

// scenarioMetadata is the subset of the generator's *_metadata.json schema
// the native runner consumes. Mirrors generate.Metadata (that module is a
// separate Go module and not imported here to keep the runner dependency-light).
type scenarioMetadata struct {
	ChallengeType      string                    `json:"challenge_type"`
	CorridorWidths     map[string]widthMeta      `json:"corridor_widths"`
	StartingConditions startingMeta              `json:"starting_conditions"`
	HasParkingLot      bool                      `json:"has_parking_lot"`
}

type widthMeta struct {
	Type    string `json:"type"`
	WidthMM int    `json:"width_mm"`
}

type startingMeta struct {
	Direction string  `json:"direction"`
	Section   string  `json:"section"`
	Position  posMeta `json:"position"`
	Yaw       float64 `json:"yaw"`
}

type posMeta struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

func loadMetadata(path string) (scenarioMetadata, error) {
	raw, err := os.ReadFile(filepath.Clean(path))
	if err != nil {
		return scenarioMetadata{}, err
	}
	var meta scenarioMetadata
	if err := json.Unmarshal(raw, &meta); err != nil {
		return scenarioMetadata{}, err
	}
	return meta, nil
}

// buildScenario derives the track geometry, spawn pose, and a centerline
// waypoint loop from scenario metadata.
func buildScenario(meta scenarioMetadata, cfg harness.Config) (trackmodel.CorridorGeometry, scenarioStart, []trackmodel.Waypoint, error) {
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
		X:        meta.StartingConditions.Position.X,
		Y:        meta.StartingConditions.Position.Y,
		Yaw:      meta.StartingConditions.Yaw,
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
func defaultLaps(_ scenarioMetadata) int { return navigator.DefaultOpenChallengeLaps }

// defaultKinematicsParams returns hardcoded parity defaults for the Ackermann
// integrator. The RobotSpecs/RobotDrivetrain constants these mirror are NOT
// ported to Go (plan §2), so they are supplied here, not read from a profile.
func defaultKinematicsParams() kinematics.Params {
	return kinematics.Params{
		WheelbaseM:          0.20,
		MaxSteerRad:         1.2252,
		MaxSteerRateRadPerS: 3.0,
		MaxAccelMPS2:        0.5,
		MaxSpeedMPS:         1.0,
		RearSteerRatio:      -1.0,
		SpeedTauS:           0.1,
		YawGain:             0.55,
		Substeps:            kinematics.DefaultSubsteps,
	}
}

// compile-time assertion that NativeRunner satisfies Runner.
var _ Runner = (*NativeRunner)(nil)

