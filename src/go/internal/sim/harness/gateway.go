package harness

import (
	"math"
	"math/rand/v2"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/localization"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/collision"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/kinematics"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/sensorerrors"
)

// sensorErrorStreamSalt separates the sensor-error RNG stream from the LIDAR
// sampler's. Any fixed non-zero constant works; what matters is that the two
// streams never coincide, so enabling sensor errors cannot perturb the LIDAR
// noise sequence an unperturbed control run was measured on.
const sensorErrorStreamSalt = 0xa076_1d64_78bd_642f

// SimHardwareGateway is the Go-native HardwareGateway backing a scenario run
// with a kinematic body and a raycast LIDAR, replacing the Python
// SimulatedHardwareGateway (the subprocess wrapper's oracle) end-to-end.
//
// It mirrors the Python oracle's behaviour:
//   - GetCurrentPose returns the kinematic pose as ground truth (perfect
//     odometry) — localize=false in the Python gateway. wall_heading
//     localization is NOT implemented yet (see TODO in Advance), so Localize
//     is currently ignored and ground truth is always returned.
//   - GetLidarScan raycasts TrackModel.RaycastScan with Gaussian range noise
//     (LidarNoiseStd) and invalid-ray dropout at InvalidRayRate.
//   - PublishDrive stores the latest command; Advance integrates it over dt
//     and regenerates the sensors, matching SimulatedHardwareGateway.advance.
type SimHardwareGateway struct {
	cfg    Config
	track  *collision.TrackModel
	kin    *kinematics.AckermannKinematics
	rng    *rand.Rand
	angles []float64

	// localizer is nil unless cfg.Localize; when set, believed carries the
	// scan-matched estimate the navigator is given in place of the true pose.
	localizer    *localization.LidarLocalizer
	believed     trackmodel.Waypoint
	haveBelieved bool

	state     kinematics.AckermannState
	command   controllers.DriveCommand
	scan      controllers.LidarScan
	elapsedS  float64
	lastScanS float64
	distanceM float64

	// imu is nil when the run configures no sensor errors, which is the
	// default and every existing corpus number's condition. A nil model
	// means GetCurrentPose returns ground-truth yaw with no arithmetic at
	// all, rather than truth-plus-zero.
	imu *sensorerrors.IMUModel
	// startPosErrorX/Y is the fixed displacement between where the body is
	// and where the pose is reported to be. Drawn once at a random bearing,
	// then held: it models a robot placed somewhere other than where it
	// believes, which does not converge out on its own.
	startPosErrorX float64
	startPosErrorY float64
	// rotationRad is the signed, UNWRAPPED rotation the body has turned
	// through, so three laps of one-way cornering accumulate rather than
	// cancel. Only the gyro scale error reads it.
	rotationRad float64
	prevTrueYaw float64

	collided   bool
	collisionX float64
	collisionY float64
}

// NewSimHardwareGateway builds a gateway from a kinematic state, the track
// model, and the harness config.
func NewSimHardwareGateway(
	cfg Config,
	track *collision.TrackModel,
	initial kinematics.AckermannState,
	kin *kinematics.AckermannKinematics,
	seed uint64,
) *SimHardwareGateway {
	g := &SimHardwareGateway{
		cfg:         cfg,
		track:       track,
		kin:         kin,
		rng:         rand.New(rand.NewPCG(seed, seed^0x9e3779b97f4a7c15)),
		state:       initial,
		prevTrueYaw: initial.Yaw,
	}
	g.initSensorErrors(seed)
	g.buildAngles()
	if cfg.Localize {
		locCfg := localization.DefaultConfig()
		if cfg.LocalizationConfig != nil {
			locCfg = *cfg.LocalizationConfig
		}
		g.localizer = localization.New(track.Walls(), locCfg)
		// Seed the prior at the true start: a race begins with the robot
		// placed where the crew believes it is, and Python's gateway
		// likewise starts from the scenario pose rather than the origin.
		g.believed = trackmodel.Waypoint{X: initial.X, Y: initial.Y}
		g.haveBelieved = true
	}
	g.refreshSensors()
	return g
}

// initSensorErrors builds the IMU model and draws the start-pose offset,
// when cfg.SensorErrors asks for any perturbation at all.
//
// The error stream is spawned from the run seed but kept SEPARATE from the
// LIDAR sampler above. Drawing these from the sensor stream would shift the
// LIDAR noise sequence, silently changing every existing result -- including
// the unperturbed control runs a perturbed arm is supposed to be compared
// against. Mirrors the Python gateway's own seed_seq.spawn(1).
func (g *SimHardwareGateway) initSensorErrors(seed uint64) {
	errors := g.cfg.SensorErrors
	if !errors.Any() {
		return
	}

	errRNG := rand.New(rand.NewPCG(seed^sensorErrorStreamSalt, seed))
	g.imu = sensorerrors.NewIMUModel(errors, errRNG)

	// A random bearing, so the error is not systematically along-track --
	// which a localizer finds far easier to correct than a lateral one.
	bearing := errRNG.Float64()*2*math.Pi - math.Pi
	g.startPosErrorX = errors.StartPosErrorM * math.Cos(bearing)
	g.startPosErrorY = errors.StartPosErrorM * math.Sin(bearing)
}

// buildAngles fills the full 360 sweep (robot frame, 0 = forward, +pi/2 =
// left), matching the Python gateway's np.linspace(-pi, pi, lidar_rays).
func (g *SimHardwareGateway) buildAngles() {
	n := g.cfg.LidarSamples
	if n <= 0 {
		n = 360
	}
	angles := navutil.AngleFanClosed(n)
	g.angles = angles
}

// State returns the current kinematic state (the simulated body pose).
func (g *SimHardwareGateway) State() kinematics.AckermannState { return g.state }

// PublishDrive stores the latest command; applied on the next Advance.
func (g *SimHardwareGateway) PublishDrive(command controllers.DriveCommand) {
	g.command = command
}

// reportedYaw is the heading the robot BELIEVES it has: ground truth with no
// arithmetic when no IMU error model is configured, otherwise the model's
// drifted answer. The localizer takes yaw as accurate and does not search
// over it, so it must be given the same value the navigator steers on.
func (g *SimHardwareGateway) reportedYaw() float64 {
	if g.imu == nil {
		return g.state.Yaw
	}
	return g.imu.Yaw(g.state.Yaw, g.elapsedS, g.rotationRad)
}

// GetCurrentPose returns the pose the robot believes it has: the LIDAR
// scan-matcher's estimate when cfg.Localize is set, otherwise the kinematic
// pose (perfect odometry, offset by any configured start-placement error).
func (g *SimHardwareGateway) GetCurrentPose() (trackmodel.Pose, bool) {
	yaw := g.reportedYaw()
	if g.localizer != nil && g.haveBelieved {
		return trackmodel.Pose{X: g.believed.X, Y: g.believed.Y, Yaw: yaw}, true
	}
	if g.imu == nil {
		return trackmodel.Pose{X: g.state.X, Y: g.state.Y, Yaw: yaw}, true
	}
	return trackmodel.Pose{
		X:   g.state.X + g.startPosErrorX,
		Y:   g.state.Y + g.startPosErrorY,
		Yaw: yaw,
	}, true
}

// updateBelievedPose scan-matches the freshest sweep against the track walls
// and carries the result forward as the robot's own position estimate. The
// prior is the PREVIOUS estimate, never ground truth: feeding truth back in
// would silently re-anchor the robot every tick and hide exactly the drift
// this models.
func (g *SimHardwareGateway) updateBelievedPose() {
	if g.localizer == nil || len(g.scan.RangesM) == 0 {
		return
	}
	nowS := g.elapsedS
	g.believed = g.localizer.EstimatePosition(
		g.believed, g.reportedYaw(), g.scan.RangesM, g.scan.AnglesRad, &nowS,
	)
	g.haveBelieved = true
}

// GetLidarScan returns the most recent simulated sweep (ranges + angles).
func (g *SimHardwareGateway) GetLidarScan() (controllers.LidarScan, bool) {
	if len(g.scan.RangesM) == 0 {
		return controllers.LidarScan{}, false
	}
	return g.scan, true
}

// GetWheelOdometry returns the accumulated signed wheel travel and current
// speed, matching the Python SimulatedHardwareGateway.
func (g *SimHardwareGateway) GetWheelOdometry() (controllers.WheelOdometry, bool) {
	return controllers.WheelOdometry{DistanceM: g.distanceM, StampS: g.elapsedS, SpeedMPS: g.state.V}, true
}

// SetBelievedWalls is accepted but a no-op: blind mode (believed-wall
// relocalization) is out of scope for the sighted native runner.
func (g *SimHardwareGateway) SetBelievedWalls(_ *trackmodel.TrackWalls) {}

// ResetPosition re-seeds the kinematic state's XY, matching the protocol.
func (g *SimHardwareGateway) ResetPosition(x, y float64) {
	g.state.X, g.state.Y = x, y
	g.refreshSensors()
}

// ResetHeadingReference is a no-op. The IMU error model deliberately does
// NOT reset here: a BNO085's yaw zero is fixed at boot, so the bias a
// chassis set down askew carries is not something the navigator can re-zero
// away mid-round.
func (g *SimHardwareGateway) ResetHeadingReference() {}

// CorrectHeadingForDirectionChange shifts the kinematic yaw by deltaRad.
func (g *SimHardwareGateway) CorrectHeadingForDirectionChange(deltaRad float64) {
	g.state.Yaw = navutil.WrapAngle(g.state.Yaw + deltaRad)
}

// Advance integrates the last command over dt (defaulting to the control
// interval) and regenerates the sensors, matching the Python advance().
func (g *SimHardwareGateway) Advance(dt float64) {
	if dt <= 0 {
		dt = g.cfg.dt()
	}
	g.elapsedS += dt

	prevX, prevY, prevYaw := g.state.X, g.state.Y, g.state.Yaw
	candidate := g.kin.Step(g.state, g.command.SpeedMPS, g.command.SteeringNorm, dt)
	g.state = candidate

	// Signed along the heading, not unsigned path length: a quadrature
	// encoder counts down in reverse, so the real distance_m is signed.
	// Accumulating hypot() here would make a reversing robot report travel
	// forwards, matching the Python oracle's _wheel_distance_m update.
	g.distanceM += (g.state.X-prevX)*math.Cos(prevYaw) + (g.state.Y-prevY)*math.Sin(prevYaw)

	// Unwrapped so a one-way round accumulates: the gyro scale error scales
	// with the course turned rather than the clock, and wrapping here would
	// cancel twelve corners back to nearly nothing.
	g.rotationRad += navutil.WrapAngle(g.state.Yaw - g.prevTrueYaw)
	g.prevTrueYaw = g.state.Yaw

	// The chassis is allowed to graze a wall; the integrated pose is kept
	// (the Python allowed_step logic is ported separately and applied by the
	// runner via FootprintCollides when scoring). Here the body always moves
	// to the candidate pose, matching the no-solid-walls default.

	surface := g.track.ContactSurfaceAt(
		g.state.X, g.state.Y, g.state.Yaw,
		g.cfg.ChassisLengthM, g.cfg.ChassisWidthM,
	)
	g.collided = surface != collision.SurfaceNone
	if g.collided {
		g.collisionX, g.collisionY = g.state.X, g.state.Y
	}

	// Regenerate the scan only when a sweep period has elapsed (or every tick
	// when LidarHz <= 0, the always-fresh default).
	if g.elapsedS-g.lastScanS >= g.cfg.lidarPeriodS() || g.lenScan() == 0 {
		g.lastScanS = g.elapsedS
		g.refreshSensors()
	}
}

func (g *SimHardwareGateway) lenScan() int { return len(g.scan.RangesM) }

// refreshSensors casts rays from the SENSOR origin -- LidarMountXOffsetM
// forward of the chassis centre, matching SimulatedHardwareGateway's own
// Pose.sensor_origin(LIDAR_MOUNT_X_OFFSET) -- and applies the Gaussian noise
// + invalid-ray dropout the Python oracle models.
//
// Casting from the body centre instead (which this did until 2026-09-06) puts
// every return 12.2 cm further away than the real sensor would see it, and
// makes the LIDAR scan-matcher, which predicts ranges FROM the mount offset,
// match against a sensor that does not exist.
func (g *SimHardwareGateway) refreshSensors() {
	sensorX := g.state.X + g.cfg.LidarMountXOffsetM*math.Cos(g.state.Yaw)
	sensorY := g.state.Y + g.cfg.LidarMountXOffsetM*math.Sin(g.state.Yaw)
	ranges := g.track.RaycastScan(
		sensorX, sensorY, g.state.Yaw,
		g.angles, g.cfg.LidarMinRangeM, g.cfg.LidarMaxRangeM,
	)

	if g.cfg.LidarNoiseStd > 0 {
		for i, r := range ranges {
			ranges[i] = r + g.rng.NormFloat64()*g.cfg.LidarNoiseStd
			if ranges[i] < g.cfg.LidarMinRangeM {
				ranges[i] = g.cfg.LidarMinRangeM
			}
			if ranges[i] > g.cfg.LidarMaxRangeM {
				ranges[i] = g.cfg.LidarMaxRangeM
			}
		}
	}

	if g.cfg.InvalidRayRate > 0 {
		for i := range ranges {
			if g.rng.Float64() < g.cfg.InvalidRayRate {
				ranges[i] = math.Inf(1)
			}
		}
	}

	ranges = controllers.SanitizeLidarRanges(ranges, g.cfg.LidarMaxRangeM)

	g.scan = controllers.LidarScan{RangesM: ranges, AnglesRad: g.angles}

	g.updateBelievedPose()
}

// Collided reports whether the chassis is currently touching a terminal
// surface (re-evaluated every Advance).
func (g *SimHardwareGateway) Collided() bool { return g.collided }

// CollisionXY returns the last collision point, or (0,0) if never collided.
func (g *SimHardwareGateway) CollisionXY() (float64, float64) {
	return g.collisionX, g.collisionY
}
