package harness

import (
	"math"
	"math/rand/v2"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/collision"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/kinematics"
)

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

	state     kinematics.AckermannState
	command   controllers.DriveCommand
	scan      controllers.LidarScan
	elapsedS  float64
	lastScanS float64

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
		cfg:   cfg,
		track: track,
		kin:   kin,
		rng:   rand.New(rand.NewPCG(seed, seed^0x9e3779b97f4a7c15)),
		state: initial,
	}
	g.buildAngles()
	g.refreshSensors()
	return g
}

// buildAngles fills the full 360 sweep (robot frame, 0 = forward, +pi/2 =
// left), matching the Python gateway's np.linspace(-pi, pi, lidar_rays).
func (g *SimHardwareGateway) buildAngles() {
	n := g.cfg.LidarSamples
	if n <= 0 {
		n = 360
	}
	angles := make([]float64, n)
	for i := 0; i < n; i++ {
		// linspace(-pi, pi, n): endpoints inclusive, matching numpy.
		angles[i] = -math.Pi + (2*math.Pi)*float64(i)/float64(n-1)
	}
	g.angles = angles
}

// State returns the current kinematic state (the simulated body pose).
func (g *SimHardwareGateway) State() kinematics.AckermannState { return g.state }

// PublishDrive stores the latest command; applied on the next Advance.
func (g *SimHardwareGateway) PublishDrive(command controllers.DriveCommand) {
	g.command = command
}

// GetCurrentPose returns the ground-truth kinematic pose. localize is not
// implemented (TODO below), so this is always perfect odometry.
func (g *SimHardwareGateway) GetCurrentPose() (trackmodel.Pose, bool) {
	return trackmodel.Pose{X: g.state.X, Y: g.state.Y, Yaw: g.state.Yaw}, true
}

// GetLidarScan returns the most recent simulated sweep (ranges + angles).
func (g *SimHardwareGateway) GetLidarScan() (controllers.LidarScan, bool) {
	if len(g.scan.RangesM) == 0 {
		return controllers.LidarScan{}, false
	}
	return g.scan, true
}

// GetWheelOdometry returns zero travel; the navigator's control loop does not
// consume odometry, so a no-op estimate is sufficient.
func (g *SimHardwareGateway) GetWheelOdometry() (controllers.WheelOdometry, bool) {
	return controllers.WheelOdometry{StampS: g.elapsedS, SpeedMPS: g.state.V}, true
}

// SetBelievedWalls is accepted but a no-op: blind mode (believed-wall
// relocalization) is out of scope for the sighted native runner.
func (g *SimHardwareGateway) SetBelievedWalls(_ *trackmodel.TrackWalls) {}

// ResetPosition re-seeds the kinematic state's XY, matching the protocol.
func (g *SimHardwareGateway) ResetPosition(x, y float64) {
	g.state.X, g.state.Y = x, y
	g.refreshSensors()
}

// ResetHeadingReference is a no-op (no IMU drift model in the native runner).
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

	candidate := g.kin.Step(g.state, g.command.SpeedMPS, g.command.SteeringNorm, dt)
	g.state = candidate

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

// refreshSensors casts rays from the sensor origin (chassis centre, matching
// the simplified Go model — the LIDAR mount offset is NOT yet applied here;
// TODO: offset the cast by RobotSpecs.LIDAR_MOUNT_X_OFFSET) and applies the
// Gaussian noise + invalid-ray dropout the Python oracle models.
func (g *SimHardwareGateway) refreshSensors() {
	sensorX, sensorY := g.state.X, g.state.Y
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
}

// Collided reports whether the chassis is currently touching a terminal
// surface (re-evaluated every Advance).
func (g *SimHardwareGateway) Collided() bool { return g.collided }

// CollisionXY returns the last collision point, or (0,0) if never collided.
func (g *SimHardwareGateway) CollisionXY() (float64, float64) {
	return g.collisionX, g.collisionY
}
