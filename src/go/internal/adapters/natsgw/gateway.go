// Package natsgw implements controllers.HardwareGateway over the robot-go
// NATS transport, for bench/dev and single-Pi5 deployments where the nav
// stack runs in one process.
//
// It is the concrete adapter ports.go (controllers.HardwareGateway) said
// would one day exist: it imports the port (never the other way), so the
// dependency direction stays hexagonal. It does NOT add any new NATS subjects
// of its own -- it reuses the topics other nodes already publish:
//
//   - PublishDrive -> vtitan.actuation.v1.ackermann_cmd (the motor node's
//     input, consumed by loop.go's *natsx.Subscriber[*actuationv1.AckermannCmd]).
//   - GetLidarScan -> vtitan.sensor.v1.scan (the lidar node's output).
//   - GetWheelOdometry -> vtitan.actuation.v1.joint_states (the motor
//     node's encoder feedback, internal/node/motor.Feedback), decoded the
//     same way ros2_hardware_gateway.py decodes /joint_states.
//   - GetCurrentPose -> computed IN-PROCESS from the same scan + IMU caches,
//     via localization.LidarLocalizer. There is no POSE NATS subject in this
//     tree (no localizer node -- see localization/doc.go), so pose is
//     estimated here rather than subscribed. Wheel odometry is a measurement
//     and is subscribed; pose is an inference and is computed.
//
// The pull-cache shape (subscribe once, store latest under a sync.RWMutex,
// serve each Get* from the cache) is copied verbatim from
// cmd/telemetry-node/main.go's natsSource, the one production implementation
// of this exact "read live state from a push transport" pattern; the
// diag.Aggregator doc comment predicted this adapter before it was built.
package natsgw

import (
	"context"
	"errors"
	"math"
	"slices"
	"sync"

	"github.com/nats-io/nats.go"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/localization"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
	natsx "github.com/teamvoltimor/vtitan/src/go/internal/transport/nats"
)

// maxSteeringWheelAngleRad is the road-wheel angle at full lock, mapping
// DriveCommand.SteeringNorm ([-1,1], + = left) onto AckermannCmd.steering_angle
// [rad] (positive = left, per the ROS convention the whole stack speaks).
//
// It is the binding steering limit -- ackermann_motor_node clamps the SERVO
// limit, so this decode alone governs how far a full-lock command actually
// turns. The true value is the active servo profile's max_wheel_angle_deg
// (src/config/profiles/<profile>/robot.toml, 55.0 deg for the
// 180deg-injora-14kg servo), loaded from VTITAN_HARDWARE_PROFILE. This module
// has no profile loader, so it hardcodes the 55.0 deg default and names it
// here so it is a one-line swap when a profile is wired in -- matching the
// proto doc's "pull it from the profile, do not invent a bound" instruction
// by centralizing the value rather than scattering a literal.
const maxSteeringWheelAngleRad = 55.0 * math.Pi / 180.0

// nanosecondsPerSecond scales a protobuf Timestamp's nanos field into the
// fractional seconds controllers.WheelOdometry.StampS carries.
const nanosecondsPerSecond = 1e-9

// Gateway is a NATS-backed controllers.HardwareGateway. See the package doc
// for the transport/source decisions.
type Gateway struct {
	conn     *nats.Conn
	drivePub *natsx.Publisher[*actuationv1.AckermannCmd]

	scan *sensorv1.Scan
	imu  *sensorv1.Imu

	// pendingWalls / pendingResetXY are consumed by the scan loop on its next
	// tick, so the localizer rebuild and pose re-seed happen atomically with
	// the cache the navigator reads (no torn state across the mutex). Only
	// touched under g.mu.
	pendingWalls   *trackmodel.TrackWalls
	pendingResetXY *trackmodel.Waypoint

	// walls is the localizer's current seed geometry; swapped under g.mu when
	// pendingWalls is drained.
	walls *trackmodel.TrackWalls

	locCfg localization.Config

	pose trackmodel.Pose

	// wheel is the latest odometry decoded from joint_states; haveWheel
	// stays false until the first message with a drive joint arrives.
	wheel controllers.WheelOdometry
	// headingOffsetRad is added to the IMU-derived yaw before it is returned,
	// so ResetHeadingReference/CorrectHeadingForDirectionChange can shift the
	// estimator's heading without a round-trip to a (nonexistent) localizer
	// node. zero means "trust the IMU as-is".
	headingOffsetRad float64

	// wheelRadiusM scales the drive joint's ANGLE into linear travel. It is
	// a construction-time fact rather than something read per message: the
	// publisher sends SI radians precisely so the consumer applies the
	// radius it believes in, matching ros2_hardware_gateway.py's use of
	// RobotSpecs.WHEEL_RADIUS.
	wheelRadiusM float64

	mu                sync.RWMutex
	havePose          bool
	captureHeadingRef bool

	haveWheel bool
}

// New builds a Gateway over an already-connected conn and the track walls the
// localizer seeds from. locCfg tunes the in-process localizer; the zero value
// is NOT usable -- pass localization.DefaultConfig(). wheelRadiusM is
// robot.toml's [wheel] radius, used to turn the drive joint's angle into
// linear travel; it must be positive.
func New(
	conn *nats.Conn,
	initialWalls *trackmodel.TrackWalls,
	locCfg localization.Config,
	wheelRadiusM float64,
) (*Gateway, error) {
	if conn == nil {
		return nil, errors.New("natsgw: nil NATS connection")
	}
	if initialWalls == nil {
		return nil, errors.New("natsgw: initial walls are required to seed the localizer")
	}
	if wheelRadiusM <= 0 {
		return nil, errors.New("natsgw: wheel radius must be positive")
	}
	return &Gateway{
		conn: conn,
		drivePub: natsx.NewPublisher[*actuationv1.AckermannCmd](
			conn,
			actuationv1.AckermannCmdSubject,
		),
		locCfg:       locCfg,
		walls:        initialWalls,
		wheelRadiusM: wheelRadiusM,
	}, nil
}

// localizer builds a fresh LidarLocalizer over walls. Rebuilt (not mutated) on
// SetBelievedWalls so the gateway owns the localizer lifecycle cleanly.
func (g *Gateway) localizer(walls *trackmodel.TrackWalls) *localization.LidarLocalizer {
	return localization.New(walls, g.locCfg)
}

var _ controllers.HardwareGateway = (*Gateway)(nil)

// PublishDrive encodes command as an AckermannCmd and publishes it on
// vtitan.actuation.v1.ackermann_cmd, the motor node's subscriber subject
// (loop.go:118). SteeringNorm (+ = left) maps to a positive (left) wheel
// angle; the magnitude scales against maxSteeringWheelAngleRad.
func (g *Gateway) PublishDrive(command controllers.DriveCommand) {
	if err := g.drivePub.Publish(g.driveCommand(command)); err != nil {
		// Best-effort: a publish failure here is a transport dropout, not a
		// domain error the navigator can act on. The motor node's deadline
		// watchdog will safety-stop on the missing command stream.
		_ = err
	}
}

// driveCommand encodes a DriveCommand as an AckermannCmd without publishing.
// SteeringNorm (+ = left) maps to a positive (left) wheel angle; the magnitude
// scales against maxSteeringWheelAngleRad and is clamped to [-1, 1].
func (g *Gateway) driveCommand(command controllers.DriveCommand) *actuationv1.AckermannCmd {
	steering := math.Max(-1.0, math.Min(1.0, command.SteeringNorm)) * maxSteeringWheelAngleRad
	return &actuationv1.AckermannCmd{
		Speed:         float32(command.SpeedMPS),
		SteeringAngle: float32(steering),
	}
}

// GetCurrentPose returns the latest in-process localizer estimate, ok=false
// until the first scan has been scored. The localizer runs inside the scan
// watch loop (see Run), so this is a cache read.
func (g *Gateway) GetCurrentPose() (trackmodel.Pose, bool) {
	g.mu.RLock()
	defer g.mu.RUnlock()
	return g.pose, g.havePose
}

// GetLidarScan returns the latest Scan, reconstituted into the controller's
// LidarScan shape (ranges + per-ray angles derived from the proto's min/increment
// fields). ok=false until the first scan arrives.
func (g *Gateway) GetLidarScan() (controllers.LidarScan, bool) {
	g.mu.RLock()
	scan := g.scan
	g.mu.RUnlock()
	if scan == nil {
		return controllers.LidarScan{}, false
	}
	return scanToLidarScan(scan), true
}

// LatestScan returns the most recent raw sensorv1.Scan the gateway has cached,
// or nil if none has arrived yet. Used by the track-navigator's --record mode to
// write the /scan topic into the run's MCAP bag alongside /nav_debug.
func (g *Gateway) LatestScan() *sensorv1.Scan {
	g.mu.RLock()
	defer g.mu.RUnlock()
	return g.scan
}

// GetWheelOdometry returns the latest wheel travel decoded from
// vtitan.actuation.v1.joint_states, ok=false until the first message
// arrives (or forever, if Run was given no joint-states subscriber -- a
// deployment with no encoder wired). This is the direct analogue of
// ros2_hardware_gateway.py's /joint_states subscription, and its absence is
// what kept internal/nav/bayexit sim-only.
//
// ok=false is a normal state, not an error: the navigator holds (commands
// zero drive) rather than guessing at travel it cannot measure.
func (g *Gateway) GetWheelOdometry() (controllers.WheelOdometry, bool) {
	g.mu.RLock()
	defer g.mu.RUnlock()
	return g.wheel, g.haveWheel
}

// SetBelievedWalls re-points the in-process localizer at the layout the robot
// now believes in. Mid-round blind wall estimation updates this; the gateway
// rebuilds its localizer rather than mutating domain state across a boundary.
func (g *Gateway) SetBelievedWalls(walls *trackmodel.TrackWalls) {
	if walls == nil {
		return
	}
	// Rebuild under the read lock so a concurrent GetCurrentPose can't read a
	// nil localizer. The localizer is rebuilt lazily inside the scan loop via
	// a held pointer, guarded here.
	g.mu.Lock()
	g.pendingWalls = walls
	g.mu.Unlock()
}

// ResetPosition re-seeds the localizer's position and clears its between-tick
// tracking state (jump confirmation, last-estimate timestamp).
func (g *Gateway) ResetPosition(x, y float64) {
	g.mu.Lock()
	g.pose = trackmodel.Pose{X: x, Y: y}
	g.havePose = true
	g.pendingResetXY = &trackmodel.Waypoint{X: x, Y: y}
	g.mu.Unlock()
}

// ResetHeadingReference re-zeros the estimator's heading against the NEXT IMU
// reading: the next scan-scored yaw absorbs an offset that makes it match the
// IMU exactly, discarding any drift accumulated since construction.
func (g *Gateway) ResetHeadingReference() {
	g.mu.Lock()
	g.captureHeadingRef = true
	g.mu.Unlock()
}

// CorrectHeadingForDirectionChange shifts the estimator's heading by deltaRad
// in full -- used when blind direction inference overturns the direction
// assumed at construction.
func (g *Gateway) CorrectHeadingForDirectionChange(deltaRad float64) {
	g.mu.Lock()
	g.headingOffsetRad += deltaRad
	g.mu.Unlock()
}

// scanToLidarScan converts a sensorv1.Scan into the controller's LidarScan,
// expanding the proto's angle_min/angle_increment into a per-ray angle slice
// (0 rad = forward, +pi/2 = left) matching controllers.LidarScan.
func scanToLidarScan(scan *sensorv1.Scan) controllers.LidarScan {
	n := len(scan.GetRanges())
	angles := make([]float64, n)
	angleMin := float64(scan.GetAngleMin())
	inc := float64(scan.GetAngleIncrement())
	for i := range n {
		angles[i] = angleMin + float64(i)*inc
	}
	ranges := make([]float64, n)
	for i, r := range scan.GetRanges() {
		ranges[i] = float64(r)
	}
	return controllers.LidarScan{RangesM: ranges, AnglesRad: angles}
}

// imuYawRad extracts yaw (rotation about Z) from an IMU orientation quaternion,
// in radians, 0 = forward/+pi/2 = left (matching trackmodel.Pose.Yaw and the
// LidarScan convention).
func imuYawRad(imu *sensorv1.Imu) (float64, bool) {
	q := imu.GetOrientation()
	if q == nil {
		return 0, false
	}
	// Standard yaw from a unit quaternion (ENU, Z-up):
	//   yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
	yaw := math.Atan2(2*(q.GetW()*q.GetZ()+q.GetX()*q.GetY()),
		1-2*(q.GetY()*q.GetY()+q.GetZ()*q.GetZ()))
	return yaw, true
}

// Run starts the background subscription loops and blocks until ctx is done
// or a subscription hits a non-cancellation error. The loops feed the latest
// Scan/Imu into the mutex-guarded cache (the natsSource pattern), and the
// scan loop additionally scores the in-process localizer every tick so
// GetCurrentPose serves a fresh estimate without its own NATS round-trip.
//
// Call Run (typically in its own goroutine) before the navigator starts
// calling Get*. It returns nil on ctx cancellation.
//
// jointSub may be nil, for a deployment with no encoder publishing
// joint_states: GetWheelOdometry then keeps reporting ok=false, which the
// navigator already treats as a normal state.
func (g *Gateway) Run(
	ctx context.Context,
	scanSub *natsx.Subscriber[*sensorv1.Scan],
	imuSub *natsx.Subscriber[*sensorv1.Imu],
	jointSub *natsx.Subscriber[*actuationv1.JointStates],
) error {
	// Buffered so a loop that returns after another has already won the
	// select below doesn't block forever on an unread send.
	loopErr := make(chan error, 3)
	go func() { loopErr <- g.scanLoop(ctx, scanSub) }()
	go func() { loopErr <- g.imuLoop(ctx, imuSub) }()
	if jointSub != nil {
		go func() { loopErr <- g.jointLoop(ctx, jointSub) }()
	}

	err := <-loopErr
	if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
		return nil
	}
	return err
}

// jointLoop stores the wheel odometry decoded from each JointStates.
func (g *Gateway) jointLoop(
	ctx context.Context,
	sub *natsx.Subscriber[*actuationv1.JointStates],
) error {
	for {
		joints, err := sub.Read(ctx)
		if err != nil {
			if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
				return nil
			}
			return err //nolint:wrapcheck // Read already wraps with "nats: ..." context
		}
		odometry, ok := wheelOdometryFrom(joints, g.wheelRadiusM)
		if !ok {
			continue
		}
		g.mu.Lock()
		g.wheel = odometry
		g.haveWheel = true
		g.mu.Unlock()
	}
}

// wheelOdometryFrom converts the drive joint's accumulating angle and rate
// into linear travel and speed, ok=false when the message carries no drive
// joint or no position for it.
//
// Indexed by joint NAME rather than array position, matching
// ros2_hardware_gateway.py's _joint_state_callback: JointStates carries an
// arbitrary set of joints in an arbitrary order, and assuming index 0 is the
// drive wheel would break silently the moment another joint is added.
//
// A missing velocity entry yields a zero speed rather than dropping the
// sample: position is what bayexit and the parking clamp actually
// difference, and refusing the whole message over an absent rate would
// withhold the travel they need.
func wheelOdometryFrom(
	joints *actuationv1.JointStates,
	wheelRadiusM float64,
) (controllers.WheelOdometry, bool) {
	index := slices.Index(joints.GetName(), actuationv1.DriveJoint)
	if index < 0 || index >= len(joints.GetPosition()) {
		return controllers.WheelOdometry{}, false
	}

	speedMPS := 0.0
	if index < len(joints.GetVelocity()) {
		speedMPS = joints.GetVelocity()[index] * wheelRadiusM
	}
	stamp := joints.GetStamp()
	return controllers.WheelOdometry{
		DistanceM: joints.GetPosition()[index] * wheelRadiusM,
		SpeedMPS:  speedMPS,
		// Seconds since the epoch, the same sec + nanosec*1e-9 the Python
		// callback assembles. Only differences between successive stamps are
		// meaningful to the consumer, so the epoch itself does not matter --
		// only that every sample shares one.
		StampS: float64(stamp.GetSeconds()) + float64(stamp.GetNanos())*nanosecondsPerSecond,
	}, true
}

// scanLoop stores each Scan and, when a yaw is available, scores the localizer
// against it, caching the resulting pose. It also drains any pending wall
// swap or position re-seed under the same lock so the navigator never reads a
// half-updated cache.
func (g *Gateway) scanLoop(ctx context.Context, sub *natsx.Subscriber[*sensorv1.Scan]) error {
	for {
		scan, err := sub.Read(ctx)
		if err != nil {
			if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
				return nil
			}
			return err //nolint:wrapcheck // Read already wraps with "nats: ..." context
		}

		g.mu.Lock()
		g.scan = scan

		// Drain a pending wall swap / position re-seed before scoring.
		if g.pendingWalls != nil {
			g.walls = g.pendingWalls
			g.pendingWalls = nil
		}
		if g.pendingResetXY != nil {
			g.pose = trackmodel.Pose{X: g.pendingResetXY.X, Y: g.pendingResetXY.Y, Yaw: g.pose.Yaw}
			g.havePose = true
			g.pendingResetXY = nil
		}

		yaw, haveYaw := g.currentYawLocked()
		if haveYaw {
			g.scorePoseLocked(scan, yaw)
		}
		g.mu.Unlock()
	}
}

// imuLoop stores each Imu. The scan loop does the actual localizer scoring
// (it needs both scan and yaw together); this just keeps the latest IMU for
// yaw. If a scan arrives before any IMU, the next scan will still score once
// yaw is present.
func (g *Gateway) imuLoop(ctx context.Context, sub *natsx.Subscriber[*sensorv1.Imu]) error {
	for {
		imu, err := sub.Read(ctx)
		if err != nil {
			if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
				return nil
			}
			return err //nolint:wrapcheck // Read already wraps with "nats: ..." context
		}
		g.mu.Lock()
		g.imu = imu
		if g.captureHeadingRef {
			// Zero the offset against the next IMU: the next scored yaw becomes
			// the reference and subsequent IMU drifts are tracked from it.
			if yaw, ok := imuYawRad(imu); ok {
				g.headingOffsetRad = -yaw
			}
			g.captureHeadingRef = false
		}
		g.mu.Unlock()
	}
}

// currentYawLocked returns the IMU-derived yaw plus the gateway's heading
// offset. Caller must hold g.mu.
func (g *Gateway) currentYawLocked() (float64, bool) {
	if g.imu == nil {
		return 0, false
	}
	yaw, ok := imuYawRad(g.imu)
	if !ok {
		return 0, false
	}
	return yaw + g.headingOffsetRad, true
}

// scorePoseLocked estimates (x, y) from the cached prior pose and the given
// scan, caches the result. Caller must hold g.mu.
func (g *Gateway) scorePoseLocked(scan *sensorv1.Scan, yaw float64) {
	lidar := scanToLidarScan(scan)
	if len(lidar.RangesM) == 0 || len(lidar.RangesM) != len(lidar.AnglesRad) {
		return
	}

	prior := trackmodel.Waypoint{X: g.pose.X, Y: g.pose.Y}
	if !g.havePose {
		// No prior yet: seed at the origin so the first estimate is a pure
		// scan match rather than a no-op (EstimatePosition returns priorXY
		// unchanged when prior is meaningless).
		prior = trackmodel.Waypoint{}
	}

	est := g.localizer(g.walls).EstimatePosition(prior, yaw, lidar.RangesM, lidar.AnglesRad, nil)
	g.pose = trackmodel.Pose{X: est.X, Y: est.Y, Yaw: yaw}
	g.havePose = true
}

// Close releases the NATS connection the gateway was built over.
func (g *Gateway) Close() error {
	if g.conn == nil {
		return nil
	}
	g.conn.Close()
	return nil
}
