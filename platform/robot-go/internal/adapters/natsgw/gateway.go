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
//   - GetCurrentPose -> computed IN-PROCESS from the same scan + IMU caches,
//     via localization.LidarLocalizer. There is no pose/odometry NATS subject
//     in this tree (no localizer node, no wheel odometry -- see
//     localization/doc.go), so pose is estimated here rather than subscribed.
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
	"sync"

	"github.com/nats-io/nats.go"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/localization"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
	actuationv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/actuation/v1"
	sensorv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/sensor/v1"
	natsx "github.com/teamvoltimor/vtitan/platform/robot-go/internal/transport/nats"
)

// maxSteeringWheelAngleRad is the road-wheel angle at full lock, mapping
// DriveCommand.SteeringNorm ([-1,1], + = left) onto AckermannCmd.steering_angle
// [rad] (positive = left, per the ROS convention the whole stack speaks).
//
// It is the binding steering limit -- ackermann_motor_node clamps the SERVO
// limit, so this decode alone governs how far a full-lock command actually
// turns. The true value is the active servo profile's max_wheel_angle_deg
// (platform/shared/config/profiles/<profile>/robot.toml, 55.0 deg for the
// 180deg-injora-14kg servo), loaded from VTITAN_HARDWARE_PROFILE. This module
// has no profile loader, so it hardcodes the 55.0 deg default and names it
// here so it is a one-line swap when a profile is wired in -- matching the
// proto doc's "pull it from the profile, do not invent a bound" instruction
// by centralizing the value rather than scattering a literal.
const maxSteeringWheelAngleRad = 55.0 * math.Pi / 180.0

// Gateway is a NATS-backed controllers.HardwareGateway. See the package doc
// for the transport/source decisions.
type Gateway struct {
	conn     *nats.Conn
	drivePub *natsx.Publisher[*actuationv1.AckermannCmd]

	locCfg localization.Config

	mu       sync.RWMutex
	scan     *sensorv1.Scan
	imu      *sensorv1.Imu
	pose     trackmodel.Pose
	havePose bool
	// headingOffsetRad is added to the IMU-derived yaw before it is returned,
	// so ResetHeadingReference/CorrectHeadingForDirectionChange can shift the
	// estimator's heading without a round-trip to a (nonexistent) localizer
	// node. zero means "trust the IMU as-is".
	headingOffsetRad  float64
	captureHeadingRef bool

	// pendingWalls / pendingResetXY are consumed by the scan loop on its next
	// tick, so the localizer rebuild and pose re-seed happen atomically with
	// the cache the navigator reads (no torn state across the mutex). Only
	// touched under g.mu.
	pendingWalls   *trackmodel.TrackWalls
	pendingResetXY *trackmodel.Waypoint

	// walls is the localizer's current seed geometry; swapped under g.mu when
	// pendingWalls is drained.
	walls *trackmodel.TrackWalls
}

// New builds a Gateway over an already-connected conn and the track walls the
// localizer seeds from. locCfg tunes the in-process localizer; the zero value
// is NOT usable -- pass localization.DefaultConfig().
func New(
	conn *nats.Conn,
	initialWalls *trackmodel.TrackWalls,
	locCfg localization.Config,
) (*Gateway, error) {
	if conn == nil {
		return nil, errors.New("natsgw: nil NATS connection")
	}
	if initialWalls == nil {
		return nil, errors.New("natsgw: initial walls are required to seed the localizer")
	}
	return &Gateway{
		conn: conn,
		drivePub: natsx.NewPublisher[*actuationv1.AckermannCmd](
			conn,
			actuationv1.AckermannCmdSubject,
		),
		locCfg: locCfg,
		walls:  initialWalls,
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

// GetWheelOdometry always reports ok=false in this adapter: the robot has no
// wheel odometry -- nothing publishes nav_msgs/Odometry, and the navigator
// has zero callers for it today (see localization/doc.go and the port's own
// "a normal state, not an error" note). No sensor, producer, or consumer
// exists, so there is nothing to build.
func (g *Gateway) GetWheelOdometry() (controllers.WheelOdometry, bool) {
	return controllers.WheelOdometry{}, false
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
	min := float64(scan.GetAngleMin())
	inc := float64(scan.GetAngleIncrement())
	for i := range n {
		angles[i] = min + float64(i)*inc
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
func (g *Gateway) Run(
	ctx context.Context,
	scanSub *natsx.Subscriber[*sensorv1.Scan],
	imuSub *natsx.Subscriber[*sensorv1.Imu],
) error {
	scanErr := make(chan error, 1)
	imuErr := make(chan error, 1)
	go func() { scanErr <- g.scanLoop(ctx, scanSub) }()
	go func() { imuErr <- g.imuLoop(ctx, imuSub) }()

	select {
	case err := <-scanErr:
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return nil
		}
		return err
	case err := <-imuErr:
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return nil
		}
		return err
	}
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
