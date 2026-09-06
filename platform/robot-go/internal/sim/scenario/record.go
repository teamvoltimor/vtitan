package scenario

import (
	"fmt"
	"log/slog"
	"math"
	"path/filepath"
	"slices"
	"time"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navigator"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/recording"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/kinematics"
)

// scanFrameID is the LIDAR's frame in the robot's TF tree, matching what the
// real driver stamps onto /scan so a sim bag and a hardware bag place their
// scans in the same frame.
const scanFrameID = "lidar_link"

// The ROS topic names the Python stack records, matching
// scripts/common/bag_io.Topics so every diag_bag_*.py script reads a sim run
// the same way it reads a pulled one.
const (
	scanTopic       = "/scan"
	imuTopic        = "/imu/data"
	tfTopic         = "/tf"
	ackermannTopic  = "/ackermann_cmd"
	planTopic       = "/plan"
	navDebugTopic   = "/nav_debug"
	driveSpeedTopic = "/motor/drive_speed"
	steeringTopic   = "/motor/steering_position"
)

// TF frames. mapFrame is the track's fixed world frame and baseFrame the
// chassis; scanFrameID hangs off baseFrame so a viewer draws the sweep from
// wherever the robot currently is.
const (
	mapFrame  = "map"
	baseFrame = "base_link"
)

// degreesPerRadian converts the SI values the simulator works in into the
// units two of the motor topics are published in on the robot.
const degreesPerRadian = 180.0 / math.Pi

// nanosPerSecond converts the simulation's float seconds into MCAP log time.
const nanosPerSecond = 1e9

// simRecorder writes one scenario's run to an MCAP bag.
//
// It records two channels, deliberately in DIFFERENT encodings.
//
// /scan is ROS2 CDR (sensor_msgs/msg/LaserScan), byte-compatible with what
// rosbag2 records on the robot. That is not a stylistic choice: Foxglove
// Studio renders sensor_msgs/msg/LaserScan natively, while a custom
// protobuf like vtitan.sensor.v1.Scan appears only under Raw Messages and
// draws nothing in the 3D panel -- which reads as an EMPTY BAG to anyone
// comparing against a pulled hardware run. It also means the Python
// diag_bag_*.py suite and test/bagreplay, both of which read /scan as CDR,
// work on a sim run unchanged.
//
// The navigator debug snapshot stays protobuf on its NATS subject, because
// no ROS message describes it and the Python stack records its own version
// as an opaque JSON string. Mixed encodings in one bag are legal MCAP and
// every reader here handles both.
//
// A nil *simRecorder is valid and does nothing, so the run loop needs no
// branch around each call.
type simRecorder struct {
	run *recording.RunRecorder
	// wheelRadiusM converts the body's linear speed into the WHEEL angular
	// rate /motor/drive_speed carries. That topic is in DEGREES PER SECOND,
	// not rpm and not m/s -- a trap that has already cost a 6x
	// misinterpretation of recorded data once.
	wheelRadiusM float64
	// maxSteerRad converts the navigator's normalised steering command into
	// the physical road-wheel angle /ackermann_cmd and
	// /motor/steering_position carry.
	maxSteerRad float64
	// lidarXOffsetM/lidarZOffsetM place the sensor on the chassis, from
	// robot.toml's [lidar] mount offsets.
	lidarXOffsetM float64
	lidarZOffsetM float64
	// lastPlan is the most recently published route, so /plan is written on
	// REPLANS rather than every tick. A three-lap round is ~1800 ticks and a
	// plan is ~100 poses; republishing an unchanged route would be most of
	// the bag and would hide the thing worth seeing, which is WHEN the path
	// moved. Foxglove holds the last message on a topic, so the drawn plan
	// is still correct between replans.
	lastPlan []trackmodel.Waypoint
	// prevYawRad backs the yaw RATE published on /imu/data, which the
	// kinematic state does not carry directly.
	prevYawRad float64
	haveYaw    bool
	// simClockNanos is the bag's time base. Log time comes from the
	// SIMULATION clock, not the wall clock: a 640-scenario sweep runs
	// concurrently and finishes in seconds, so wall-clock stamps would
	// compress a 110-second round into a smear and make Foxglove's timeline
	// useless. Sim time also makes two runs of the same scenario line up
	// tick for tick.
	simClockNanos uint64
}

// newSimRecorder creates the run directory for one scenario under root and
// opens its bag. Returns nil when root is empty (recording off).
func newSimRecorder(root, scenarioID string, geom recorderGeometry) (*simRecorder, error) {
	if root == "" {
		return nil, nil //nolint:nilnil // a nil recorder is the documented "off" state
	}
	run, err := recording.NewRun(root, recording.RunOptions{Name: scenarioID})
	if err != nil {
		return nil, fmt.Errorf("sim recorder: %w", err)
	}
	if err := run.Open(); err != nil {
		return nil, fmt.Errorf("sim recorder: %w", err)
	}
	return &simRecorder{
		run:           run,
		wheelRadiusM:  geom.WheelRadiusM,
		maxSteerRad:   geom.MaxSteerRad,
		lidarXOffsetM: geom.LidarXOffsetM,
		lidarZOffsetM: geom.LidarZOffsetM,
	}, nil
}

// recorderGeometry is the chassis geometry the bag needs but the control
// loop does not: wheel size to express speed as a wheel rate, steering limit
// to turn a normalised command into an angle, and the LIDAR mount to place
// the sensor frame.
type recorderGeometry struct {
	WheelRadiusM  float64
	MaxSteerRad   float64
	LidarXOffsetM float64
	LidarZOffsetM float64
}

// tick writes one control step's scan and debug snapshot, then advances the
// bag clock by dt. Errors are returned rather than logged: a bag that
// silently stopped recording halfway is worse than a failed run, because the
// gap looks like the robot rather than the recorder.
func (r *simRecorder) tick(
	scan controllers.LidarScan, ok bool,
	nav *navigator.Navigator, state kinematics.AckermannState,
	reportedYaw float64, dt float64,
) error {
	if r == nil {
		return nil
	}
	logTime := r.simClockNanos
	sec, nsec := splitStamp(logTime)

	if ok && len(scan.RangesM) > 0 {
		if err := r.run.WriteROS2(
			scanTopic, recording.LaserScanType, recording.LaserScanSchema,
			recording.EncodeLaserScan(scanToCDR(scan, logTime)), logTime,
		); err != nil {
			return fmt.Errorf("sim recorder: writing scan: %w", err)
		}
	}

	// Two transforms, and both are needed for the scan to land in the right
	// place: map->base_link says where the body is, base_link->lidar_link
	// where the sensor sits on it. Without the second, a viewer has a scan in
	// a frame it has never heard of and draws nothing.
	//
	// map->base_link is GROUND TRUTH -- where the body actually is, which is
	// what a viewer should draw. The IMU below carries what the robot
	// BELIEVES instead, so a run with sensor errors configured shows the two
	// diverging, which is the whole point of recording both.
	//
	// base_link->lidar_link carries the mount OFFSET but no rotation. That is
	// correct for the simulator specifically: its raycast produces angles
	// already in the robot frame (0 = straight ahead), so the physical
	// mount's inversion and yaw offset -- which the real driver must correct
	// for -- have no analogue here. A rotation would double-apply a
	// correction the sim never applied in the first place.
	if err := r.run.WriteROS2(
		tfTopic, recording.TFMessageType, recording.TFMessageSchema,
		recording.EncodeTFMessage([]recording.TransformCDR{
			{
				FrameID: mapFrame, ChildFrameID: baseFrame,
				StampSec: sec, StampNanosec: nsec,
				X: state.X, Y: state.Y, YawRad: state.Yaw,
			},
			{
				FrameID: baseFrame, ChildFrameID: scanFrameID,
				StampSec: sec, StampNanosec: nsec,
				X: r.lidarXOffsetM, Z: r.lidarZOffsetM,
			},
		}), logTime,
	); err != nil {
		return fmt.Errorf("sim recorder: writing tf: %w", err)
	}

	yawRate := 0.0
	if r.haveYaw {
		yawRate = navutil.WrapAngle(reportedYaw-r.prevYawRad) / dt
	}
	r.prevYawRad, r.haveYaw = reportedYaw, true
	if err := r.run.WriteROS2(
		imuTopic, recording.ImuType, recording.ImuSchema,
		recording.EncodeImu(recording.ImuCDR{
			FrameID: baseFrame, StampSec: sec, StampNanosec: nsec,
			YawRad: reportedYaw, YawRateRadPS: yawRate,
		}), logTime,
	); err != nil {
		return fmt.Errorf("sim recorder: writing imu: %w", err)
	}

	// The command comes off the debug snapshot rather than from the gateway:
	// it is already the value the navigator issued this tick, and reading it
	// there avoids widening the gateway interface for something only the
	// recorder wants. Nil before the first command (the blind creep phase
	// issues none), which records as a zero command -- accurate, since
	// nothing was sent.
	debug := nav.DebugSnapshot()
	var cmdSpeed, cmdSteerNorm float64
	if debug.CommandedSpeedMPS != nil {
		cmdSpeed = *debug.CommandedSpeedMPS
	}
	if debug.CommandedSteerNorm != nil {
		cmdSteerNorm = *debug.CommandedSteerNorm
	}
	steerRad := cmdSteerNorm * r.maxSteerRad
	if err := r.run.WriteROS2(
		ackermannTopic, recording.AckermannType, recording.AckermannSchema,
		recording.EncodeAckermannDriveStamped(recording.AckermannCDR{
			FrameID: baseFrame, StampSec: sec, StampNanosec: nsec,
			SteeringAngleRad: float32(steerRad), SpeedMPS: float32(cmdSpeed),
		}), logTime,
	); err != nil {
		return fmt.Errorf("sim recorder: writing ackermann_cmd: %w", err)
	}

	// Both motor topics are ACHIEVED values, read back off the body, not the
	// command above -- that is what the robot's own motor node publishes,
	// and the gap between the two is where a rate limit or a speed lag shows
	// up. drive_speed is the WHEEL rate in deg/s (see wheelRadiusM);
	// steering_position is the road-wheel angle in degrees.
	driveSpeedDegPS := 0.0
	if r.wheelRadiusM > 0 {
		driveSpeedDegPS = state.V / r.wheelRadiusM * degreesPerRadian
	}
	if err := r.run.WriteROS2(
		driveSpeedTopic, recording.Float32Type, recording.Float32Schema,
		recording.EncodeFloat32(float32(driveSpeedDegPS)), logTime,
	); err != nil {
		return fmt.Errorf("sim recorder: writing drive_speed: %w", err)
	}
	if err := r.run.WriteROS2(
		steeringTopic, recording.Float32Type, recording.Float32Schema,
		recording.EncodeFloat32(float32(state.Steer*degreesPerRadian)), logTime,
	); err != nil {
		return fmt.Errorf("sim recorder: writing steering_position: %w", err)
	}

	// The plan is drawn in the same frame the pose is reported in, which is
	// what makes the two comparable. On a BLIND run that frame is the
	// robot's believed one -- it plans from an assumed start -- so the path
	// appears where the navigator thinks it is going, which is exactly the
	// view that explains a blind failure.
	if plan := nav.Waypoints(); !slices.Equal(plan, r.lastPlan) {
		points := make([]recording.PathPointCDR, len(plan))
		for i, wp := range plan {
			points[i] = recording.PathPointCDR{X: wp.X, Y: wp.Y}
		}
		if err := r.run.WriteROS2(
			planTopic, recording.PathType, recording.PathSchema,
			recording.EncodePath(recording.PathCDR{
				FrameID: mapFrame, StampSec: sec, StampNanosec: nsec, Points: points,
			}), logTime,
		); err != nil {
			return fmt.Errorf("sim recorder: writing plan: %w", err)
		}
		r.lastPlan = slices.Clone(plan)
	}

	// /nav_debug is the std_msgs/String JSON the Python navigator publishes,
	// and carrying it is what lets the existing diag_bag_*.py suite -- corner
	// overshoot, escape, steer headroom, direction gates, a dozen more -- run
	// against a SIMULATED run unmodified.
	//
	// The snapshot used to ALSO go out as protobuf on its NATS subject, which
	// is the Go stack's own representation. That is gone, and not for tidiness:
	// rosbag2 refuses to open a bag whose topics do not share one serialization
	// format ("Topics with different rmw serialization format have been
	// found"), so the single protobuf channel made the bag unreadable by every
	// Python script -- the exact tooling this topic exists to unlock. Foxglove
	// tolerates the mix; rosbag2 does not, and rosbag2 is the stricter
	// consumer. Nothing reads the protobuf channel today, and a Go consumer
	// can parse the JSON.
	wire, err := debug.MarshalWireJSON()
	if err != nil {
		return fmt.Errorf("sim recorder: encoding nav debug JSON: %w", err)
	}
	if err := r.run.WriteROS2(
		navDebugTopic, recording.StringType, recording.StringSchema,
		recording.EncodeString(string(wire)), logTime,
	); err != nil {
		return fmt.Errorf("sim recorder: writing nav debug JSON: %w", err)
	}
	r.simClockNanos += uint64(dt * nanosPerSecond)
	return nil
}

// splitStamp converts the bag clock into the sec/nanosec pair a ROS header
// carries.
func splitStamp(nanos uint64) (sec int32, nsec uint32) {
	return int32(nanos / nanosPerSecond), uint32(nanos % nanosPerSecond)
}

// close finalizes the bag.
func (r *simRecorder) close() error {
	if r == nil {
		return nil
	}
	if err := r.run.Close(); err != nil {
		return fmt.Errorf("sim recorder: closing bag: %w", err)
	}
	return nil
}

// scanToCDR converts the simulator's scan into sensor_msgs/msg/LaserScan.
//
// The angle fan is regenerated as min/increment rather than carried
// per-sample, because that is the shape LaserScan has: a start angle and a
// fixed step. The simulator's fan is uniform by construction, so nothing is
// lost.
func scanToCDR(scan controllers.LidarScan, stampNanos uint64) recording.LaserScanCDR {
	ranges := make([]float32, len(scan.RangesM))
	for i, r := range scan.RangesM {
		ranges[i] = float32(r)
	}

	var angleMin, angleMax, increment float64
	if n := len(scan.AnglesRad); n > 0 {
		angleMin = scan.AnglesRad[0]
		angleMax = scan.AnglesRad[n-1]
		if n > 1 {
			increment = (angleMax - angleMin) / float64(n-1)
		}
	}

	rangeMin, rangeMax := math.Inf(1), 0.0
	for _, r := range scan.RangesM {
		rangeMin = math.Min(rangeMin, r)
		rangeMax = math.Max(rangeMax, r)
	}
	if math.IsInf(rangeMin, 1) {
		rangeMin = 0
	}

	return recording.LaserScanCDR{
		FrameID:      scanFrameID,
		StampSec:     int32(stampNanos / nanosPerSecond),
		StampNanosec: uint32(stampNanos % nanosPerSecond),
		AngleMin:     float32(angleMin),
		AngleMax:     float32(angleMax),
		AngleIncr:    float32(increment),
		RangeMin:     float32(rangeMin),
		RangeMax:     float32(rangeMax),
		Ranges:       ranges,
	}
}

// SimRunsRootFor resolves where a sweep's bags go: the caller's explicit
// directory, else a timestamped sweep directory under the repo-root
// data/sim/runs tree.
//
// The sweep gets its own directory because a sweep is the unit anyone
// compares: "the 640 cases I ran before the change" against "the 640 after".
// Dropping every scenario straight into data/sim/runs/ would interleave two
// sweeps' bags under names that collide on the scenario ID.
func SimRunsRootFor(explicit string, now time.Time) (string, error) {
	if explicit != "" {
		return explicit, nil
	}
	root, err := recording.SimRunsRoot()
	if err != nil {
		return "", fmt.Errorf("sim recorder: %w", err)
	}
	return filepath.Join(root, "sweep_"+now.Format("20060102_150405")), nil
}

// defaultWheelRadiusM matches robot.toml's [wheel] radius, used when no
// config root is supplied so a recorded run without one still reports a
// plausible wheel rate rather than dividing by zero.
const defaultWheelRadiusM = 0.035

// recorderGeometryFor reads the recorder's chassis geometry from the shipped
// robot.toml, falling back to the defaults so a run without a config root
// still records plausible telemetry rather than dividing by zero.
func recorderGeometryFor(
	logger *slog.Logger, configRoot string, hardwareProfileNames []string, maxSteerRad float64,
) recorderGeometry {
	geom := recorderGeometry{WheelRadiusM: defaultWheelRadiusM, MaxSteerRad: maxSteerRad}
	if configRoot == "" {
		return geom
	}
	loaded, err := profile.LoadRobotConfig(
		filepath.Join(configRoot, profile.DefaultRobotTOMLPath), hardwareProfileNames,
	)
	if err != nil {
		logger.Warn("sim recorder: reading robot.toml, using default geometry", "error", err)
		return geom
	}
	if loaded.Wheel.Radius > 0 {
		geom.WheelRadiusM = loaded.Wheel.Radius
	}
	geom.LidarXOffsetM = loaded.Lidar.MountXOffset
	geom.LidarZOffsetM = loaded.Lidar.MountZOffset
	return geom
}
