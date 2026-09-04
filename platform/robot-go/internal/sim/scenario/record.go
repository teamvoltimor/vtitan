package scenario

import (
	"fmt"
	"math"
	"path/filepath"
	"time"

	"google.golang.org/protobuf/types/known/timestamppb"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navigator"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/recording"
	navv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/nav/v1"
	sensorv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/sensor/v1"
)

// scanFrameID is the LIDAR's frame in the robot's TF tree, matching what the
// real driver stamps onto /scan so a sim bag and a hardware bag place their
// scans in the same frame.
const scanFrameID = "lidar_link"

// nanosPerSecond converts the simulation's float seconds into MCAP log time.
const nanosPerSecond = 1e9

// simRecorder writes one scenario's run to an MCAP bag.
//
// It records exactly the two subjects cmd/track-navigator --record writes on
// hardware -- the LIDAR scan and the navigator debug snapshot -- so a sim bag
// and a Go hardware bag are the same format and Foxglove Studio opens both
// directly. A third subject here would be a sim-only channel that no hardware
// bag has, and every consumer would have to learn which kind it was holding.
//
// NOT the same format as test/bagreplay reads. That package replays the
// ROS2/rosbag2 bags the PYTHON stack records: same MCAP container, but CDR
// message encoding on ROS topic names (/scan, /nav_debug), where this writes
// protobuf on NATS subjects. Both Go recorders have that gap; it is not
// introduced here.
//
// A nil *simRecorder is valid and does nothing, so the run loop needs no
// branch around each call.
type simRecorder struct {
	run *recording.RunRecorder
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
func newSimRecorder(root, scenarioID string) (*simRecorder, error) {
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
	return &simRecorder{run: run}, nil
}

// tick writes one control step's scan and debug snapshot, then advances the
// bag clock by dt. Errors are returned rather than logged: a bag that
// silently stopped recording halfway is worse than a failed run, because the
// gap looks like the robot rather than the recorder.
func (r *simRecorder) tick(scan controllers.LidarScan, ok bool, nav *navigator.Navigator, dt float64) error {
	if r == nil {
		return nil
	}
	logTime := r.simClockNanos
	if ok && len(scan.RangesM) > 0 {
		if err := r.run.WriteMessage(sensorv1.ScanSubject, scanToProto(scan, logTime), logTime); err != nil {
			return fmt.Errorf("sim recorder: writing scan: %w", err)
		}
	}
	if err := r.run.WriteMessage(
		navv1.NavigatorDebugSubject, nav.DebugSnapshot().ToProto(), logTime,
	); err != nil {
		return fmt.Errorf("sim recorder: writing nav debug: %w", err)
	}
	r.simClockNanos += uint64(dt * nanosPerSecond)
	return nil
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

// scanToProto converts the simulator's scan into the wire message the real
// LIDAR driver publishes.
//
// The angle fan is regenerated as min/increment rather than carried
// per-sample, because that is the shape sensor_msgs/LaserScan (and this
// repo's mirror of it) has: a start angle and a fixed step. The simulator's
// fan is uniform by construction, so nothing is lost.
func scanToProto(scan controllers.LidarScan, stampNanos uint64) *sensorv1.Scan {
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

	return &sensorv1.Scan{
		Stamp:          timestamppb.New(time.Unix(0, int64(stampNanos))),
		FrameId:        scanFrameID,
		AngleMin:       float32(angleMin),
		AngleMax:       float32(angleMax),
		AngleIncrement: float32(increment),
		RangeMin:       float32(rangeMin),
		RangeMax:       float32(rangeMax),
		Ranges:         ranges,
	}
}

// SimRunsRootFor resolves where a sweep's bags go: the caller's explicit
// directory, else a timestamped sweep directory under the repo-root
// data/runs_sim tree.
//
// The sweep gets its own directory because a sweep is the unit anyone
// compares: "the 640 cases I ran before the change" against "the 640 after".
// Dropping every scenario straight into runs_sim/ would interleave two
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
