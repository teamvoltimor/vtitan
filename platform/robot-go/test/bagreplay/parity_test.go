package bagreplay_test

import (
	"log/slog"
	"math"
	"os"
	"path/filepath"
	"sort"
	"sync"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navigator"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
	navnode "github.com/teamvoltimor/vtitan/platform/robot-go/internal/node/nav"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/platform/robot-go/test/bagreplay"
)

// repoRootFromPackageDir is the relative path from this package
// (platform/robot-go/test/bagreplay) back to the repo root, where
// profile.DefaultRobotTOMLPath (platform/config/robot.toml) lives.
const repoRootFromPackageDir = "../../../.."

// lidarYawOffsetRadForBags mirrors the mandatory rotation the real ROS2
// hardware gateway applies before the Python navigator ever sees a bearing
// (ros2_hardware_gateway.py's _LIDAR_YAW_OFFSET_RAD). robot.toml's
// `[lidar] inverted = true` -- the chassis mount is physically upside-down --
// is set in the BASE config, not behind a hardware profile, so no
// VTITAN_HARDWARE_PROFILE override is needed to reproduce it here. Without
// this, replayed bearings are 180deg off from what Python's escape/collision
// logic actually saw, which was misdiagnosed once already as a navigator
// behavior bug (see doc.go).
//
// Memoized with sync.OnceValue: toLidarScan runs once per scan row (tens of
// thousands of times per bag), and re-parsing robot.toml through viper on
// every call is not just slow but crashes mapstructure under the load.
//
// This deliberately models the PYTHON stack, so it stays a rotation even
// though the Go driver corrects an upside-down mount with a mirror (see
// lidar.correctAngleDeg). The two are not in conflict and must not be
// unified: Python consumes sllidar_ros2's output, which has already applied
// its own `inverted` mirror at the driver, and adds 180deg on top; Go
// consumes raw serial and applies the whole correction itself. Replay has to
// reproduce the frame Python's navigator actually saw, not the frame Go
// would have built from the same sensor.
var lidarYawOffsetRadForBags = sync.OnceValue(func() float64 {
	basePath := filepath.Join(repoRootFromPackageDir, profile.DefaultRobotTOMLPath)
	cfg, err := profile.Load[profile.RobotConfig](basePath, profile.ActiveNames())
	if err != nil {
		slog.Default().Warn("bagreplay: loading robot.toml, replaying with no LIDAR yaw offset",
			"error", err)
		return 0
	}
	if !cfg.Lidar.Inverted {
		return cfg.Lidar.MountYawOffsetDeg * math.Pi / 180
	}
	return (180 + cfg.Lidar.MountYawOffsetDeg) * math.Pi / 180
})

// parityGateway is the in-memory controllers.HardwareGateway the replay drives
// the Go navigator through. Each Step reads the staged pose and the most
// recent /scan, isolating the navigator under test exactly as doc.go intends:
// the pose is the bag's recorded pose, the scan is the bag's recorded /scan,
// so any divergence from /nav_debug is navigator behavior, not localization.
type parityGateway struct {
	pose     trackmodel.Pose
	havePose bool
	scan     controllers.LidarScan
	haveScan bool
}

func (g *parityGateway) PublishDrive(controllers.DriveCommand) {}
func (g *parityGateway) GetCurrentPose() (trackmodel.Pose, bool) {
	return g.pose, g.havePose
}
func (g *parityGateway) GetLidarScan() (controllers.LidarScan, bool) {
	return g.scan, g.haveScan
}
func (g *parityGateway) GetWheelOdometry() (controllers.WheelOdometry, bool) {
	return controllers.WheelOdometry{}, false
}
func (g *parityGateway) SetBelievedWalls(*trackmodel.TrackWalls)  {}
func (g *parityGateway) ResetPosition(float64, float64)           {}
func (g *parityGateway) ResetHeadingReference()                   {}
func (g *parityGateway) CorrectHeadingForDirectionChange(float64) {}

// reconstructPath builds the Python navigator's driven path from the recorded
// steer_target points, keyed by waypoint_index. The bag does not store the
// path, but on every normal_drive tick the Python navigator published exactly
// the lookahead point it was aiming at (steer_target_x/y), so collecting those
// per index reconstructs the path it actually drove. The Go navigator re-seeks
// the nearest waypoint on its own, so an approximate path is enough to pin
// crosstrack/steer parity.
func reconstructPath(rows []bagreplay.NavDebugRow) []trackmodel.Waypoint {
	type accum struct {
		xs, ys []float64
	}
	byIndex := map[int]*accum{}
	maxIndex := -1
	for _, r := range rows {
		wi := r.Snapshot.WaypointIndex
		sx := r.Snapshot.SteerTargetX
		sy := r.Snapshot.SteerTargetY
		if wi == nil || sx == nil || sy == nil {
			continue
		}
		a, ok := byIndex[*wi]
		if !ok {
			a = &accum{}
			byIndex[*wi] = a
		}
		a.xs = append(a.xs, *sx)
		a.ys = append(a.ys, *sy)
		if *wi > maxIndex {
			maxIndex = *wi
		}
	}
	if maxIndex < 0 {
		return nil
	}
	path := make([]trackmodel.Waypoint, maxIndex+1)
	for i := 0; i <= maxIndex; i++ {
		a := byIndex[i]
		if a == nil {
			// A gap index: leave a zero waypoint; the navigator will simply
			// skip toward the next real one. Acceptable for a parity probe.
			continue
		}
		path[i] = trackmodel.Waypoint{X: median(a.xs), Y: median(a.ys)}
	}
	return path
}

func median(xs []float64) float64 {
	sort.Float64s(xs)
	return xs[len(xs)/2]
}

// TestParity_NavigatorVsBag is the first real parity gate: it feeds a bag's
// recorded pose + /scan through the Go navigator and diffs its per-tick debug
// snapshot against the bag's own recorded /nav_debug. It reports field-by-field
// diff stats, not just pass/fail.
//
// Parking and BlindCreep fields are deliberately skipped: the Go port never
// emits them (see node/nav/debug.go's DebugFor doc, and navigator/doc.go's
// scope list), so their absence is an accepted, documented gap rather than a
// parity failure.
func TestParity_NavigatorVsBag(t *testing.T) {
	dir := parityBagDir(t)

	navRows, err := bagreplay.ReadNavDebug(dir)
	if err != nil {
		t.Fatalf("ReadNavDebug: %v", err)
	}
	scanRows, err := bagreplay.ReadScan(dir)
	if err != nil {
		t.Fatalf("ReadScan: %v", err)
	}
	if len(navRows) == 0 || len(scanRows) == 0 {
		t.Skipf("bag has no /nav_debug (%d) or /scan (%d)", len(navRows), len(scanRows))
	}

	path := reconstructPath(navRows)
	if len(path) == 0 {
		t.Skip("bag never reached a sighted normal_drive phase; cannot reconstruct a path")
	}

	// Direction comes from the bag's recorded travel direction.
	var direction trackmodel.Direction
	for _, r := range navRows {
		if r.Snapshot.Direction != nil {
			switch *r.Snapshot.Direction {
			case "clockwise":
				direction = trackmodel.Clockwise
			case "counterclockwise":
				direction = trackmodel.Counterclockwise
			}
			break
		}
	}

	gw := &parityGateway{}
	nav, err := navigator.New(navigator.Params{
		Gateway:           gw,
		Waypoints:         path,
		Direction:         func() *trackmodel.Direction { d := direction; return &d }(),
		Config:            navigator.DefaultConfig(),
		ControllersConfig: controllers.DefaultConfig(),
	})
	if err != nil {
		t.Fatalf("navigator.New: %v", err)
	}

	stats := newParityStats()
	processed := 0
	for i, navRow := range navRows {
		ref := navRow.Snapshot

		// Stage the recorded pose, when present.
		if ref.PoseX != nil && ref.PoseY != nil && ref.PoseYaw != nil {
			gw.pose = trackmodel.Pose{X: *ref.PoseX, Y: *ref.PoseY, Yaw: *ref.PoseYaw}
			gw.havePose = true
		} else {
			gw.havePose = false
		}

		// Stage the most recent /scan at or before this tick.
		gw.haveScan = false
		for _, s := range scanRows {
			if s.ElapsedS > navRow.ElapsedS {
				break
			}
			gw.scan = toLidarScan(s.Scan)
			gw.haveScan = true
		}

		nav.Step()
		got := nav.DebugSnapshot()

		// Exercise the documented wire path (node/nav.DebugFor) so the
		// parity gate covers the conversion the live node actually publishes,
		// not just the navigator's internal snapshot. It must not panic.
		if dbg := navnode.DebugFor(got); dbg == nil {
			t.Fatalf("DebugFor returned nil on tick %d", i)
		}

		stats.compare(got, ref, gw.haveScan)
		processed++
	}

	report := stats.report()
	t.Logf("processed %d ticks from %s (path=%d waypoints, dir=%s)",
		processed, dir, len(path), direction.String())
	for _, line := range report {
		t.Log(line)
	}
	stats.reportPerception(t)
}

// toLidarScan converts a decoded ROS2 LaserScan into the Go navigator's scan
// type, building the per-ray angle array the collision controller expects.
// Mirrors ros2_hardware_gateway.py's _lidar_callback: rotate raw bearings by
// the mandatory LIDAR yaw offset (see lidarYawOffsetRadForBags) and replace
// non-finite ranges with LidarMaxRangeM (sanitize_lidar_ranges), so the
// navigator sees the same robot-frame scan it would live on hardware.
func toLidarScan(s bagreplay.LaserScan) controllers.LidarScan {
	yawOffset := lidarYawOffsetRadForBags()
	n := len(s.RangesM)
	ranges := make([]float64, n)
	angles := make([]float64, n)
	for i := range s.RangesM {
		r := float64(s.RangesM[i])
		if math.IsNaN(r) || math.IsInf(r, 0) {
			r = controllers.DefaultLidarMaxRangeM
		}
		ranges[i] = r
		angles[i] = float64(s.AngleMin) + float64(s.AngleIncrement)*float64(i) + yawOffset
	}
	return controllers.LidarScan{RangesM: ranges, AnglesRad: angles}
}

func parityBagDir(t *testing.T) string {
	t.Helper()
	if override := os.Getenv("VTITAN_BAG_DIR"); override != "" {
		return override
	}
	// Documented complete sighted bag from an earlier session, in the shared
	// repo-root data/live/runs tree (see internal/recording/root.go).
	return filepath.Join(repoRootFromPackageDir, "data", "live", "runs", "run_20260829_140424")
}
