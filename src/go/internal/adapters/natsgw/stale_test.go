package natsgw

import (
	"testing"
	"time"

	natstest "github.com/nats-io/nats-server/v2/test"
	"github.com/nats-io/nats.go"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/localization"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
)

// fakeClock is a settable clock for the staleness gate.
type fakeClock struct{ t time.Time }

const testStaleTimeout = 500 * time.Millisecond

func (c *fakeClock) now() time.Time { return c.t }

// receiveScan stands in for scanLoop's cache update.
func receiveScan(g *Gateway) {
	g.mu.Lock()
	g.scan = &sensorv1.Scan{AngleIncrement: 0.1, Ranges: []float32{1, 1}}
	g.scanAt = g.clock()
	g.pose = trackmodel.Pose{X: 1, Y: 2}
	g.havePose = true
	g.mu.Unlock()
}

// receiveIMU stands in for imuLoop's cache update.
func receiveIMU(g *Gateway) {
	g.mu.Lock()
	g.imu = &sensorv1.Imu{Orientation: &sensorv1.Quaternion{W: 1}}
	g.imuAt = g.clock()
	g.mu.Unlock()
}

// A LIDAR that stops publishing must stop the navigator: with scan and pose
// both ok=false, Step publishes a zero command. Before this gate the last
// scan was served forever, the navigator kept publishing, and the motor
// node's command watchdog never had reason to fire.
func TestStaleScan_WithdrawsScanAndPose(t *testing.T) {
	t.Parallel()

	clk := &fakeClock{t: time.Unix(1000, 0)}
	g := &Gateway{staleTimeout: testStaleTimeout, now: clk.now}

	receiveScan(g)
	if _, ok := g.GetLidarScan(); !ok {
		t.Fatal("fresh scan: GetLidarScan ok=false")
	}
	if _, ok := g.GetCurrentPose(); !ok {
		t.Fatal("fresh scan: GetCurrentPose ok=false")
	}

	clk.t = clk.t.Add(testStaleTimeout)
	if _, ok := g.GetLidarScan(); !ok {
		t.Fatal("scan exactly at the timeout: GetLidarScan ok=false, want the boundary inclusive like Python's >")
	}

	clk.t = clk.t.Add(time.Millisecond)
	if _, ok := g.GetLidarScan(); ok {
		t.Error("stale scan: GetLidarScan ok=true")
	}
	if _, ok := g.GetCurrentPose(); ok {
		t.Error("stale scan: GetCurrentPose ok=true")
	}

	receiveScan(g)
	if _, ok := g.GetLidarScan(); !ok {
		t.Error("scan after recovery: GetLidarScan ok=false")
	}
	if _, ok := g.GetCurrentPose(); !ok {
		t.Error("scan after recovery: GetCurrentPose ok=false")
	}
}

// Before the first scan the pose is whatever ResetPosition seeded: the gate
// only withdraws a pose once a feed exists to go stale, as in Python.
func TestStaleScan_SeededPoseBeforeFirstScanIsUntouched(t *testing.T) {
	t.Parallel()

	clk := &fakeClock{t: time.Unix(1000, 0)}
	g := &Gateway{staleTimeout: testStaleTimeout, now: clk.now}
	g.ResetPosition(0.5, 0.5)

	clk.t = clk.t.Add(time.Hour)
	if _, ok := g.GetCurrentPose(); !ok {
		t.Error("seeded pose with no scans yet: GetCurrentPose ok=false")
	}
	if _, ok := g.GetLidarScan(); ok {
		t.Error("no scans yet: GetLidarScan ok=true")
	}
}

func TestNew_RefusesNonPositiveStaleTimeout(t *testing.T) {
	t.Parallel()

	// New validates conn and walls before the timeout, so both must be real
	// or the error under test is never reached.
	srv := natstest.RunRandClientPortServer()
	t.Cleanup(srv.Shutdown)
	conn, err := nats.Connect(srv.ClientURL())
	if err != nil {
		t.Fatalf("connecting to the test server: %v", err)
	}
	t.Cleanup(conn.Close)
	walls := trackmodel.NewTrackWalls(trackmodel.CorridorGeometry{}, -1.5, 1.5)

	if _, err = New(conn, walls, localization.DefaultConfig(), 0.03, testStaleTimeout); err != nil {
		t.Fatalf("New with a valid stale timeout: %v", err)
	}
	for _, d := range []time.Duration{0, -time.Second} {
		if _, err = New(conn, walls, localization.DefaultConfig(), 0.03, d); err == nil {
			t.Errorf("New with stale timeout %v: no error", d)
		}
	}
}

// An IMU that stops publishing must withdraw the pose: its yaw scores every
// scan, so a frozen heading freezes the pose. Before this gate a dead IMU left
// GetCurrentPose serving a pose built from its last orientation forever, and
// the navigator kept steering on it.
func TestStaleIMU_WithdrawsPose(t *testing.T) {
	t.Parallel()

	clk := &fakeClock{t: time.Unix(1000, 0)}
	g := &Gateway{staleTimeout: testStaleTimeout, now: clk.now}

	receiveIMU(g)
	receiveScan(g)
	if _, ok := g.GetCurrentPose(); !ok {
		t.Fatal("fresh IMU: GetCurrentPose ok=false")
	}

	clk.t = clk.t.Add(testStaleTimeout)
	if _, ok := g.GetCurrentPose(); !ok {
		t.Fatal("IMU exactly at the timeout: GetCurrentPose ok=false, want the boundary inclusive like the scan gate")
	}

	clk.t = clk.t.Add(time.Millisecond)
	// Refresh only the scan: the pose must still be withdrawn, because the yaw
	// it is scored with is the stale one.
	receiveScan(g)
	if _, ok := g.GetLidarScan(); !ok {
		t.Error("stale IMU withdrew a fresh scan: GetLidarScan ok=false")
	}
	if _, ok := g.GetCurrentPose(); ok {
		t.Error("stale IMU: GetCurrentPose ok=true")
	}

	receiveIMU(g)
	if _, ok := g.GetCurrentPose(); !ok {
		t.Error("IMU after recovery: GetCurrentPose ok=false")
	}
}
