package navigator_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/corridorestimator"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navigator"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// corridorScan builds a scan of a corridor of the given width, seen from its
// centre by a chassis aligned to it: the ray directly left plus the ray
// directly right span wall to wall, which is what MeasureCorridorWidth reads.
func corridorScan(widthM, yaw float64) controllers.LidarScan {
	const n = 360
	ranges := make([]float64, n)
	angles := make([]float64, n)
	half := widthM / 2.0
	for i := range n {
		a := wrapPi(float64(i) / float64(n) * 2 * math.Pi)
		angles[i] = a
		switch {
		case math.Abs(wrapPi(a-math.Pi/2)) < 0.05, math.Abs(wrapPi(a+math.Pi/2)) < 0.05:
			ranges[i] = half
		default:
			// Open ahead and behind, so nothing else gates the reading.
			ranges[i] = 3.0
		}
	}
	return controllers.LidarScan{RangesM: ranges, AnglesRad: angles}
}

// blindNavigator builds a navigator in the blind bootstrap (no Direction),
// staged with a corridor scan of the given width.
func blindNavigator(t *testing.T, widthM, yaw float64) (*navigator.Navigator, *fakeGateway) {
	t.Helper()

	gateway := &fakeGateway{}
	gateway.pose = trackmodel.Pose{X: 1.0, Y: 1.0, Yaw: yaw}
	gateway.havePose = true
	gateway.scan = corridorScan(widthM, yaw)
	gateway.haveScan = true

	nav, err := navigator.New(navigator.Params{
		Gateway:           gateway,
		Waypoints:         squareLoop(),
		Direction:         nil,
		Config:            navigator.DefaultConfig(),
		ControllersConfig: controllers.DefaultConfig(),
		Logger:            discardLogger(),
	})
	if err != nil {
		t.Fatalf("New() error = %v", err)
	}
	return nav, gateway
}

// Before the creep has read anything there is no belief to report, which is
// distinct from believing zero -- the follower must fall back to its own
// wide default rather than classify a 0 m corridor as narrow.
func TestBelievedCreepWidthUnsetBeforeAnyReading(t *testing.T) {
	t.Parallel()
	nav, _ := blindNavigator(t, 1.0, 0.0)

	if _, ok := nav.BelievedCreepWidthM(); ok {
		t.Error("reported a belief before any reading was taken")
	}
}

// The creep is where the cleanest readings of the round are taken -- driving
// straight down a corridor -- so they must be buffered, not discarded.
func TestCreepBuffersWidthReadings(t *testing.T) {
	t.Parallel()
	const widthM = 1.0
	nav, _ := blindNavigator(t, widthM, 0.0)

	nav.Step()

	believed, ok := nav.BelievedCreepWidthM()
	if !ok {
		t.Fatal("no belief after a creep tick on a clean corridor scan")
	}
	if math.Abs(believed-widthM) > 1e-6 {
		t.Errorf("believed width = %v, want %v", believed, widthM)
	}
}

// The belief is the MEAN of the readings, so a corridor read repeatedly
// converges rather than tracking only the latest tick.
func TestBelievedCreepWidthIsTheMeanOfTheReadings(t *testing.T) {
	t.Parallel()
	nav, gateway := blindNavigator(t, 1.0, 0.0)

	nav.Step()
	gateway.scan = corridorScan(0.6, 0.0)
	nav.Step()

	believed, ok := nav.BelievedCreepWidthM()
	if !ok {
		t.Fatal("no belief after two creep ticks")
	}
	if want := 0.8; math.Abs(believed-want) > 1e-6 {
		t.Errorf("believed width = %v, want the mean %v", believed, want)
	}
}

// The readings are votes replayed into the width estimator exactly once, so
// taking them must hand over ownership and leave the buffer empty.
func TestTakeCreepWidthsDrainsTheBuffer(t *testing.T) {
	t.Parallel()
	nav, _ := blindNavigator(t, 1.0, 0.0)
	nav.Step()

	taken := nav.TakeCreepWidths()
	if len(taken) == 0 {
		t.Fatal("took no readings after a creep tick")
	}
	if again := nav.TakeCreepWidths(); len(again) != 0 {
		t.Errorf("second take returned %d readings, want 0", len(again))
	}
	if _, ok := nav.BelievedCreepWidthM(); ok {
		t.Error("still reported a belief after the buffer was drained")
	}
}

// Each reading keeps the heading it was taken at, because the buffer can
// span a turn: attributing all of them by one section label would file the
// pre-turn readings against the post-turn corridor.
func TestCreepWidthsKeepTheHeadingTheyWereTakenAt(t *testing.T) {
	t.Parallel()
	const secondYaw = math.Pi / 2
	nav, gateway := blindNavigator(t, 1.0, 0.0)

	nav.Step()
	gateway.pose = trackmodel.Pose{X: 1.0, Y: 1.0, Yaw: secondYaw}
	gateway.scan = corridorScan(1.0, secondYaw)
	nav.Step()

	taken := nav.TakeCreepWidths()
	if len(taken) != 2 {
		t.Fatalf("buffered %d readings, want 2", len(taken))
	}
	if math.Abs(taken[0].Yaw) > 1e-9 {
		t.Errorf("first reading yaw = %v, want 0", taken[0].Yaw)
	}
	if math.Abs(taken[1].Yaw-secondYaw) > 1e-9 {
		t.Errorf("second reading yaw = %v, want %v", taken[1].Yaw, secondYaw)
	}
}

// An implausible or off-axis scan is exactly what the measurement gates
// exist to reject; a rejected reading must not enter the buffer, or the mean
// the follower acts on is polluted by the readings the gate refused.
func TestCreepIgnoresUnmeasurableScans(t *testing.T) {
	t.Parallel()
	nav, gateway := blindNavigator(t, 1.0, 0.0)

	// Well outside the plausible corridor range in both directions.
	gateway.scan = corridorScan(6.0, 0.0)
	nav.Step()

	if _, ok := nav.BelievedCreepWidthM(); ok {
		t.Error("buffered a reading the plausibility gate should have rejected")
	}
}

// The buffer is capped, oldest dropped first: a creep that never settles
// must not buffer without bound, and the mean the follower acts on should
// describe the corridor the robot is in NOW rather than being dragged back
// by readings from a corridor several turns ago.
func TestCreepWidthBufferIsCappedOldestFirst(t *testing.T) {
	t.Parallel()
	cap := corridorestimator.DefaultConfig().MaxStartSamples
	if cap <= 0 {
		t.Skip("no cap configured")
	}
	nav, gateway := blindNavigator(t, 1.0, 0.0)

	// Fill past the cap with the WIDE reading, then push exactly `cap`
	// NARROW ones in: if the oldest are dropped, nothing wide survives.
	for range cap {
		nav.Step()
	}
	gateway.scan = corridorScan(0.6, 0.0)
	for range cap {
		nav.Step()
	}

	taken := nav.TakeCreepWidths()
	if len(taken) != cap {
		t.Fatalf("buffered %d readings, want the cap %d", len(taken), cap)
	}
	for i, w := range taken {
		if math.Abs(w.WidthM-0.6) > 1e-6 {
			t.Errorf("reading %d = %v, want only the newest 0.6 readings to survive", i, w.WidthM)
		}
	}
}

// The creep drives the chassis a long way down the corridor while the
// navigator is NOT following the path, so its waypoint index is still 0 when
// the direction settles. Resuming from that index means chasing a waypoint
// the robot has already driven past.
func TestDirectionSettleResyncsTheWaypointIndex(t *testing.T) {
	t.Parallel()

	// The direction estimator reads only the two side rays, so this is the
	// same decisive reading its own tests use: right open, left close.
	decisive := controllers.LidarScan{
		RangesM:   []float64{0.3, 3.0},
		AnglesRad: []float64{math.Pi / 2, -math.Pi / 2},
	}

	path := squareLoop()
	// Standing beside the FAR side of the loop, not at waypoint 0 -- which is
	// where a creep leaves the chassis.
	far := path[len(path)/2]

	gateway := &fakeGateway{}
	gateway.pose = trackmodel.Pose{X: far.X, Y: far.Y, Yaw: 0}
	gateway.havePose = true
	gateway.scan = decisive
	gateway.haveScan = true

	nav, err := navigator.New(navigator.Params{
		Gateway:           gateway,
		Waypoints:         path,
		Direction:         nil,
		Config:            navigator.DefaultConfig(),
		ControllersConfig: controllers.DefaultConfig(),
		Logger:            discardLogger(),
	})
	if err != nil {
		t.Fatalf("New() error = %v", err)
	}
	if nav.WaypointIndex() != 0 {
		t.Fatalf("waypoint index started at %d, want 0", nav.WaypointIndex())
	}

	for range 30 {
		nav.Step()
		if nav.Direction() != nil {
			break
		}
	}
	if nav.Direction() == nil {
		t.Fatal("direction never settled on a decisive reading")
	}

	if nav.WaypointIndex() == 0 {
		t.Error("waypoint index still 0 after the direction settled: the path " +
			"was not resynced to where the chassis actually is")
	}
}
