package navigator_test

import (
	"errors"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navigator"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// TestNew_RejectsMissingGateway covers New's ErrNoGateway guard: every Step
// branch both reads and publishes through the gateway, so there is no
// degraded mode to construct.
func TestNew_RejectsMissingGateway(t *testing.T) {
	t.Parallel()

	_, err := navigator.New(navigator.Params{
		Waypoints: squareLoop(),
		Config:    navigator.DefaultConfig(),
	})
	if !errors.Is(err, navigator.ErrNoGateway) {
		t.Fatalf("New() error = %v, want ErrNoGateway", err)
	}
}

// TestNew_RejectsEmptyPath covers ErrNoWaypoints. Python tolerates an empty
// path only because _apply_path_wall_budget early-returns; here every later
// index into the list would panic, so it is rejected at construction.
func TestNew_RejectsEmptyPath(t *testing.T) {
	t.Parallel()

	_, err := navigator.New(navigator.Params{
		Gateway: &fakeGateway{},
		Config:  navigator.DefaultConfig(),
	})
	if !errors.Is(err, navigator.ErrNoWaypoints) {
		t.Fatalf("New() error = %v, want ErrNoWaypoints", err)
	}
}

// TestNew_DefaultsNumLaps checks that NumLaps=0 falls back to
// DefaultOpenChallengeLaps rather than reading as "already finished" on the
// first tick, which is what a literal zero would mean to _is_holding.
func TestNew_DefaultsNumLaps(t *testing.T) {
	t.Parallel()

	nav, gateway := newNavigator(t)
	gateway.setPose(1.5, 1.0, 0.0)
	nav.Step()

	if got := nav.DebugSnapshot().NumLaps; got != navigator.DefaultOpenChallengeLaps {
		t.Fatalf("NumLaps = %d, want %d", got, navigator.DefaultOpenChallengeLaps)
	}
}

// TestStep_NoPose_StopsRatherThanCoasting covers step()'s first branch: with
// no localization the robot must be commanded to stop, not left coasting on
// the previously published command.
func TestStep_NoPose_StopsRatherThanCoasting(t *testing.T) {
	t.Parallel()

	nav, gateway := newNavigator(t)
	nav.Step()

	command, ok := gateway.lastDrive()
	if !ok {
		t.Fatal("published no drive command, want an explicit stop")
	}
	if command.SpeedMPS != 0 || command.SteeringNorm != 0 {
		t.Fatalf("command = %+v, want zero speed and steering", command)
	}

	debug := nav.DebugSnapshot()
	if debug.Phase != navigator.PhaseNoPose {
		t.Fatalf("Phase = %v, want PhaseNoPose", debug.Phase)
	}
	// PhaseNoPose reports the command it issued but has no pose to report,
	// so the pose fields stay nil rather than carrying a stale reading.
	if debug.PoseX != nil || debug.PoseY != nil || debug.PoseYaw != nil {
		t.Fatalf(
			"pose fields = (%v, %v, %v), want all nil",
			debug.PoseX,
			debug.PoseY,
			debug.PoseYaw,
		)
	}
}

// TestStep_WaypointReached_AdvancesIndex covers the reached-distance branch:
// standing on a waypoint advances the index by exactly one and reports
// PhaseWaypointReached without driving.
func TestStep_WaypointReached_AdvancesIndex(t *testing.T) {
	t.Parallel()

	nav, gateway := newNavigator(t)
	first := squareLoop()[0]
	gateway.setPose(first.X, first.Y, 0.0)
	nav.Step()

	if got := nav.WaypointIndex(); got != 1 {
		t.Fatalf("WaypointIndex() = %d, want 1", got)
	}
	if got := nav.DebugSnapshot().Phase; got != navigator.PhaseWaypointReached {
		t.Fatalf("Phase = %v, want PhaseWaypointReached", got)
	}
}

// driveOneLap walks the robot over every waypoint in turn, leaving the index
// at len(path) -- one past the end, which is the state handleWaypointWrap
// exists to resolve on the following tick.
func driveOneLap(t *testing.T, nav *navigator.Navigator, gateway *fakeGateway) {
	t.Helper()

	for _, wp := range squareLoop() {
		gateway.setPose(wp.X, wp.Y, 0.0)
		nav.Step()
	}
	if got, want := nav.WaypointIndex(), len(squareLoop()); got != want {
		t.Fatalf("after one lap WaypointIndex() = %d, want %d", got, want)
	}
}

// TestStep_WaypointWrap_CountsLap covers the waypoint-only lap fallback,
// which is the ONLY lap-counting mechanism in this port -- Python prefers
// LapDetector's geometric confirmation and this port has no equivalent.
func TestStep_WaypointWrap_CountsLap(t *testing.T) {
	t.Parallel()

	nav, gateway := newNavigator(t)
	driveOneLap(t, nav, gateway)

	nav.Step()

	if got := nav.LapsCompleted(); got != 1 {
		t.Fatalf("LapsCompleted() = %d, want 1", got)
	}
	if got := nav.WaypointIndex(); got != 0 {
		t.Fatalf("WaypointIndex() = %d, want 0 after wrap", got)
	}
	if got := nav.DebugSnapshot().Phase; got != navigator.PhaseWaypointWrapFallback {
		t.Fatalf("Phase = %v, want PhaseWaypointWrapFallback", got)
	}
}

// TestStep_FinishedHold covers _handle_finish's `pc is None` branch: with no
// ParkController the Open Challenge simply holds position once the final lap
// lands, and keeps holding on every later tick.
func TestStep_FinishedHold(t *testing.T) {
	t.Parallel()

	nav, gateway := newNavigator(t, func(p *navigator.Params) { p.NumLaps = 1 })
	driveOneLap(t, nav, gateway)
	nav.Step() // wrap: completes lap 1 of 1

	if got := nav.LapsCompleted(); got != 1 {
		t.Fatalf("LapsCompleted() = %d, want 1", got)
	}

	nav.Step()

	command, ok := gateway.lastDrive()
	if !ok {
		t.Fatal("published no drive command, want an explicit hold")
	}
	if command.SpeedMPS != 0 || command.SteeringNorm != 0 {
		t.Fatalf("command = %+v, want zero speed and steering", command)
	}
	if got := nav.DebugSnapshot().Phase; got != navigator.PhaseFinishedHold {
		t.Fatalf("Phase = %v, want PhaseFinishedHold", got)
	}
}

// TestReset_ClearsRaceState covers reset(): the state machine can cycle
// FINISHED -> BOOT_CHECK -> READY -> RACING from the button alone, with no
// process restart, so a stale lap count would make the next race's first
// tick read as already finished.
func TestReset_ClearsRaceState(t *testing.T) {
	t.Parallel()

	nav, gateway := newNavigator(t)
	driveOneLap(t, nav, gateway)
	nav.Step()

	if nav.LapsCompleted() == 0 {
		t.Fatal("precondition failed: expected a completed lap before Reset")
	}

	nav.Reset()

	if got := nav.LapsCompleted(); got != 0 {
		t.Fatalf("LapsCompleted() = %d, want 0 after Reset", got)
	}
	if got := nav.WaypointIndex(); got != 0 {
		t.Fatalf("WaypointIndex() = %d, want 0 after Reset", got)
	}
}

// TestReplacePath_ReseeksNearestWaypoint covers replace_path's re-seek: the
// old index means nothing on a new path, so progress is preserved by
// position rather than restarted at zero.
func TestReplacePath_ReseeksNearestWaypoint(t *testing.T) {
	t.Parallel()

	nav, _ := newNavigator(t)
	path := squareLoop()

	nav.ReplacePath(path, trackmodel.Waypoint{X: 2.0, Y: 1.9}, nil)

	if got := nav.WaypointIndex(); got != 2 {
		t.Fatalf("WaypointIndex() = %d, want 2 (nearest to (2.0, 1.9))", got)
	}
	if got := nav.Waypoints(); len(got) != len(path) {
		t.Fatalf("Waypoints() length = %d, want %d", len(got), len(path))
	}
}

// TestReplacePath_HeadingBreaksNearTie covers the heading re-rank: near a
// corner several waypoints sit at almost the same distance while pointing in
// very different directions, and picking purely by position can hand back a
// point past the turn -- measured on hardware as a ~193 deg demanded swing
// where ~90 deg would do.
func TestReplacePath_HeadingBreaksNearTie(t *testing.T) {
	t.Parallel()

	// Two candidates equidistant from the robot, with opposite outgoing
	// bearings: index 0 points +x, index 2 points -x.
	path := []trackmodel.Waypoint{
		{X: 1.0, Y: 1.0},
		{X: 1.2, Y: 1.0},
		{X: 1.2, Y: 1.2},
		{X: 1.0, Y: 1.2},
	}
	robot := trackmodel.Waypoint{X: 1.1, Y: 1.1}

	facingPlusX := 0.0
	navPlusX, _ := newNavigator(t, func(p *navigator.Params) { p.Waypoints = path })
	navPlusX.ReplacePath(path, robot, &facingPlusX)

	facingMinusX := math.Pi
	navMinusX, _ := newNavigator(t, func(p *navigator.Params) { p.Waypoints = path })
	navMinusX.ReplacePath(path, robot, &facingMinusX)

	if navPlusX.WaypointIndex() == navMinusX.WaypointIndex() {
		t.Fatalf("heading did not break the tie: both chose index %d", navPlusX.WaypointIndex())
	}
	if got := navPlusX.WaypointIndex(); got != 0 {
		t.Fatalf("facing +x chose index %d, want 0", got)
	}
	if got := navMinusX.WaypointIndex(); got != 2 {
		t.Fatalf("facing -x chose index %d, want 2", got)
	}
}

// TestReplacePath_SuppressesWrapAcrossSeam covers the suppress_next_wrap
// guard: a large forward jump is never earned progress, it means the re-seek
// landed on the far side of the start/finish seam, so the wrap at the end of
// the tail is the ENTRY into a lap rather than its completion.
func TestReplacePath_SuppressesWrapAcrossSeam(t *testing.T) {
	t.Parallel()

	nav, gateway := newNavigator(t)
	path := squareLoop()
	last := path[len(path)-1]

	// Index 0 -> 3 on a 4-point loop is a jump of 3, past len/2.
	nav.ReplacePath(path, last, nil)
	if got := nav.WaypointIndex(); got != len(path)-1 {
		t.Fatalf("WaypointIndex() = %d, want %d", got, len(path)-1)
	}

	// Reach the final waypoint, taking the index one past the end.
	gateway.setPose(last.X, last.Y, 0.0)
	nav.Step()
	if got := nav.WaypointIndex(); got != len(path) {
		t.Fatalf("WaypointIndex() = %d, want %d", got, len(path))
	}

	nav.Step()

	if got := nav.LapsCompleted(); got != 0 {
		t.Fatalf("LapsCompleted() = %d, want 0 -- the seeded wrap must not count a lap", got)
	}
	if got := nav.WaypointIndex(); got != 0 {
		t.Fatalf("WaypointIndex() = %d, want 0 after the suppressed wrap", got)
	}
}

// TestSetTravelDirection_ReachesDebug covers set_travel_direction: the
// escape maneuver reads the direction as its fallback side, and the debug
// snapshot is how a bag shows which side that was.
func TestSetTravelDirection_ReachesDebug(t *testing.T) {
	t.Parallel()

	nav, gateway := newNavigator(t)
	nav.SetTravelDirection(trackmodel.Counterclockwise)

	// A non-zero yaw, so this also covers the pose reaching the snapshot
	// intact rather than through a zero that any bug would also produce.
	const yaw = 0.75
	gateway.setPose(1.5, 1.0, yaw)
	nav.Step()

	if got := nav.DebugSnapshot().PoseYaw; got == nil || *got != yaw {
		t.Fatalf("PoseYaw = %v, want %v", got, yaw)
	}

	direction := nav.DebugSnapshot().Direction
	if direction == nil {
		t.Fatal("Direction = nil, want Counterclockwise")
	}
	if *direction != trackmodel.Counterclockwise {
		t.Fatalf("Direction = %v, want Counterclockwise", *direction)
	}
}

// TestStep_NormalDrive_PublishesMotion is the baseline that the ordinary
// driving path runs end to end: away from any waypoint and with a clear
// scan, the navigator commands forward motion rather than falling into one
// of the stop branches.
func TestStep_NormalDrive_PublishesMotion(t *testing.T) {
	t.Parallel()

	nav, gateway := newNavigator(t)
	gateway.setPose(1.5, 1.0, 0.0)
	gateway.scan = clearScan()
	gateway.haveScan = true

	nav.Step()

	command, ok := gateway.lastDrive()
	if !ok {
		t.Fatal("published no drive command")
	}
	if command.SpeedMPS <= 0 {
		t.Fatalf("SpeedMPS = %v, want > 0 on a clear path", command.SpeedMPS)
	}
	if got := nav.DebugSnapshot().Phase; got != navigator.PhaseNormalDrive {
		t.Fatalf("Phase = %v, want PhaseNormalDrive", got)
	}
}

// clearScan is a full sweep with nothing in range, so collision avoidance
// and the escape maneuver both stay out of the way of whatever the test is
// actually asserting.
func clearScan() controllers.LidarScan {
	const numRays = 360
	ranges := make([]float64, numRays)
	angles := navutil.AngleFan(numRays)
	for i := range ranges {
		ranges[i] = 3.0
	}
	return controllers.LidarScan{RangesM: ranges, AnglesRad: angles}
}
