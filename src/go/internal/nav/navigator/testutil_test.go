package navigator_test

import (
	"log/slog"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navigator"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// fakeGateway is an in-memory controllers.HardwareGateway, standing in for
// the ROS2/simulation adapters the Python oracle suite fakes the same way:
// every reader returns whatever the test staged, and every writer records
// what the navigator asked for.
//
// The three ok=false-capable readers default to unavailable, so a test opts
// in to pose/scan/odometry rather than silently inheriting a usable one --
// "no pose yet" is a real Step branch (PhaseNoPose), not an edge case.
type fakeGateway struct {
	pose     trackmodel.Pose
	havePose bool

	scan     controllers.LidarScan
	haveScan bool

	odometry     controllers.WheelOdometry
	haveOdometry bool

	published          []controllers.DriveCommand
	believedWalls      *trackmodel.TrackWalls
	resetPositions     []trackmodel.Waypoint
	headingResets      int
	headingCorrections []float64
}

func (g *fakeGateway) PublishDrive(command controllers.DriveCommand) {
	g.published = append(g.published, command)
}

func (g *fakeGateway) GetCurrentPose() (trackmodel.Pose, bool) {
	return g.pose, g.havePose
}

func (g *fakeGateway) GetLidarScan() (controllers.LidarScan, bool) {
	return g.scan, g.haveScan
}

func (g *fakeGateway) GetWheelOdometry() (controllers.WheelOdometry, bool) {
	return g.odometry, g.haveOdometry
}

func (g *fakeGateway) SetBelievedWalls(walls *trackmodel.TrackWalls) {
	g.believedWalls = walls
}

func (g *fakeGateway) ResetPosition(x, y float64) {
	g.resetPositions = append(g.resetPositions, trackmodel.Waypoint{X: x, Y: y})
}

func (g *fakeGateway) ResetHeadingReference() {
	g.headingResets++
}

func (g *fakeGateway) CorrectHeadingForDirectionChange(deltaRad float64) {
	g.headingCorrections = append(g.headingCorrections, deltaRad)
}

// setPose stages the pose Step will read on its next tick.
func (g *fakeGateway) setPose(x, y, yaw float64) {
	g.pose = trackmodel.Pose{X: x, Y: y, Yaw: yaw}
	g.havePose = true
}

// lastDrive is the most recent command the navigator published, and
// ok=false when it published nothing at all -- a distinction several Step
// branches turn on, so it must not collapse into a zero DriveCommand.
func (g *fakeGateway) lastDrive() (controllers.DriveCommand, bool) {
	if len(g.published) == 0 {
		return controllers.DriveCommand{}, false
	}
	return g.published[len(g.published)-1], true
}

// squareLoop is a four-waypoint closed lap, one waypoint per side, sitting
// well inside the 3.0 m mat so applyPathWallBudget's outer-wall clearance is
// the same on every point and no test accidentally depends on which corner
// is nearest an edge.
func squareLoop() []trackmodel.Waypoint {
	return []trackmodel.Waypoint{
		{X: 1.0, Y: 1.0},
		{X: 2.0, Y: 1.0},
		{X: 2.0, Y: 2.0},
		{X: 1.0, Y: 2.0},
	}
}

// discardLogger keeps the lap/escape messages CoreNavigator logs out of the
// test output; the tests assert on state, never on log lines.
func discardLogger() *slog.Logger {
	return slog.New(slog.DiscardHandler)
}

// newNavigator builds a Navigator over a fresh fakeGateway with the shipped
// defaults, mirroring the Python oracle's `navigator` fixture
// (CoreNavigator built from NavigationTuning.load_default()). Tests mutate
// the returned gateway to stage each tick.
//
// Deliberately reads DefaultConfig()/controllers.DefaultConfig() rather than
// restating literals, so a shipped-default change surfaces here as a test
// failure instead of being masked by a duplicated constant.
func newNavigator(
	t *testing.T,
	mutate ...func(*navigator.Params),
) (*navigator.Navigator, *fakeGateway) {
	t.Helper()

	gateway := &fakeGateway{}
	params := navigator.Params{
		Gateway:           gateway,
		Waypoints:         squareLoop(),
		Direction:         clockwiseDir(),
		Config:            navigator.DefaultConfig(),
		ControllersConfig: controllers.DefaultConfig(),
		Logger:            discardLogger(),
	}
	for _, m := range mutate {
		m(&params)
	}

	nav, err := navigator.New(params)
	if err != nil {
		t.Fatalf("New() error = %v, want nil", err)
	}
	return nav, gateway
}

// clockwiseDir returns a pointer to trackmodel.Clockwise, the travel direction
// Params.Direction now requires (Optional: a *trackmodel.Direction).
func clockwiseDir() *trackmodel.Direction {
	d := trackmodel.Clockwise
	return &d
}
