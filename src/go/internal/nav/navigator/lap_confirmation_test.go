package navigator

import (
	"log/slog"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/racetracker"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// nopGateway is a do-nothing controllers.HardwareGateway, enough to build a
// Navigator for the lap-confirmation test.
type nopGateway struct{}

var _ controllers.HardwareGateway = nopGateway{}

func (nopGateway) PublishDrive(controllers.DriveCommand)       {}
func (nopGateway) GetCurrentPose() (trackmodel.Pose, bool)     { return trackmodel.Pose{}, false }
func (nopGateway) GetLidarScan() (controllers.LidarScan, bool) { return controllers.LidarScan{}, false }
func (nopGateway) GetWheelOdometry() (controllers.WheelOdometry, bool) {
	return controllers.WheelOdometry{}, false
}
func (nopGateway) SetBelievedWalls(*trackmodel.TrackWalls)  {}
func (nopGateway) ResetPosition(float64, float64)           {}
func (nopGateway) ResetHeadingReference()                   {}
func (nopGateway) CorrectHeadingForDirectionChange(float64) {}

// With a LapDetector, a waypoint wrap must only ARM the geometric
// confirmation; the lap lands when the robot actually crosses the line, not
// when the index runs off the end. Without one the wrap counts directly (the
// fallback the external tests cover).
func TestHandleWaypointWrap_ArmsDetectorInsteadOfCounting(t *testing.T) {
	t.Parallel()

	nav, err := New(Params{
		Gateway:           nopGateway{},
		Waypoints:         []trackmodel.Waypoint{{X: 1, Y: 1}},
		Config:            DefaultConfig(),
		ControllersConfig: controllers.DefaultConfig(),
		Logger:            slog.New(slog.DiscardHandler),
	})
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	det, err := racetracker.NewLapDetector(
		trackmodel.Waypoint{X: 1, Y: 1}, trackmodel.South, trackmodel.Clockwise,
	)
	if err != nil {
		t.Fatalf("NewLapDetector: %v", err)
	}
	nav.SetLapDetector(det)
	nav.waypointIndex = len(nav.waypoints) // force the wrap branch

	if finished := nav.handleWaypointWrap(trackmodel.Pose{}); finished {
		t.Fatal("handleWaypointWrap returned finished with a detector, want the tick to continue")
	}
	if nav.lapsCompleted != 0 {
		t.Fatalf("lapsCompleted = %d, want 0: a wrap only arms the detector", nav.lapsCompleted)
	}

	// SOUTH + Clockwise travels toward -x, so the finish line is at x=1 and
	// "short of it" is x>1.
	south := trackmodel.South
	nav.currentCorridor = &south
	nav.confirmGeometricLap(trackmodel.Pose{X: 1.5, Y: 1}) // short of the line: arms prevDot
	nav.confirmGeometricLap(trackmodel.Pose{X: 0.5, Y: 1}) // forward crossing

	if nav.lapsCompleted != 1 {
		t.Fatalf("lapsCompleted = %d, want 1 after the confirmed crossing", nav.lapsCompleted)
	}
}
