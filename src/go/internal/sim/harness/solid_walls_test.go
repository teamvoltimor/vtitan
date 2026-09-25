package harness_test

import (
	"math"
	"strings"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/collision"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/harness"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/kinematics"
)

// obstaclesSolid is what the native runner makes solid in an Obstacles
// round: the outer wall and the parking fins.
var obstaclesSolid = collision.SolidSurfacesFor(collision.NewSurfaceSet(
	collision.SurfaceInnerWall, collision.SurfaceObstacle, collision.SurfaceParkingLot))

// wallRun drives straight at the south outer wall for two seconds from 5 cm
// off it and returns the gateway.
func wallRun(t *testing.T, noResponse bool) *harness.SimHardwareGateway {
	t.Helper()

	cfg := harness.DefaultConfig()
	cfg.SolidSurfaces = obstaclesSolid
	cfg.SlideOnContact = true
	cfg.NoContactResponse = noResponse
	widths := map[trackmodel.Section]float64{
		trackmodel.North: 1.0, trackmodel.South: 1.0, trackmodel.East: 1.0, trackmodel.West: 1.0,
	}
	track := collision.NewTrackModel(collision.NewTrackModelParams{
		Geometry:  trackmodel.CorridorGeometryFromWidths(widths, 3.0),
		MinCoordM: 0.0,
		MaxCoordM: 3.0,
	})
	start := kinematics.AckermannState{X: 1.5, Y: cfg.ChassisLengthM/2 + 0.05, Yaw: -math.Pi / 2}
	gw := harness.NewSimHardwareGateway(
		cfg, track, start, kinematics.NewAckermannKinematics(kinematics.DefaultParams()), 1,
	)
	gw.PublishDrive(controllers.DriveCommand{SpeedMPS: 0.4})
	for range 40 {
		gw.Advance(0.05)
	}
	return gw
}

// A solid wall stops the body at its face: the step into it is refused,
// the tick scores the wall the refused step would have entered, the wheels
// report no travel past the 5 cm available, and no invariant breaks.
func TestSolidWalls_StopTheBodyAtTheFace(t *testing.T) {
	t.Parallel()

	gw := wallRun(t, false)
	st := gw.State()
	if nose := st.Y - harness.DefaultConfig().ChassisLengthM/2; nose < 0 || nose > 0.001 {
		t.Errorf("nose %.4f m from the wall, want resting against it", nose)
	}
	if !gw.Blocked() || gw.ContactSurface() != collision.SurfaceOuterWall {
		t.Errorf("blocked %v, contact %v, want blocked against the outer wall", gw.Blocked(), gw.ContactSurface())
	}
	if odo, _ := gw.GetWheelOdometry(); odo.DistanceM > 0.051 || odo.SpeedMPS != 0 {
		t.Errorf("odometry %.4f m at %.2f m/s, want at most the 5 cm to the wall, stopped", odo.DistanceM, odo.SpeedMPS)
	}
	if v := gw.PhysicsViolation(); v != "" {
		t.Errorf("PhysicsViolation() = %q, want none", v)
	}
}

// Without contact response the same drive goes through the wall, and the
// penetration invariant catches it: this is what voids the old model's
// runs.
func TestSolidWalls_PassingThroughBreaksTheInvariant(t *testing.T) {
	t.Parallel()

	gw := wallRun(t, true)
	if gw.State().Y >= 0 {
		t.Fatalf("body at y=%.3f, want it driven through the wall at y=0", gw.State().Y)
	}
	if v := gw.PhysicsViolation(); !strings.HasPrefix(v, "penetration") || !strings.Contains(v, "outer_wall") {
		t.Errorf("PhysicsViolation() = %q, want a penetration of the outer wall", v)
	}
}
