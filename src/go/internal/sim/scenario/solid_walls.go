package scenario

import (
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/collision"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/harness"
)

// simPhysics is how the world held up this step: whether a solid surface
// cut it short, the surface it scores, and the first physics invariant the
// run broke (see harness.SimHardwareGateway).
type simPhysics interface {
	Blocked() bool
	ContactSurface() collision.ContactSurface
	PhysicsViolation() string
}

// forbiddenSurfaces is the challenge's terminal set: ObstaclesForbiddenSurfaces
// for an Obstacles Challenge run, OpenForbiddenSurfaces otherwise.
func forbiddenSurfaces(obstacles bool) collision.SurfaceSet {
	if obstacles {
		return ObstaclesForbiddenSurfaces
	}
	return OpenForbiddenSurfaces
}

// gatewayConfig is the harness Config for one scenario: the solid
// surfaces depend on the challenge, since a surface whose contact ends the
// run is left passable (collision.SolidSurfacesFor).
func (r *NativeRunner) gatewayConfig(obstacles bool) harness.Config {
	cfg := r.cfg
	cfg.SolidSurfaces = collision.SolidSurfacesFor(forbiddenSurfaces(obstacles))
	cfg.SlideOnContact = r.collCfg.SlideOnContact
	cfg.NoContactResponse = r.noSolidWalls
	return cfg
}

// voided scores a run the simulator broke physics in as invalid: never a
// success, with the violation named, since whatever it would have scored
// says nothing about the robot (platform plan principle 8).
func voided(
	gw simPhysics,
	score func(collision.ContactSurface, bool, []int) Result,
	passSide *passSideScorer,
) (Result, bool) {
	v := gw.PhysicsViolation()
	if v == "" {
		return Result{}, false
	}
	res := score(collision.SurfaceNone, false, passSide.violations())
	res.InvalidSim = v
	res.Success = false
	return res, true
}
