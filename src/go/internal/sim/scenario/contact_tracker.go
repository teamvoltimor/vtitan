package scenario

import "github.com/teamvoltimor/vtitan/src/go/internal/sim/collision"

// unforgivableContactSurfaces are contacts no grace period may forgive,
// matching Python's scenario_result._UNFORGIVABLE_SURFACES.
//
// The grace contactTracker applies below models a chassis working itself
// free of a WALL, which 9.18 explicitly permits ("if the vehicle touches or
// bumps the walls, and the walls are not moved, the vehicle may continue the
// round"). The parking lot has no such concession: 9.24.7 ends the round on
// contact, full stop, so a surface that is fatal by rule cannot be waited
// out no matter when the streak started or how short it is.
var unforgivableContactSurfaces = collision.NewSurfaceSet(collision.SurfaceParkingLot)

// OpenForbiddenSurfaces/ObstaclesForbiddenSurfaces mirror Python's
// TERMINAL_SURFACES (scenario_result.py): each challenge forbids exactly one
// wall -- the Open Challenge the OUTER one, the Obstacles Challenge the INNER
// one. Contact with the other wall is still recorded in the tracker's count
// but never ends the run, so a scrape the robot drives out of does not score
// the same as failing to complete.
//
// INNER_WALL is stricter than the rules and is knowingly left that way (see
// Python's docstring): 9.18 permits touching a wall that is not moved and
// names only the Open Challenge's outer boundary as untouchable. Relaxing it
// for Obstacles would re-base every Obstacles figure in the repo at once, so
// it stays a deliberate per-challenge decision, not an oversight.
var (
	OpenForbiddenSurfaces      = collision.NewSurfaceSet(collision.SurfaceOuterWall)
	ObstaclesForbiddenSurfaces = collision.NewSurfaceSet(
		collision.SurfaceInnerWall, collision.SurfaceObstacle, collision.SurfaceParkingLot)
)

// contactTracker decides when a contact streak stops being survivable and
// ends the run, porting Python's ContactTracker (scenario_result.py) into
// the native runner.
//
// Only a surface in forbidden can end the run or accrue timeS at all --
// touching a wall this challenge permits is recorded in count, not punished.
// Among forbidden surfaces, contact is terminal on the first tick, EXCEPT
// within the opening startWindowS, where a legal starting position the track
// generator allows may already sit a few mm from a wall: such a streak is
// forgiven for up to startGraceS before it counts as a real crash.
// unforgivableContactSurfaces are never forgiven, at any point in the run.
type contactTracker struct {
	dt           float64
	startWindowS float64
	startGraceS  float64
	forbidden    collision.SurfaceSet

	haveStreak  bool
	streakStart int
	inContact   bool

	// count mirrors ContactTracker.count: every touching->not-touching
	// transition of ANY surface (forbidden or not), not just the ones that
	// can end the run.
	count int
	// timeS mirrors ContactTracker.time_s: accumulated only while touching a
	// surface this tracker was asked to consider forbidden.
	timeS float64
	// surface is the surface that ended the run, once update has returned
	// true.
	surface collision.ContactSurface
}

// newContactTracker builds a tracker over the given control interval,
// start-of-run forgiveness window/grace (seconds), and the set of surfaces
// this challenge forbids (see OpenForbiddenSurfaces/ObstaclesForbiddenSurfaces).
func newContactTracker(dt, startWindowS, startGraceS float64, forbidden collision.SurfaceSet) *contactTracker {
	return &contactTracker{dt: dt, startWindowS: startWindowS, startGraceS: startGraceS, forbidden: forbidden}
}

// update folds in one tick's contact surface; returns true if the run should
// end. Every non-none surface counts toward count, but only a streak that
// survives the start-of-run grace (or an unforgivable surface, immediately)
// ends the run or accrues timeS -- touching a wall this challenge permits is
// recorded, not punished, matching ContactTracker.update exactly.
func (c *contactTracker) update(step int, surface collision.ContactSurface) bool {
	touching := surface != collision.SurfaceNone
	if touching && !c.inContact {
		c.count++
	}
	c.inContact = touching

	if !touching || !c.forbidden.Contains(surface) {
		c.haveStreak = false
		return false
	}

	if !c.haveStreak {
		c.streakStart = step
		c.haveStreak = true
	}
	c.timeS += c.dt
	streakS := float64(step-c.streakStart) * c.dt

	if unforgivableContactSurfaces.Contains(surface) {
		// No grace of any kind, matching the Python comment: both graces
		// used to apply here and between them hid the bay-exit maneuver
		// entirely (the start window comfortably covers a whole bay exit).
		c.surface = surface
		return true
	}

	beganAtStart := float64(c.streakStart-1)*c.dt <= c.startWindowS
	ended := !beganAtStart || streakS >= c.startGraceS
	if ended {
		c.surface = surface
	}
	return ended
}
