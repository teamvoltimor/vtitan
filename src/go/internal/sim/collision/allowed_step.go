package collision

import (
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/kinematics"
)

// minStepScale is the smallest usable fraction of a commanded step,
// matching collision_stepping.py's _MIN_STEP_SCALE. Below this the move is
// submillimetre and the chassis is, for scoring purposes, against the
// surface rather than sliding along it.
const minStepScale = 1e-3

// stepBisections is the number of bisections used to find the largest
// fitting fraction of a step, matching collision_stepping.py's
// _STEP_BISECTIONS. Eight halvings resolve a 7.5mm tick to ~0.03mm, well
// under the 30mm LIDAR noise the navigator steers on, so more would be
// measuring nothing.
const stepBisections = 8

// AllowedStep returns the furthest along the commanded step the chassis
// may actually go, matching collision_stepping.py's allowed_step.
//
// It returns candidate itself when the whole step is clear, a scaled pose
// when a solid surface cuts it short, or nil when no part of it fits and
// the body cannot move at all.
//
// Rotation and translation are limited SEPARATELY, and that separation is
// the whole point. A wall bounds how far the chassis may TURN, not whether
// it may advance: a real car against a wall keeps driving with its corner
// scraping while the steering gradually pulls it clear. Scaling both
// together instead leaves the chassis stuck at its maximum yaw forever,
// because from there every step asks for more rotation -- each tick the
// turn needs about 2mm more clearance than the same tick's forward motion
// earns, so no fraction of it ever fits.
//
// So: keep the full translation and take whatever fraction of the turn
// fits alongside it. Advancing at the limiting angle earns a fraction of a
// millimeter of clearance per tick, which lets a little more of the turn
// through on the next one, and the chassis peels away. Only if the
// translation itself is blocked -- driving squarely into a wall -- is it
// cut back, and then to nothing, so head-on contact still makes no
// progress and reversing out is still a real escape.
//
// length/width are the chassis footprint dimensions to test with -- unlike
// Python's TrackModel.contact_surface, which defaults these to
// RobotSpecs.LENGTH/WIDTH, Go has no default arguments, so callers pass
// them explicitly (e.g. from profile.RobotConfig).
func AllowedStep(
	track *TrackModel,
	solidSurfaces SurfaceSet,
	length, width float64,
	state, candidate kinematics.AckermannState,
) *kinematics.AckermannState {
	if !solidSurfaces.Contains(track.ContactSurfaceAt(candidate.X, candidate.Y, candidate.Yaw, length, width)) {
		return &candidate
	}

	dx, dy := candidate.X-state.X, candidate.Y-state.Y
	dyaw := navutil.WrapAngle(candidate.Yaw - state.Yaw)

	free := func(move, turn float64) bool {
		surface := track.ContactSurfaceAt(state.X+dx*move, state.Y+dy*move, state.Yaw+dyaw*turn, length, width)
		return !solidSurfaces.Contains(surface)
	}

	largest := func(fits func(float64) bool) float64 {
		if !fits(minStepScale) {
			return 0.0
		}
		lo, hi := minStepScale, 1.0
		for range stepBisections {
			mid := (lo + hi) / 2
			if fits(mid) {
				lo = mid
			} else {
				hi = mid
			}
		}
		return lo
	}

	// Full translation, as much of the turn as fits alongside it.
	if free(1.0, 0.0) {
		turn := largest(func(t float64) bool { return free(1.0, t) })
		result := candidate
		result.Yaw = state.Yaw + dyaw*turn
		return &result
	}

	// The translation itself is blocked: hold the heading and advance as
	// far as fits, which is nothing when driving squarely into a wall.
	move := largest(func(mv float64) bool { return free(mv, 0.0) })
	if move == 0.0 {
		return nil
	}
	result := candidate
	result.X = state.X + dx*move
	result.Y = state.Y + dy*move
	result.Yaw = state.Yaw
	result.V = candidate.V * move
	return &result
}
