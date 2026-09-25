package collision

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/sim/kinematics"
	"github.com/teamvoltimor/vtitan/src/go/pkg/geom"
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
// slide changes only the blocked-translation branch. Without it the move is
// scaled along the vector it already had, so a chassis leaning into a
// surface loses the along-surface component too and barely advances; at a
// shallow angle the loss is more than an order of magnitude (see
// adr:0086-simulator-realism). With it, the translation is decomposed and
// the surviving component kept (slideAlong), which is what contact with
// friction-free sliding actually does.
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
	slide bool,
) *kinematics.AckermannState {
	if !solidSurfaces.Contains(track.ContactSurfaceAt(candidate.X, candidate.Y, candidate.Yaw, length, width)) {
		return &candidate
	}

	dx, dy := candidate.X-state.X, candidate.Y-state.Y
	dyaw := geom.WrapAngle(candidate.Yaw - state.Yaw)

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
	if slide {
		freeXY := func(mx, my float64) bool {
			surface := track.ContactSurfaceAt(state.X+mx, state.Y+my, state.Yaw, length, width)
			return !solidSurfaces.Contains(surface)
		}
		if slid := slideAlong(freeXY, largest, state, candidate, math.Hypot(dx*move, dy*move)); slid != nil {
			return slid
		}
	}
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

// slideAlong is the blocked step with its into-surface component dropped,
// or nil, matching collision_stepping.py's _slide_along. Nil also when
// sliding gains nothing over scaledMove, the distance the caller's
// scale-along-the-same-vector fallback would cover, so the caller never has
// to compare the two itself.
//
// It keeps whichever single axis survives contact. Per AXIS rather than
// against a surface normal, because TrackModel reports which surface was
// touched but not its orientation: exact on this mat, where every wall,
// inner-block face, sign and parking fin is an axis-aligned box, and wrong
// on a track with angled walls.
//
// Driving squarely into a wall still yields nothing: the into-surface axis
// is blocked and the along-surface one is zero, so head-on contact makes no
// progress and reversing out remains a real escape.
func slideAlong(
	freeXY func(mx, my float64) bool,
	largest func(fits func(float64) bool) float64,
	state, candidate kinematics.AckermannState,
	scaledMove float64,
) *kinematics.AckermannState {
	dx, dy := candidate.X-state.X, candidate.Y-state.Y
	alongX := dx * largest(func(m float64) bool { return freeXY(dx*m, 0.0) })
	alongY := dy * largest(func(m float64) bool { return freeXY(0.0, dy*m) })
	bestX, bestY := alongX, 0.0
	if math.Abs(alongX) < math.Abs(alongY) {
		bestX, bestY = 0.0, alongY
	}
	commanded := math.Hypot(dx, dy)
	traveled := math.Hypot(bestX, bestY)
	if commanded == 0 || traveled <= scaledMove {
		return nil
	}
	result := candidate
	result.X = state.X + bestX
	result.Y = state.Y + bestY
	result.Yaw = state.Yaw
	result.V = candidate.V * (traveled / commanded)
	return &result
}
