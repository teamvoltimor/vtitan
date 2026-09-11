package navigator

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"
)

// cornerCompletionFraction is the fraction of the previewed heading change
// that counts as "turn driven".
//
// Not 1.0: trackmodel.PathTurnAhead measures the path's own heading change
// over the preview window, while the release test measures the CHASSIS
// heading change, and the two do not have to agree exactly -- the robot may
// cut the corner slightly, or the preview may still have been growing when
// the latch armed. Requiring the full figure risks a latch that never
// releases on a corner the robot rounds slightly tight. Requiring most of it
// releases within a few ticks of the real exit.
const cornerCompletionFraction = 0.8

// maxLatchYawRad is a backstop: release after a full revolution of
// accumulated heading change.
//
// The release test is monotone in |yaw - yawAtArm| only until that wraps. A
// robot spinning on the spot (a failed escape, say) would otherwise
// re-satisfy the test periodically rather than never, but could also hold
// the latch for a long time in between. This bounds the damage to one
// revolution regardless.
const maxLatchYawRad = 2.0 * math.Pi

// CornerLatch is a sticky version of the corner-turn preview.
//
// trackmodel.PathTurnAhead is a PREVIEW: it measures the heading change the
// path makes within CornerPreviewDistanceM ahead of the current waypoint.
// That makes it a leading signal on approach, which is exactly what
// WaypointController.SelectLookahead wants -- and it means the signal DECAYS
// to zero as the chassis enters the arc, because the preview window slides
// past the corner the chassis is now inside.
//
// So the signal is smallest precisely when the corner is being driven, and
// thresholding it un-arms the short lookahead mid-turn. Measured on hardware
// 2026-08-30 (run_20260830_014612, first corner, west -> south at 6.1 s),
// with CornerTurnThresholdRad at 0.35:
//
//	rel_t   turn    look    aerr     xtrack
//	-3.99   1.373   0.160   +0.573   0.179   preview armed, short lookahead
//	-2.40   0.590   0.160   -0.612   0.076
//	-2.00   0.197   0.320   -1.215   0.011   <- un-armed 2 s BEFORE the corner
//	-0.60   0.000   0.320   -1.233   0.027
//	+1.21   0.000   0.320   -1.053   0.119   <- crosstrack diverging
//
// The other two demands cannot cover the gap. Crosstrack error is measured
// against the planned path and stays at 0.002-0.027 m right through the
// corner -- the chassis is ON the path, it is POINTING 70 degrees off it --
// and signAhead is an Obstacles-only boolean.
//
// Latching on ELAPSED TIME or on a waypoint count would both be proxies. The
// honest release condition is the one the preview itself stated: it said the
// path turns by previewed radians, so hold until the chassis has actually
// turned that much. Nothing else needs to be known about the corner, and a
// turn that is tighter or slower than planned holds the lookahead for
// exactly as long as it really takes rather than for a guessed budget.
//
// Ported from Python's src/navigation/core_navigator/corner_latch.py.
//
// Feed it the raw PathTurnAhead reading and the current chassis yaw each
// tick; it returns the value the lookahead selector should act on. Callers
// get the raw reading back unchanged whenever no corner is being driven, so
// a straight behaves exactly as before.
//
// The zero value is ready to use, and uses cornerCompletionFraction. Use
// NewCornerLatch to override that for a sweep.
type CornerLatch struct {
	// completionFraction is zero in the zero value, which
	// effectiveCompletionFraction reads as "use the default".
	completionFraction float64

	previewedRad   float64
	yawAtArm       *float64
	accumulatedRad float64
	lastYaw        *float64
}

// NewCornerLatch returns a latch releasing at completionFraction of the
// previewed turn.
//
// This exists for sweeps: the value trades corner clearance against how much
// of the lap is spent on the short lookahead, and that trade has to be
// measured rather than argued. Pass 0 for cornerCompletionFraction.
func NewCornerLatch(completionFraction float64) *CornerLatch {
	return &CornerLatch{completionFraction: completionFraction}
}

func (c *CornerLatch) effectiveCompletionFraction() float64 {
	if c.completionFraction == 0.0 {
		return cornerCompletionFraction
	}
	return c.completionFraction
}

// IsLatched reports whether a previewed corner is still being driven.
func (c *CornerLatch) IsLatched() bool { return c.yawAtArm != nil }

// Reset forgets any corner in progress.
//
// For a new round or a replanned path, where the previewed turn the latch is
// holding open may no longer be on the route at all.
func (c *CornerLatch) Reset() {
	c.previewedRad = 0.0
	c.yawAtArm = nil
	c.accumulatedRad = 0.0
	c.lastYaw = nil
}

// Update returns the turn-ahead value to act on this tick: turnAheadRad, or
// the previewed value being held if a corner armed earlier has not yet been
// driven.
//
// armThresholdRad is CornerTurnThresholdRad -- the same value
// SelectLookahead compares against, read from tuning by the caller rather
// than duplicated here.
func (c *CornerLatch) Update(turnAheadRad, robotYaw, armThresholdRad float64) float64 {
	if c.lastYaw != nil {
		c.accumulatedRad += math.Abs(navutil.WrapAngle(robotYaw - *c.lastYaw))
	}
	c.lastYaw = &robotYaw

	// Re-arming while already latched refreshes the target rather than
	// restarting it: a corner whose preview is still growing should hold to
	// the LARGEST turn it ever promised, not the last one seen before the
	// signal decayed.
	if turnAheadRad >= armThresholdRad {
		if c.yawAtArm == nil {
			armYaw := robotYaw
			c.yawAtArm = &armYaw
			c.accumulatedRad = 0.0
		}
		c.previewedRad = math.Max(c.previewedRad, turnAheadRad)
		return turnAheadRad
	}

	if c.yawAtArm == nil {
		return turnAheadRad
	}

	turned := math.Abs(navutil.WrapAngle(robotYaw - *c.yawAtArm))
	if turned >= c.effectiveCompletionFraction()*c.previewedRad ||
		c.accumulatedRad >= maxLatchYawRad {
		held := c.previewedRad
		c.Reset()
		// The tick that completes the turn still reports the held value:
		// releasing to a decayed reading on the same tick would drop the
		// lookahead back out mid-exit, which is the behaviour this exists
		// to prevent.
		return math.Max(turnAheadRad, held)
	}

	return math.Max(turnAheadRad, c.previewedRad)
}
