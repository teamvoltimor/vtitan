// Package widthbelief holds a corridor-width change back until the robot has
// left the corridor it describes. It ports
// platform/robot/src/navigation/deferred_width_belief.py.
//
// A width belief update rebuilds the planned path, and the path is what
// crosstrack and the steering target are measured against. When the corridor
// whose width changed is the one the robot is standing in, that rebuild moves
// the line the robot is ACTIVELY TRACKING -- measured on hardware 2026-08-30
// across six runs as a ~0.30 m crosstrack step in a single 50 ms tick, ten
// times what the chassis can physically travel in that time, which threw
// heading error past the crawl threshold and pinned the limiter for 82-100%
// of the ticks that followed.
//
// The same update applied to a corridor the robot is NOT in costs nothing:
// the robot arrives on the new line instead of being displaced onto it. So
// the step is not inherent to replanning, only to replanning UNDERNEATH the
// chassis. Deferring removes it, rather than shrinking it
// (Config.UnconfirmedWidthInnerBiasM) or spreading it over time (blended
// replans, refuted twice -- a path that slides under the robot for a second
// measured worse than one that jumps once and settles).
//
// What deferring costs is small and bounded: the current corridor keeps
// planning on the old belief for the remainder of one traverse, so it is
// centred slightly wrong for that stretch and correct from the next lap
// onward. What it cannot disturb is the corner geometry, because
// CornerArcAssumeWide already sizes every arc as if both corridors were WIDE
// and so no arc depends on the belief deferred here.
//
// The estimator only ever observes the section the robot currently occupies,
// so with deferral ON essentially every change is pending at the moment it is
// discovered and lands one corridor later. That is the intended behaviour,
// not an edge case.
package widthbelief

import (
	"maps"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/waypoints"
)

// Gate is the widths the PATH is planned from, which trail the estimator's
// own. Not a second estimator: it never forms an opinion about a width, it
// only decides WHEN an opinion already formed is allowed to move the path.
//
// Not safe for concurrent use; the navigator owns one and drives it from its
// own control loop.
type Gate struct {
	enabled bool
	applied map[trackmodel.Section]float64
	// appliedUnconfirmed is the confirmed-ness the PLAN currently reflects,
	// which trails the estimator's own for exactly as long as a change is
	// being held back. Stored as the UNCONFIRMED set rather than the observed
	// one so it is the same value the planner takes, with no complement to
	// compute (and so mis-invert) at the boundary.
	appliedUnconfirmed waypoints.UnconfirmedSections
	// seeded distinguishes "nothing planned from us yet" from "planned from
	// an empty belief". A nil/empty applied map cannot: a caller could
	// legitimately pass no widths at all.
	seeded bool
}

// New builds a Gate. enabled=false applies every change immediately, the
// behaviour before this gate existed and what
// waypoints.DEFER_CURRENT_CORRIDOR_REPLAN=false restores.
//
// OPEN-CHALLENGE-ONLY by construction rather than by the flag: callers build
// a Gate only for Open. On Obstacles the estimator is fixed and the bias
// comes from the Obstacles override, so the gate's confirmed-ness trigger
// would rebuild an identical path and re-seek the waypoint index for nothing.
func New(enabled bool) *Gate {
	return &Gate{
		enabled: enabled,
		applied: make(map[trackmodel.Section]float64),
		// A blind round starts with nothing measured. Update overwrites this
		// wholesale on its first call.
		appliedUnconfirmed: waypoints.AllUnconfirmed(),
	}
}

// Update folds the estimator's current belief in, holding back the corridor
// the robot is standing in, and returns the widths to plan from, the
// resulting unconfirmed set, and whether either differs from what the caller
// last planned on (so it can be used directly as "rebuild the path now").
//
// unconfirmed is the estimator's OWN confirmed-ness -- the sections it has
// not yet measured. The returned set is the PLAN's, which trails it.
//
// Call this EVERY tick, not only when the estimator reports a change: a
// belief held back is released by the robot MOVING, not by a new reading, so
// the tick that finally applies it is usually one where the estimator said
// nothing at all.
//
// Width and confirmed-ness are gated as ONE unit, because both move the
// planned line and they do not always move together. A corridor that is
// genuinely narrow confirms at the value the prior already held: the width
// does not change at all, but the section stops being unconfirmed, which
// drops UnconfirmedWidthInnerBiasM and shifts the line 0.05 m outward.
// Gating the width alone would let that one through -- a smaller step than
// the 0.30 m case, in the corridor least able to afford being surprised, and
// invisible to any test that only checks widths.
//
// current is the section the robot is in right now, attributed by HEADING
// rather than position: position would be circular here, since the planned
// line the position is measured against is what this gate decides.
func (g *Gate) Update(
	believed map[trackmodel.Section]float64,
	unconfirmed waypoints.UnconfirmedSections,
	current trackmodel.Section,
) (map[trackmodel.Section]float64, waypoints.UnconfirmedSections, bool) {
	if !g.seeded {
		// Nothing planned from us yet -- adopt the prior wholesale. Reported
		// as UNCHANGED because the caller's initial path was already built
		// from exactly these values; saying changed would force a redundant
		// replan on the first tick of every round.
		g.seeded = true
		g.applied = maps.Clone(believed)
		g.appliedUnconfirmed = unconfirmed
		return maps.Clone(g.applied), g.appliedUnconfirmed, false
	}

	changed := false
	for section, width := range believed {
		appliedWidth, haveApplied := g.applied[section]
		sameWidth := haveApplied && appliedWidth == width
		sameConfirmedness := g.appliedUnconfirmed.Contains(section) == unconfirmed.Contains(section)
		if sameWidth && sameConfirmedness {
			continue
		}
		if g.enabled && section == current {
			// Standing in it -- this is the one change that would move the
			// line being tracked. Hold it; the robot will leave shortly and a
			// later tick releases it through this same loop.
			continue
		}
		g.applied[section] = width
		g.appliedUnconfirmed.Set(section, unconfirmed.Contains(section))
		changed = true
	}
	return maps.Clone(g.applied), g.appliedUnconfirmed, changed
}
