"""Hold a corridor-width change back until the robot has left that corridor.

A width belief update rebuilds the planned path, and the path is what crosstrack
and the steering target are measured against. When the corridor whose width
changed is the one the robot is standing in, that rebuild moves the line the
robot is *actively tracking* -- measured on hardware 2026-08-30 across six runs
as a ~0.30 m crosstrack step in a single 50 ms tick, ten times what the chassis
can physically travel in that time, which threw heading error past
``heading.CRAWL`` and pinned the limiter for 82-100% of the ticks that followed.

The same update applied to a corridor the robot is NOT in costs nothing: the
robot arrives on the new line instead of being displaced onto it.

So the step is not inherent to replanning, only to replanning *underneath* the
chassis. Deferring the change until the robot leaves that section removes it
rather than shrinking it (``UNCONFIRMED_WIDTH_INNER_BIAS_M``) or spreading it
over time (``REPLAN_BLEND_TICKS``, refuted twice -- a path that slides under the
robot for a second measured worse than one that jumps once and settles).

What deferring costs is small and bounded: the current corridor keeps planning
on the old belief for the remainder of one traverse, so it is centred slightly
wrong for that stretch and correct from the next lap onward. What it cannot
disturb is the corner geometry, because ``CORNER_ARC_ASSUME_WIDE`` already sizes
every arc as if both corridors were WIDE and so no arc depends on the belief
being deferred here.

Note the estimator only ever observes the section the robot currently occupies
(see ``TrackNavigator._update_layout_belief``), so with deferral ON essentially
every change is pending at the moment it is discovered and lands one corridor
later. That is the intended behaviour, not an edge case.
"""

from __future__ import annotations

from shared.domain.enums import Section


class DeferredWidthBelief:
    """The widths the PATH is planned from, which trail the estimator's own.

    Not a second estimator: it never forms an opinion about a width, it only
    decides *when* an opinion already formed is allowed to move the path.
    """

    def __init__(self, *, enabled: bool) -> None:
        """Set up the gate.

        Args:
            enabled: Whether to defer at all. ``False`` applies every change
                immediately, which is the behaviour before this class existed
                and what the ``DEFER_CURRENT_CORRIDOR_REPLAN`` flag restores.
        """
        self._enabled = enabled
        self._applied: dict[Section, float] = {}
        self._applied_observed: set[Section] = set()

    def update(
        self,
        believed: dict[Section, float],
        observed: set[Section],
        current: Section,
    ) -> tuple[dict[Section, float], frozenset[Section], bool]:
        """Fold the estimator's current belief in, holding back the current corridor.

        Call this every tick, not only when the estimator reports a change: a
        belief held back is released by the robot *moving*, not by a new
        reading, so the tick that finally applies it is usually one where the
        estimator said nothing at all.

        Width and confirmed-ness are gated as ONE unit, because both move the
        planned line and they do not always move together. A corridor that is
        genuinely narrow confirms at the value the prior already held: the width
        does not change at all, but the section stops being unconfirmed, which
        drops ``UNCONFIRMED_WIDTH_INNER_BIAS_M`` and shifts the line 0.05 m
        outward. Gating the width alone would let that one through -- a smaller
        step than the 0.30 m case, in the corridor least able to afford being
        surprised, and invisible to any test that only checks widths.

        Args:
            believed: The estimator's widths for every section.
            observed: Sections the estimator has MEASURED rather than assumed,
                from ``CorridorWidthEstimator.observed_sections``.
            current: The section the robot is in right now, attributed by
                heading rather than position -- see the callers for why
                position would be circular here.

        Returns:
            ``(widths_to_plan_from, unconfirmed_sections, changed)``.
            ``changed`` is True only when either half differs from what the
            caller last planned on, so it can be used directly as "rebuild the
            path now". ``unconfirmed_sections`` is in the form
            :func:`~src.navigation.planning.waypoints.calculate_waypoints`
            wants, i.e. the complement of the applied observed set.
        """
        if not self._applied:
            # Nothing planned from us yet -- adopt the prior wholesale. Reported
            # as unchanged because the caller's initial path was already built
            # from exactly these values; saying True would force a redundant
            # replan on the first tick of every round.
            self._applied = dict(believed)
            self._applied_observed = set(observed)
            return dict(self._applied), self._unconfirmed(), False

        changed = False
        for section, width in believed.items():
            was_observed = section in self._applied_observed
            is_observed = section in observed
            if self._applied.get(section) == width and was_observed == is_observed:
                continue
            if self._enabled and section is current:
                # Standing in it -- this is the one change that would move the
                # line being tracked. Hold it; the robot will leave shortly and
                # a later tick releases it through the branch below.
                continue
            self._applied[section] = width
            if is_observed:
                self._applied_observed.add(section)
            else:
                self._applied_observed.discard(section)
            changed = True
        return dict(self._applied), self._unconfirmed(), changed

    def _unconfirmed(self) -> frozenset[Section]:
        """Sections the PLAN still treats as assumed, which trails the estimator."""
        return frozenset(Section) - self._applied_observed
