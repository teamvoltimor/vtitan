"""Unit tests for the corridor-width replan gate."""

from __future__ import annotations

from shared.config.constants import CorridorDimensions
from shared.domain.enums import Section

from src.navigation.deferred_width_belief import DeferredWidthBelief

NARROW = CorridorDimensions.NARROW
WIDE = CorridorDimensions.WIDE

_PRIOR = dict.fromkeys(Section, NARROW)


def _seeded(*, enabled: bool) -> DeferredWidthBelief:
    """A gate that has already adopted the all-narrow prior, as a round's first tick does."""
    gate = DeferredWidthBelief(enabled=enabled)
    gate.update(_PRIOR, set(), Section.NORTH)
    return gate


class TestDisabled:
    """Flag off must reproduce the ungated behaviour exactly."""

    def test_applies_the_current_corridor_immediately(self) -> None:
        gate = _seeded(enabled=False)
        widths, _, changed = gate.update({**_PRIOR, Section.NORTH: WIDE}, {Section.NORTH}, Section.NORTH)
        assert changed
        assert widths[Section.NORTH] == WIDE


class TestFirstTick:
    """Adopting the prior must not itself count as a change."""

    def test_seeding_reports_unchanged(self) -> None:
        """The caller's initial path was already built from these values.

        Reporting True here would force a redundant replan on the first tick of
        every round -- the exact class of spurious path swap this gate exists to
        remove.
        """
        gate = DeferredWidthBelief(enabled=True)
        _, unconfirmed, changed = gate.update(_PRIOR, set(), Section.NORTH)
        assert not changed
        assert unconfirmed == frozenset(Section)


class TestDeferral:
    def test_holds_a_change_to_the_corridor_being_driven(self) -> None:
        gate = _seeded(enabled=True)
        widths, unconfirmed, changed = gate.update({**_PRIOR, Section.NORTH: WIDE}, {Section.NORTH}, Section.NORTH)
        assert not changed
        assert widths[Section.NORTH] == NARROW
        assert Section.NORTH in unconfirmed

    def test_releases_it_once_the_robot_has_left(self) -> None:
        """The release is driven by MOVING, not by a new reading.

        The estimator only ever observes the section the robot occupies, so the
        tick that finally applies a held width is one where the estimator said
        nothing at all -- which is why the caller must gate every tick rather
        than only when observe() returns True.
        """
        gate = _seeded(enabled=True)
        believed, observed = {**_PRIOR, Section.NORTH: WIDE}, {Section.NORTH}
        gate.update(believed, observed, Section.NORTH)

        widths, unconfirmed, changed = gate.update(believed, observed, Section.EAST)
        assert changed
        assert widths[Section.NORTH] == WIDE
        assert Section.NORTH not in unconfirmed

    def test_a_change_elsewhere_is_never_held(self) -> None:
        """Only the tracked line is at risk; other corridors cost nothing to update."""
        gate = _seeded(enabled=True)
        widths, _, changed = gate.update({**_PRIOR, Section.EAST: WIDE}, {Section.EAST}, Section.NORTH)
        assert changed
        assert widths[Section.EAST] == WIDE


class TestConfirmationIsGatedWithTheWidth:
    """The two must move as one unit -- see the class docstring.

    A genuinely narrow corridor confirms at the value the prior already held, so
    the width does not change while the section stops being unconfirmed. That
    alone drops UNCONFIRMED_WIDTH_INNER_BIAS_M and shifts the planned line
    0.05 m outward. Gating the width alone lets that through, in the corridor
    least able to afford a surprise, and no width-only assertion would catch it.
    """

    def test_confirming_the_prior_value_is_still_held(self) -> None:
        gate = _seeded(enabled=True)
        widths, unconfirmed, changed = gate.update(_PRIOR, {Section.NORTH}, Section.NORTH)
        assert not changed
        assert widths[Section.NORTH] == NARROW
        assert Section.NORTH in unconfirmed, "confirmation leaked while the width was held"

    def test_and_is_released_together_with_it(self) -> None:
        gate = _seeded(enabled=True)
        gate.update(_PRIOR, {Section.NORTH}, Section.NORTH)
        _, unconfirmed, changed = gate.update(_PRIOR, {Section.NORTH}, Section.EAST)
        assert changed
        assert Section.NORTH not in unconfirmed
