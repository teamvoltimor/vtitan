"""Scenario-derivation constants shared across the simulation battery.

Single-sources values that ``scenario_catalog.py``, ``find_recovery_envelope.py``
and ``visualize_scenario.py`` each re-derived independently from
``shared.config.constants``, so a change to a competition rule or corridor
dimension cannot update one of them and silently leave the others stale.
"""

from __future__ import annotations

from shared.config.constants import CompetitionSpecs, CorridorDimensions

N_LAPS = CompetitionSpecs.OPEN_CHALLENGE_LAPS
"""Laps for an Open Challenge scenario run, i.e. a full round."""

NARROW_MM = int(CorridorDimensions.NARROW * 1000)
"""Narrow corridor width in millimetres, as the Go generator's CLI takes it."""

WIDE_MM = int(CorridorDimensions.WIDE * 1000)
"""Wide corridor width in millimetres, as the Go generator's CLI takes it."""
