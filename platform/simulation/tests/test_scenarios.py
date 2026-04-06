"""Unit tests for scenario coordinate transformations."""

import pytest
from shared.config.enums import Section
from src.generation.scenarios import apply_scenario_to_section, SCENARIOS


def test_apply_scenario_to_south():
    # Scenario 1 is [("green", 1.0, 0.6)]
    # South should return unmodified (x, y) where x=1.0, y=0.6
    signs = apply_scenario_to_section(1, Section.SOUTH)
    assert len(signs) == 1
    assert signs[0] == ("green", 1.0, 0.6)


def test_apply_scenario_to_north():
    # North flips Y only: y = 3.0 - 0.6 = 2.4, x remains 1.0
    signs = apply_scenario_to_section(1, Section.NORTH)
    assert len(signs) == 1
    assert signs[0] == ("green", 1.0, 2.4)


def test_apply_scenario_to_east():
    # East swaps x and y, and x = 3.0 - y = 3.0 - 0.6 = 2.4, y = 1.0
    signs = apply_scenario_to_section(1, Section.EAST)
    assert len(signs) == 1
    assert signs[0] == ("green", 2.4, 1.0)


def test_apply_scenario_to_west():
    # West swaps x and y, and x = y = 0.6, y = 1.0
    signs = apply_scenario_to_section(1, Section.WEST)
    assert len(signs) == 1
    assert signs[0] == ("green", 0.6, 1.0)


def test_invalid_scenario_id():
    with pytest.raises(ValueError):
        apply_scenario_to_section(99, Section.SOUTH)


def test_multiple_signs_in_scenario():
    # Scenario 13: [("green", 1.0, 0.4), ("green", 2.0, 0.6)]
    signs = apply_scenario_to_section(13, Section.WEST)
    assert len(signs) == 2
    assert signs[0] == ("green", 0.4, 1.0)
    assert signs[1] == ("green", 0.6, 2.0)
