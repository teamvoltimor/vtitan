"""Unit tests for ScenarioRandomizer."""

import pytest
import math
from shared.config.constants import DictKeys, TrackDimensions
from shared.config.enums import Section, Direction
from src.generation.randomizer import (
    ScenarioRandomizer,
    _pick_start_position,
    _compute_starting_yaw,
    _compute_second_block_depth,
    _parking_positions_for_section,
    _zone_from_parking,
)


@pytest.fixture
def mock_config():
    return {
        DictKeys.COLORS: {
            "red": {DictKeys.MEAN: [1.0, 0.0, 0.0], DictKeys.STD: [0.1, 0.1, 0.1]},
            "green": {DictKeys.MEAN: [0.0, 1.0, 0.0], DictKeys.STD: [0.1, 0.1, 0.1]},
        }
    }


@pytest.fixture
def randomizer(mock_config):
    return ScenarioRandomizer(mock_config)


def test_randomize_corridor_widths(randomizer):
    widths = randomizer.randomize_corridor_widths()
    assert len(widths) == 4
    for section in [Section.NORTH, Section.SOUTH, Section.EAST, Section.WEST]:
        assert section in widths
        assert DictKeys.WIDTH in widths[section]
        assert DictKeys.TYPE in widths[section]


def test_randomize_starting_conditions(randomizer):
    widths = {
        Section.SOUTH: {DictKeys.WIDTH: 0.6, DictKeys.TYPE: "narrow"},
        Section.NORTH: {DictKeys.WIDTH: 1.0, DictKeys.TYPE: "wide"},
        Section.EAST: {DictKeys.WIDTH: 1.0, DictKeys.TYPE: "wide"},
        Section.WEST: {DictKeys.WIDTH: 1.0, DictKeys.TYPE: "wide"},
    }
    conds = randomizer.randomize_starting_conditions(widths)
    assert DictKeys.DIRECTION in conds
    assert DictKeys.SECTION in conds
    assert DictKeys.POSITION in conds
    assert DictKeys.YAW in conds
    assert isinstance(conds[DictKeys.POSITION], tuple)


def test_generate_parking_lot_positions(randomizer):
    parking = randomizer.generate_parking_lot_positions(Section.SOUTH)
    assert DictKeys.BLOCK1_POS in parking
    assert DictKeys.BLOCK2_POS in parking
    assert DictKeys.BLOCK1_YAW in parking
    assert DictKeys.BLOCK2_YAW in parking
    assert DictKeys.DEPTH in parking


def test_generate_starting_zone(randomizer):
    zone_no_park = randomizer.generate_starting_zone(Section.SOUTH, 0.6, None)
    assert "x" in zone_no_park
    assert "y" in zone_no_park
    assert "length" in zone_no_park

    parking_config = randomizer.generate_parking_lot_positions(Section.SOUTH)
    zone_park = randomizer.generate_starting_zone(Section.SOUTH, 1.0, parking_config)
    assert "x" in zone_park
    assert "y" in zone_park
    assert "length" in zone_park


def test_compute_starting_yaw():
    yaw = _compute_starting_yaw(Section.SOUTH, Direction.CLOCKWISE)
    assert yaw == math.pi

    yaw = _compute_starting_yaw(Section.NORTH, Direction.COUNTERCLOCKWISE)
    assert yaw == math.pi


def test_pick_start_position():
    pos = _pick_start_position(Section.SOUTH, 1.0)
    # Expected x to be one of [1.0, 1.5, 2.0] and y to be 0.5
    assert pos[0] in [1.0, 1.5, 2.0]
    assert pos[1] == 0.5


def test_compute_second_block_depth():
    d = _compute_second_block_depth(1.0, 0.225)
    assert d == 1.225
    d = _compute_second_block_depth(2.0, 0.225)
    assert d == 1.775


def test_parking_positions_for_section():
    b1, b2, yaw = _parking_positions_for_section(Section.SOUTH, 1.0, 1.225, 0.1)
    assert b1 == (1.0, 0.1)
    assert b2 == (1.225, 0.1)


def test_zone_from_parking():
    parking_config = {
        DictKeys.BLOCK1_POS: (1.0, 0.1),
        DictKeys.BLOCK2_POS: (1.5, 0.1),
    }
    length, x, y = _zone_from_parking(Section.SOUTH, parking_config, 0.5, 3.0)
    # Centered between 1.0 and 1.5 => 1.25
    assert x == 1.25
    assert y == 0.1
    # Spacing is 0.5. Available gap is 0.5 - 0.02 = 0.48.
    # Length should be min(0.5, 0.48 * 0.9 = 0.432) => 0.432
    assert length == pytest.approx(0.432)
