"""The live-visualizer's Open Challenge catalog loads the Go-generated fixtures."""

from __future__ import annotations

import pytest

from src.simulation.scenario_catalog import (
    all_open_scenarios,
    all_test_scenarios,
    find_scenario,
    open_scenario_by_index,
)


def test_catalog_loads_all_fixtures() -> None:
    # tests/fixtures/scenarios/open/*_metadata.json
    assert len(all_test_scenarios()) == 28


def test_labels_are_unique() -> None:
    labels = [s.label for s in all_test_scenarios()]
    assert len(labels) == len(set(labels))


def test_find_scenario_by_index() -> None:
    scenarios = all_test_scenarios()
    assert find_scenario("0", scenarios) is scenarios[0]
    assert find_scenario("27", scenarios) is scenarios[27]


def test_find_scenario_by_index_wraps_modulo() -> None:
    scenarios = all_test_scenarios()
    assert find_scenario("28", scenarios) is scenarios[0]
    assert find_scenario("56", scenarios) is scenarios[0]
    assert find_scenario("29", scenarios) is scenarios[1]


def test_find_scenario_by_unique_label_substring() -> None:
    scenarios = all_test_scenarios()
    found = find_scenario("go_open_0003", scenarios)
    assert found.label.startswith("go_open_0003")


def test_find_scenario_by_ambiguous_label_raises() -> None:
    with pytest.raises(ValueError, match="multiple scenarios"):
        find_scenario("go_open", all_test_scenarios())


def test_find_scenario_by_unknown_label_raises() -> None:
    with pytest.raises(ValueError, match="No scenario"):
        find_scenario("does-not-exist", all_test_scenarios())


def test_dynamic_open_catalog_has_640_scenarios() -> None:
    scenarios = all_open_scenarios()
    assert len(scenarios) == 640


def test_dynamic_open_labels_are_unique() -> None:
    labels = [s.label for s in all_open_scenarios()]
    assert len(labels) == len(set(labels))


def test_dynamic_open_metadata_has_start_cell() -> None:
    scenario = open_scenario_by_index(0)
    sc = scenario.metadata["starting_conditions"]
    assert "position" in sc
    assert "yaw" in sc
    assert "section" in sc
    assert "direction" in sc


def test_dynamic_open_index_out_of_range_raises() -> None:
    with pytest.raises(ValueError, match="0-639"):
        open_scenario_by_index(640)
