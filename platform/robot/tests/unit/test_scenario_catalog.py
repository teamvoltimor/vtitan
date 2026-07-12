"""The live-visualizer's Open Challenge catalog loads the Go-generated fixtures."""

from __future__ import annotations

import pytest

from src.simulation.scenario_catalog import all_test_scenarios, find_scenario


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


def test_find_scenario_by_index_out_of_range_raises() -> None:
    with pytest.raises(ValueError, match="out of range"):
        find_scenario("99", all_test_scenarios())


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
