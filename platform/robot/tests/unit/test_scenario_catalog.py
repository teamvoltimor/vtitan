"""The live-visualizer's scenario catalog must mirror TestThreeLapSolvability exactly."""

from __future__ import annotations

import pytest

from src.simulation.scenario_catalog import all_test_scenarios, find_scenario


def test_catalog_has_one_entry_per_test_case() -> None:
    # 8 symmetric-wide + 8 symmetric-narrow + 4 mixed + 8 random = 28.
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
    found = find_scenario("random#3", scenarios)
    assert found.label.startswith("random#3")


def test_find_scenario_by_ambiguous_label_raises() -> None:
    with pytest.raises(ValueError, match="multiple scenarios"):
        find_scenario("symmetric_wide", all_test_scenarios())


def test_find_scenario_by_unknown_label_raises() -> None:
    with pytest.raises(ValueError, match="No scenario"):
        find_scenario("does-not-exist", all_test_scenarios())
