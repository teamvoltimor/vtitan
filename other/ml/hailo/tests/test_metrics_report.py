"""Unit tests for src.metrics_report's JSON transport of MetricsResult."""

from __future__ import annotations

import pytest

from eval.metrics import MetricsResult
from src.metrics_report import MetricsReport

_RESULT = MetricsResult(
    mAP50=0.9,
    mAP75=0.8,
    mAP50_95=0.7,
    per_class={0: 0.9, 1: 0.8},
    per_class_5095={0: 0.7, 1: 0.6},
    confusion={(0, 0): 5, (0, -1): 1, (-1, 1): 2},
)


def test_from_result_and_to_result_round_trip() -> None:
    report = MetricsReport.from_result(_RESULT)
    assert report.to_result() == _RESULT


def test_json_round_trip_preserves_confusion_with_tuple_keys() -> None:
    report = MetricsReport.from_result(_RESULT)

    payload = report.model_dump_json()
    restored = MetricsReport.model_validate_json(payload)

    assert restored == report
    assert restored.to_result() == _RESULT


def test_json_dump_encodes_confusion_keys_as_strings() -> None:
    report = MetricsReport.from_result(_RESULT)
    dumped = report.model_dump(mode="json")

    assert dumped["confusion"] == {"0:0": 5, "0:-1": 1, "-1:1": 2}


def test_python_mode_dump_keeps_native_tuple_keys() -> None:
    report = MetricsReport.from_result(_RESULT)
    dumped = report.model_dump(mode="python")

    assert dumped["confusion"] == {(0, 0): 5, (0, -1): 1, (-1, 1): 2}


def test_report_is_frozen() -> None:
    report = MetricsReport.from_result(_RESULT)
    with pytest.raises(Exception, match=r"frozen|immutable"):
        report.mAP50 = 0.1  # type: ignore[misc]
