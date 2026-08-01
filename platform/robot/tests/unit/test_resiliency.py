"""Unit tests for the ``with_retry`` decorator."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest
from shared.domain.exceptions import HardwareError

from src.hardware.resiliency import with_retry


class _CustomError(Exception):
    """A distinct exception type for testing custom ``exceptions`` tuples."""


class _OtherError(Exception):
    """An exception type never listed in ``exceptions``, for propagation tests."""


def _patch_sleep(monkeypatch: pytest.MonkeyPatch) -> MagicMock:
    sleep = MagicMock()
    monkeypatch.setattr("src.hardware.resiliency.time.sleep", sleep)
    return sleep


def test_succeeds_first_call_no_retry_no_sleep(monkeypatch: pytest.MonkeyPatch) -> None:
    sleep = _patch_sleep(monkeypatch)
    func = MagicMock(return_value="ok")
    wrapped = with_retry()(func)

    result = wrapped()

    assert result == "ok"
    assert func.call_count == 1
    sleep.assert_not_called()


def test_succeeds_after_transient_failures(monkeypatch: pytest.MonkeyPatch) -> None:
    sleep = _patch_sleep(monkeypatch)
    func = MagicMock(side_effect=[HardwareError("1"), HardwareError("2"), "ok"])
    wrapped = with_retry(max_retries=3)(func)

    result = wrapped()

    assert result == "ok"
    assert func.call_count == 3
    assert sleep.call_count == 2


def test_exhausts_retries_and_raises_last_error(monkeypatch: pytest.MonkeyPatch) -> None:
    sleep = _patch_sleep(monkeypatch)
    errors = [HardwareError("first"), HardwareError("second"), HardwareError("last")]
    func = MagicMock(side_effect=errors)
    wrapped = with_retry(max_retries=3)(func)

    with pytest.raises(HardwareError) as exc_info:
        wrapped()

    assert exc_info.value is errors[-1]
    assert func.call_count == 3
    assert sleep.call_count == 2


def test_unhandled_exception_propagates_immediately(monkeypatch: pytest.MonkeyPatch) -> None:
    sleep = _patch_sleep(monkeypatch)
    func = MagicMock(side_effect=_OtherError("boom"))
    wrapped = with_retry(max_retries=3, exceptions=(HardwareError,))(func)

    with pytest.raises(_OtherError):
        wrapped()

    assert func.call_count == 1
    sleep.assert_not_called()


def test_custom_exceptions_tuple_only_retries_listed_types(monkeypatch: pytest.MonkeyPatch) -> None:
    sleep = _patch_sleep(monkeypatch)
    func = MagicMock(side_effect=[_CustomError("1"), "ok"])
    wrapped = with_retry(max_retries=3, exceptions=(_CustomError,))(func)

    result = wrapped()

    assert result == "ok"
    assert func.call_count == 2
    assert sleep.call_count == 1


def test_custom_exceptions_tuple_does_not_retry_unlisted_type(monkeypatch: pytest.MonkeyPatch) -> None:
    sleep = _patch_sleep(monkeypatch)
    func = MagicMock(side_effect=HardwareError("nope"))
    wrapped = with_retry(max_retries=3, exceptions=(_CustomError,))(func)

    with pytest.raises(HardwareError):
        wrapped()

    assert func.call_count == 1
    sleep.assert_not_called()


def test_max_retries_one_means_single_attempt_no_sleep(monkeypatch: pytest.MonkeyPatch) -> None:
    sleep = _patch_sleep(monkeypatch)
    func = MagicMock(side_effect=HardwareError("nope"))
    wrapped = with_retry(max_retries=1)(func)

    with pytest.raises(HardwareError):
        wrapped()

    assert func.call_count == 1
    sleep.assert_not_called()


def test_delay_sec_is_passed_to_sleep(monkeypatch: pytest.MonkeyPatch) -> None:
    sleep = _patch_sleep(monkeypatch)
    func = MagicMock(side_effect=[HardwareError("1"), "ok"])
    wrapped = with_retry(max_retries=2, delay_sec=2.5)(func)

    wrapped()

    sleep.assert_called_once_with(2.5)


def test_max_retries_zero_never_calls_func_and_raises_fallback(monkeypatch: pytest.MonkeyPatch) -> None:
    """``range(1, 1)`` is empty, so the loop body never runs and ``last_err`` stays ``None``.

    This reaches the otherwise-dead ``raise HardwareError(msg)`` fallback path.
    """
    sleep = _patch_sleep(monkeypatch)
    func = MagicMock(return_value="ok")
    wrapped = with_retry(max_retries=0)(func)

    with pytest.raises(HardwareError, match=r"Hardware action failed completely after retries\."):
        wrapped()

    func.assert_not_called()
    sleep.assert_not_called()


def test_wrapped_function_receives_args_and_kwargs(monkeypatch: pytest.MonkeyPatch) -> None:
    _patch_sleep(monkeypatch)
    func = MagicMock(return_value="ok")
    wrapped = with_retry()(func)

    result = wrapped(1, 2, key="value")

    assert result == "ok"
    func.assert_called_once_with(1, 2, key="value")
