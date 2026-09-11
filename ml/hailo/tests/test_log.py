"""Unit tests for src.log's structured JSON logging setup."""

from __future__ import annotations

import json
import logging

from src.log import _JsonFormatter, configure_logging, get_logger


def test_get_logger_returns_a_logger_bound_to_the_given_name() -> None:
    logger = get_logger("some.module")
    assert isinstance(logger, logging.Logger)
    assert logger.name == "some.module"


def test_json_formatter_emits_valid_json_with_expected_keys() -> None:
    record = logging.LogRecord(
        name="test.logger",
        level=logging.INFO,
        pathname=__file__,
        lineno=1,
        msg="hello %s",
        args=("world",),
        exc_info=None,
    )
    payload = json.loads(_JsonFormatter().format(record))

    assert payload["level"] == "INFO"
    assert payload["logger"] == "test.logger"
    assert payload["msg"] == "hello world"
    assert "ts" in payload
    assert "exc" not in payload


def _raise_value_error() -> None:
    msg = "boom"
    raise ValueError(msg)


def test_json_formatter_includes_exc_info_when_present() -> None:
    try:
        _raise_value_error()
    except ValueError:
        import sys  # noqa: PLC0415

        record = logging.LogRecord(
            name="test.logger",
            level=logging.ERROR,
            pathname=__file__,
            lineno=1,
            msg="failed",
            args=(),
            exc_info=sys.exc_info(),
        )

    payload = json.loads(_JsonFormatter().format(record))
    assert "ValueError: boom" in payload["exc"]


def test_configure_logging_wires_a_json_stream_handler() -> None:
    configure_logging(level="DEBUG")
    root = logging.getLogger()

    assert root.level == logging.DEBUG
    assert len(root.handlers) == 1
    assert isinstance(root.handlers[0].formatter, _JsonFormatter)


def test_configure_logging_defaults_to_info() -> None:
    configure_logging()
    assert logging.getLogger().level == logging.INFO
