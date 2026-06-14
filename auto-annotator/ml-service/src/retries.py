"""src.retries – Retry logic for transient failures (timeouts, file locks, etc.).

Exponential backoff for network operations, file I/O, and other transient errors.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, TypeVar, overload

from src.utils import get_logger

if TYPE_CHECKING:
    from collections.abc import Callable

logger = get_logger(__name__)

T = TypeVar("T")


@overload
def retry[T](
    func: Callable[..., T],
    *,
    max_attempts: int = 3,
    base_delay_ms: int = 100,
    max_delay_ms: int = 5000,
    backoff_factor: float = 2.0,
    retryable: Callable[[Exception], bool] | None = None,
) -> T: ...


def retry[T](
    func: Callable[..., T],
    *,
    max_attempts: int = 3,
    base_delay_ms: int = 100,
    max_delay_ms: int = 5000,
    backoff_factor: float = 2.0,
    retryable: Callable[[Exception], bool] | None = None,
) -> T:
    """Retry a function call with exponential backoff.

    Useful for transient failures like file locks, network timeouts, or brief service unavailability.

    Args:
        func: Callable to retry.
        max_attempts: Maximum number of attempts (default 3).
        base_delay_ms: Initial delay in milliseconds between retries (default 100).
        max_delay_ms: Maximum delay in milliseconds (default 5000).
        backoff_factor: Multiplier for exponential backoff (default 2.0).
        retryable: Optional predicate to determine if an exception is retryable.
                   If None, retries all exceptions. If returns False, raises immediately.

    Returns:
        Return value of func.

    Raises:
        The last exception encountered if all attempts fail.

    Example:
        def is_transient(exc: Exception) -> bool:
            return isinstance(exc, (TimeoutError, ConnectionError))

        result = retry(
            some_api_call,
            max_attempts=5,
            retryable=is_transient,
        )
    """
    last_error: Exception | None = None
    delay_ms = base_delay_ms

    for attempt in range(1, max_attempts + 1):
        try:
            return func()
        except Exception as e:
            last_error = e

            if retryable and not retryable(e):
                raise

            if attempt == max_attempts:
                logger.exception(
                    "Retry exhausted after %d attempts",
                    max_attempts,
                    extra={"_extra": {"error": str(e), "last_delay_ms": delay_ms}},
                )
                raise

            logger.debug(
                "Attempt %d failed, retrying in %dms",
                attempt,
                delay_ms,
                extra={"_extra": {"error": str(e)}},
            )
            time.sleep(delay_ms / 1000.0)
            delay_ms = min(int(delay_ms * backoff_factor), max_delay_ms)

    if last_error:
        raise last_error
    msg = "Retry logic failed unexpectedly"
    raise RuntimeError(msg)
