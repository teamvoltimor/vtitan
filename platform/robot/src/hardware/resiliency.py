"""Hardware failure and resiliency management for the Robot layer."""

import logging
import time
from collections.abc import Callable
from typing import ParamSpec, TypeVar

from shared.domain.exceptions import HardwareError

logger = logging.getLogger(__name__)

P = ParamSpec("P")
R = TypeVar("R")


def with_retry(
    max_retries: int = 3,
    delay_sec: float = 0.5,
    exceptions: tuple[type[Exception], ...] = (HardwareError, IOError),
) -> Callable[[Callable[P, R]], Callable[P, R]]:
    """Decorator to retry hardware initialization or polling on transient failures.

    Args:
        max_retries: Number of times to retry before giving up.
        delay_sec: Delay between retries in seconds.
        exceptions: Tuple of exceptions to catch and retry.
    """

    def decorator(func: Callable[P, R]) -> Callable[P, R]:
        def wrapper(*args: P.args, **kwargs: P.kwargs) -> R:
            last_err = None
            for attempt in range(1, max_retries + 1):
                try:
                    return func(*args, **kwargs)
                except exceptions as e:
                    last_err = e
                    logger.warning("Hardware action failed (attempt %d/%d): %s", attempt, max_retries, e)
                    if attempt < max_retries:
                        time.sleep(delay_sec)

            logger.error("Hardware action failed completely after retries.")
            if last_err:
                raise last_err
            msg = "Hardware action failed completely after retries."
            raise HardwareError(msg)

        return wrapper

    return decorator
