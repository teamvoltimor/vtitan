"""Standardized error handling and recovery utilities for telemetry operations.

Provides centralized error handling, logging, recovery, and HTTP response mapping
across the telemetry system. All domain exceptions flow through ErrorHandler to
translate into appropriate HTTP responses, logs, and retries.

Usage:
    handler = ErrorHandler(logger, TelemetryError)
    try:
        with handler.retry("recorder.write"):
            recorder.write(snapshot)
    except RecorderError as e:
        response = handler.to_http_response(e)
        return response
"""

from __future__ import annotations

import logging
from contextlib import contextmanager
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Generic, TypeVar

from fastapi import HTTPException

from src.telemetry.exceptions import (
	BroadcastError,
	RecorderError,
	SessionNotFoundError,
	TelemetryError,
)

if TYPE_CHECKING:
	from collections.abc import Callable, Generator

logger = logging.getLogger(__name__)

T = TypeVar("T")
E = TypeVar("E", bound=Exception)


@dataclass
class ErrorContext:
    """Context information for an error occurrence.

    Fields:
        operation: Name of operation that failed
        error_type: Specific exception type
        message: User-friendly error message
        details: Additional context (dict of key-value pairs)
        severity: Error severity ('critical', 'error', 'warning')
        retry_count: Number of retries attempted
        recoverable: Whether error is recoverable
    """

    operation: str
    error_type: type[Exception]
    message: str
    details: dict[str, Any]
    severity: str = "error"  # 'critical', 'error', 'warning'
    retry_count: int = 0
    recoverable: bool = False

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "operation": self.operation,
            "error_type": self.error_type.__name__,
            "message": self.message,
            "details": self.details,
            "severity": self.severity,
            "retry_count": self.retry_count,
            "recoverable": self.recoverable,
        }


class ErrorHandler(Generic[E]):
    """Context manager for standardized error handling and recovery.

    Features:
        - Automatic error logging with context
        - Structured error information
        - Retry logic with exponential backoff
        - Recovery strategy dispatch
        - Type-safe exception handling

    Usage:
        try:
            with ErrorHandler(logger, ConfigurationError) as handler:
                config = load_config()
                handler.context({"config_file": path})
        except ConfigurationError as e:
            handler.log_error(e, severity="critical")
    """

    def __init__(
        self,
        logger_obj: logging.Logger,
        base_exception: type[E],
        max_retries: int = 3,
    ) -> None:
        """Initialize error handler.

        Args:
            logger_obj: Logger instance for output
            base_exception: Base exception type for this handler
            max_retries: Maximum retry attempts
        """
        self.logger = logger_obj
        self.base_exception = base_exception
        self.max_retries = max_retries
        self.error_context: ErrorContext | None = None

    def __enter__(self) -> ErrorHandler[E]:
        """Enter context manager."""
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> bool:
        """Exit context manager, logging any exception."""
        if exc_type is not None and issubclass(exc_type, self.base_exception):
            self.log_error(exc_val, severity="error")
            return False
        return False

    def context(self, operation: str, **details: Any) -> None:
        """Set error context information.

        Args:
            operation: Name of current operation
            **details: Additional context key-value pairs
        """
        self.error_context = ErrorContext(
            operation=operation,
            error_type=self.base_exception,
            message="",
            details=details,
        )

    def log_error(
        self,
        error: Exception,
        operation: str | None = None,
        severity: str = "error",
        recoverable: bool = False,
        **details: Any,
    ) -> ErrorContext:
        """Log an error with structured context.

        Args:
            error: Exception that occurred
            operation: Operation name (uses context if not provided)
            severity: Error severity level
            recoverable: Whether error can be recovered
            **details: Additional context

        Returns:
            ErrorContext: Context information logged
        """
        op = operation or (self.error_context.operation if self.error_context else "unknown")

        base_details = self.error_context.details if self.error_context else {}
        merged_details = {**base_details, **details}

        context = ErrorContext(
            operation=op,
            error_type=type(error),
            message=str(error),
            details=merged_details,
            severity=severity,
            recoverable=recoverable,
        )

        # Log with appropriate level
        log_method = {
            "critical": self.logger.critical,
            "error": self.logger.error,
            "warning": self.logger.warning,
        }.get(severity, self.logger.error)

        log_method(
            f"[{context.operation}] {context.error_type.__name__}: {context.message}",
            extra={"context": context.to_dict()},
        )

        return context

    @contextmanager
    def retry(
        self,
        operation: str,
        backoff_factor: float = 2.0,
        initial_delay: float = 0.1,
    ) -> Generator[int, None, None]:
        """Context manager for retry logic with exponential backoff.

        Args:
            operation: Operation name for logging
            backoff_factor: Multiplier for delay between retries
            initial_delay: Initial delay in seconds

        Yields:
            Retry attempt number (0-indexed)

        Example:
            for attempt in handler.retry("database_query", backoff_factor=2.0):
                try:
                    result = query()
                    break
                except ConnectionError as e:
                    if attempt < handler.max_retries - 1:
                        continue
                    raise
        """
        import time

        for attempt in range(self.max_retries):
            try:
                yield attempt
                return
            except self.base_exception as e:
                if attempt < self.max_retries - 1:
                    delay = initial_delay * (backoff_factor ** attempt)
                    self.logger.warning(
                        f"[{operation}] Retry {attempt + 1}/{self.max_retries} after {delay:.2f}s: {e}"
                    )
                    time.sleep(delay)
                else:
                    self.log_error(e, operation=operation, severity="critical")
                    raise

    def handle_batch(
        self,
        items: list[Any],
        operation: str,
        handler_func: Callable[[Any], Any],
        skip_on_error: bool = False,
    ) -> tuple[list[Any], list[ErrorContext]]:
        """Handle batch processing with individual error tracking.

        Args:
            items: Items to process
            operation: Operation name
            handler_func: Function to apply to each item
            skip_on_error: Skip failed items (True) or raise (False)

        Returns:
            Tuple of (successful_results, error_contexts)

        Raises:
            TelemetryError: If skip_on_error=False and any item fails
        """
        results: list[Any] = []
        errors: list[ErrorContext] = []

        for i, item in enumerate(items):
            try:
                result = handler_func(item)
                results.append(result)
            except self.base_exception as e:
                ctx = self.log_error(
                    e,
                    operation=f"{operation}[{i}]",
                    severity="warning" if skip_on_error else "error",
                )
                errors.append(ctx)

                if not skip_on_error:
                    raise

        return results, errors

    def to_http_response(self, error: Exception) -> HTTPException:
        """Translate domain exception to HTTP response.

        Maps TelemetryError subclasses to appropriate HTTP status codes and messages.
        Logs the error before returning, ensuring all errors are centrally tracked.

        Args:
            error: Domain exception from telemetry system.

        Returns:
            HTTPException with appropriate status code.
        """
        status_code = 500
        detail = "Internal server error"

        if isinstance(error, SessionNotFoundError):
            status_code = 404
            detail = f"Session not found: {error}"
        elif isinstance(error, RecorderError):
            status_code = 503
            detail = f"Recording system unavailable: {error}"
        elif isinstance(error, BroadcastError):
            status_code = 503
            detail = f"Broadcast system unavailable: {error}"
        elif isinstance(error, TelemetryError):
            status_code = 500
            detail = f"Telemetry error: {error}"
        else:
            status_code = 500
            detail = "Internal server error"

        self.log_error(
            error,
            severity="error" if status_code < 500 else "critical",
            recoverable=status_code == 503,
        )

        return HTTPException(status_code=status_code, detail=detail)
