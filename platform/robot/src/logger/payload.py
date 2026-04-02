import json
import logging
from dataclasses import asdict, dataclass
from datetime import UTC, datetime

from src.logger.constants import DETAILS_KEY


@dataclass
class LogPayload:
    """Represents a structured log entry for the logging system."""

    timestamp: str
    level: str
    logger: str
    message: str
    details: dict[str, object] | None = None
    exc_info: str | None = None

    def __init__(self, record: logging.LogRecord) -> None:
        """
        Initialize a LogPayload with default values.

        Args:
            record: The LogRecord from which to extract log information. The actual values will be set by the
            logger before serialization.
        """
        # Initialize with default values; actual values will be set by the logger before serialization.
        self.timestamp = datetime.now(UTC).isoformat()
        self.level = record.levelname.lower()
        self.logger = record.name

        # getMessage() already interpolates record.args into the message string,
        # so we never put args in the payload — third-party libraries (e.g. urllib3)
        # pass non-JSON-serializable objects like Retry as args.
        self.message = record.getMessage()

        # If the record has a 'details' attribute that is a non-empty dict, include it in the payload
        details = getattr(record, DETAILS_KEY, None)
        if isinstance(details, dict) and details:
            self.details = details

        # Include formatted exception info if exc_info is set
        if record.exc_info:
            self.exc_info = logging.Formatter().formatException(record.exc_info)

    def to_json(self) -> str:
        """
        Convert the LogPayload to a JSON string, omitting any None values.

        Returns:
            A JSON string representation of the log payload.
        """
        # Filter out None values to keep the JSON clean
        data = {k: v for k, v in asdict(self).items() if v is not None}

        # This one line handles everything:
        # 1. Normal JSON types are encoded normally.
        # 2. Anything else (dates, classes, etc.) is converted to a string.
        return json.dumps(data, default=str, ensure_ascii=False)
