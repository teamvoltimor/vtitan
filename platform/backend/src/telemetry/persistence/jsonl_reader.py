"""Generic JSONL file reader for resilient parsing.

Provides typed, reusable abstraction for JSONL deserialization
(DRY Principle: eliminates duplicate parsing logic).
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Generic, Iterator, TypeVar

logger = logging.getLogger(__name__)

T = TypeVar("T")


class JSONLReader(Generic[T]):
    """Reads and deserializes JSONL files to typed models.

    Provides optional resilience to corrupted lines with detailed logging.
    """

    @staticmethod
    def read_file(
        path: Path,
        model: type[T],
        skip_invalid: bool = False,
    ) -> tuple[Iterator[T], list[tuple[int, Exception]]]:
        """Read JSONL file with optional error resilience.

        Args:
            path: Path to JSONL file.
            model: Pydantic model class for deserialization.
            skip_invalid: If True, skip corrupted lines; else raise.

        Returns:
            (iterator of models, list of (line_num, exception) for corrupted lines)

        Raises:
            FileNotFoundError: If file does not exist.
            ValueError: If skip_invalid=False and corrupted line encountered.
        """
        if not path.exists():
            raise FileNotFoundError(f"File not found: {path}")

        corrupted: list[tuple[int, Exception]] = []

        def read() -> Iterator[T]:
            with path.open("r", encoding="utf-8") as fh:
                for line_num, line in enumerate(fh, start=1):
                    text = line.strip()
                    if not text:
                        continue

                    try:
                        yield model.model_validate_json(text)
                    except Exception as exc:
                        if skip_invalid:
                            logger.warning(
                                f"Skipping corrupted line {line_num}",
                                exc_info=exc,
                                extra={"path": str(path)},
                            )
                            corrupted.append((line_num, exc))
                        else:
                            raise ValueError(f"Line {line_num}: {exc}") from exc

        return read(), corrupted
