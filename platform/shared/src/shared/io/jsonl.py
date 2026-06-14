"""JSONL file utilities for streaming JSON data.

Provides reusable JSONL (JSON Lines) operations:
- Reading/writing line-delimited JSON files
- Handling corrupted lines gracefully
- Counting lines efficiently
- Type-safe serialization with Pydantic models

This module centralizes JSONL logic previously duplicated in backend
recorder and simulation modules.

Usage:
    # Write Pydantic models to JSONL
    with JsonlWriter(Path("data.jsonl")) as writer:
        for item in items:
            writer.write(item.model_dump(by_alias=True))

    # Read with error recovery
    items = JsonlReader.read_all(
        Path("data.jsonl"),
        model=RobotSnapshot,
        skip_corrupted=True
    )
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import TYPE_CHECKING, Any, TypeVar

if TYPE_CHECKING:
    from collections.abc import Iterator

    from pydantic import BaseModel

logger = logging.getLogger(__name__)

T = TypeVar("T")


class JsonlWriter:
    """Write line-delimited JSON to a file.

    Features:
        - Context manager support (auto-close)
        - Automatic newline insertion
        - Error handling with logging
        - Lazy file opening
    """

    def __init__(self, path: Path | str, encoding: str = "utf-8") -> None:
        """Initialize JSONL writer.

        Args:
            path: File path to write to
            encoding: File encoding (default: utf-8)
        """
        self.path = Path(path)
        self.encoding = encoding
        self._file: Any = None

    def __enter__(self) -> JsonlWriter:
        """Enter context manager."""
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("a", encoding=self.encoding)
        return self

    def __exit__(self, *_: object) -> None:
        """Exit context manager and close file."""
        self.close()

    def write(self, obj: Any) -> None:
        """Write a single object as JSON line.

        Args:
            obj: Object to serialize (typically dict or model_dump())

        Raises:
            OSError: If file write fails
            ValueError: If object is not JSON-serializable
        """
        if self._file is None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._file = self.path.open("a", encoding=self.encoding)

        try:
            json.dump(obj, self._file, separators=(",", ":"))
            self._file.write("\n")
            self._file.flush()
        except OSError as e:
            logger.error(f"Failed to write JSONL to {self.path}: {e}", exc_info=True)
            raise
        except (TypeError, ValueError) as e:
            logger.error(f"Failed to serialize object to JSON: {e}", exc_info=True)
            raise

    def close(self) -> None:
        """Close the file handle if open."""
        if self._file is not None:
            self._file.close()
            self._file = None


class JsonlReader:
    """Read line-delimited JSON from a file.

    Features:
        - Streaming reads (memory efficient)
        - Pydantic model validation
        - Corrupted line handling
        - Error recovery
    """

    @staticmethod
    def iterate(
        path: Path | str,
        model: type[BaseModel] | None = None,
        skip_corrupted: bool = False,
    ) -> Iterator[dict[str, Any] | BaseModel]:
        """Iterate over JSONL lines, optionally validating with model.

        Args:
            path: File path to read from
            model: Optional Pydantic model for validation
            skip_corrupted: Skip corrupted lines (True) or raise (False)

        Yields:
            Dict or validated model instance for each line

        Raises:
            FileNotFoundError: If file does not exist
            json.JSONDecodeError: If line is not valid JSON (unless skip_corrupted=True)
        """
        file_path = Path(path)
        if not file_path.exists():
            raise FileNotFoundError(f"JSONL file not found: {file_path}")

        try:
            with file_path.open("r", encoding="utf-8") as f:
                for line_num, line in enumerate(f, start=1):
                    text = line.strip()
                    if not text:
                        continue

                    try:
                        data = json.loads(text)
                        if model:
                            yield model.model_validate(data)
                        else:
                            yield data
                    except (json.JSONDecodeError, ValueError) as e:
                        if skip_corrupted:
                            logger.warning(
                                f"Skipping corrupted line {line_num} in {file_path}: {e}"
                            )
                            continue
                        else:
                            logger.error(
                                f"Error parsing line {line_num} in {file_path}: {e}",
                                exc_info=True,
                            )
                            raise
        except OSError as e:
            logger.error(f"Failed to read JSONL from {file_path}: {e}", exc_info=True)
            raise

    @staticmethod
    def read_all(
        path: Path | str,
        model: type[BaseModel] | None = None,
        skip_corrupted: bool = False,
    ) -> list[dict[str, Any] | BaseModel]:
        """Read all lines from JSONL file into memory.

        Args:
            path: File path to read from
            model: Optional Pydantic model for validation
            skip_corrupted: Skip corrupted lines

        Returns:
            List of dicts or validated model instances
        """
        return list(
            JsonlReader.iterate(path, model=model, skip_corrupted=skip_corrupted)
        )

    @staticmethod
    def count_lines(path: Path | str) -> int:
        """Count non-empty lines in JSONL file efficiently.

        Args:
            path: File path to count

        Returns:
            Number of non-empty lines
        """
        file_path = Path(path)
        if not file_path.exists():
            return 0

        try:
            # Fast byte-counting approach
            return file_path.read_bytes().count(b"\n")
        except OSError as e:
            logger.warning(f"Failed to count lines in {file_path}: {e}")
            return 0


class JsonlValidator:
    """Validate JSONL files for integrity and correctness.

    Features:
        - Scan for corrupted lines
        - Schema validation
        - Statistics gathering
    """

    @staticmethod
    def validate_file(
        path: Path | str,
        model: type[BaseModel] | None = None,
    ) -> dict[str, Any]:
        """Validate JSONL file and return statistics.

        Args:
            path: File path to validate
            model: Optional Pydantic model for schema validation

        Returns:
            Dictionary with validation results:
                - total_lines: Total non-empty lines
                - valid_lines: Lines that parsed successfully
                - corrupted_lines: Lines that failed to parse
                - corrupted_indices: Line numbers of corrupted entries
                - is_valid: Whether all lines are valid
        """
        file_path = Path(path)
        stats = {
            "total_lines": 0,
            "valid_lines": 0,
            "corrupted_lines": 0,
            "corrupted_indices": [],
            "is_valid": True,
        }

        if not file_path.exists():
            return stats

        try:
            with file_path.open("r", encoding="utf-8") as f:
                for line_num, line in enumerate(f, start=1):
                    text = line.strip()
                    if not text:
                        continue

                    stats["total_lines"] += 1

                    try:
                        data = json.loads(text)
                        if model:
                            model.model_validate(data)
                        stats["valid_lines"] += 1
                    except (json.JSONDecodeError, ValueError):
                        stats["corrupted_lines"] += 1
                        stats["corrupted_indices"].append(line_num)
                        stats["is_valid"] = False
        except OSError as e:
            logger.error(
                f"Failed to validate JSONL file {file_path}: {e}", exc_info=True
            )
            stats["is_valid"] = False

        return stats

    @staticmethod
    def repair_file(
        input_path: Path | str,
        output_path: Path | str,
        model: type[BaseModel] | None = None,
    ) -> dict[str, int]:
        """Create a new JSONL file with corrupted lines removed.

        Args:
            input_path: Source JSONL file
            output_path: Destination JSONL file
            model: Optional Pydantic model for validation

        Returns:
            Dictionary with repair statistics:
                - lines_read: Total lines processed
                - lines_written: Lines successfully written
                - lines_skipped: Corrupted lines removed
        """
        stats = {"lines_read": 0, "lines_written": 0, "lines_skipped": 0}

        try:
            with JsonlWriter(output_path) as writer:
                for item in JsonlReader.iterate(
                    input_path, model=model, skip_corrupted=True
                ):
                    stats["lines_read"] += 1
                    if isinstance(item, dict):
                        writer.write(item)
                    else:
                        writer.write(item.model_dump(by_alias=True))
                    stats["lines_written"] += 1
        except OSError as e:
            logger.error(f"Failed to repair JSONL file: {e}", exc_info=True)
            raise

        stats["lines_skipped"] = stats["lines_read"] - stats["lines_written"]
        return stats
