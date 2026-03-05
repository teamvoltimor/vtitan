"""Klevor robot — WRO 2026 track navigator."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

import rclpy

from src.navigation.navigator import TrackNavigator

logger = logging.getLogger(__name__)


def main() -> None:
    """Parse arguments and run the TrackNavigator node."""
    parser = argparse.ArgumentParser(
        description="Navigate the WRO robot using waypoint following.",
    )
    parser.add_argument(
        "--metadata",
        required=True,
        help="Path to the scenario metadata JSON file.",
    )
    parser.add_argument(
        "--laps",
        type=int,
        default=3,
        help="Number of laps to complete (default: 3).",
    )
    parser.add_argument(
        "--params",
        help="Optional path to navigator_params.json for runtime overrides.",
    )
    args = parser.parse_args()

    metadata_path = Path(args.metadata)
    if not metadata_path.exists():
        logger.error("Metadata file not found: %s", metadata_path)
        raise SystemExit(1)

    rclpy.init()
    navigator: TrackNavigator | None = None
    try:
        navigator = TrackNavigator(
            metadata_path=metadata_path,
            num_laps=args.laps,
            params_path=args.params,
        )
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        if navigator is not None:
            navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
