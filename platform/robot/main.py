"""Klevor robot — WRO 2026 navigator and driver entrypoints."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

import rclpy

logger = logging.getLogger(__name__)


def _run_navigate(args: argparse.Namespace) -> None:
    from src.navigation.navigator import TrackNavigator

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
        while rclpy.ok() and not getattr(navigator, "shutdown_requested", False):
            rclpy.spin_once(navigator, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if navigator is not None:
            navigator.destroy_node()
        rclpy.shutdown()


def _run_drive(args: argparse.Namespace) -> None:
    from src.navigation.driver import SimpleRobotDriver

    rclpy.init()
    driver: SimpleRobotDriver | None = None
    try:
        driver = SimpleRobotDriver(direction=args.direction, duration=args.duration)
        while rclpy.ok() and not getattr(driver, "shutdown_requested", False):
            rclpy.spin_once(driver, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if driver is not None:
            try:
                driver.destroy_node()
            except RuntimeError:
                logger.warning("Exception during node teardown", exc_info=True)
        try:
            rclpy.shutdown()
        except RuntimeError:
            logger.warning("Exception during rclpy shutdown", exc_info=True)


def main() -> None:
    """CLI entrypoint for the robot — navigate or drive."""
    parser = argparse.ArgumentParser(description="Klevor robot — WRO 2026.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    # navigate subcommand
    nav = subparsers.add_parser("navigate", help="Waypoint-following track navigator.")
    nav.add_argument("--metadata", required=True, help="Path to scenario metadata JSON.")
    nav.add_argument("--laps", type=int, default=3, help="Laps to complete (default: 3).")
    nav.add_argument("--params", help="Optional navigator_params.json for runtime overrides.")

    # drive subcommand
    drv = subparsers.add_parser("drive", help="Simple timed driver for video recording.")
    drv.add_argument(
        "--direction",
        choices=["clockwise", "counterclockwise"],
        default="clockwise",
        help="Direction around the track (default: clockwise).",
    )
    drv.add_argument("--duration", type=int, default=30, help="Drive duration in seconds (default: 30).")

    args = parser.parse_args()

    if args.command == "navigate":
        _run_navigate(args)
    elif args.command == "drive":
        _run_drive(args)


if __name__ == "__main__":
    main()
