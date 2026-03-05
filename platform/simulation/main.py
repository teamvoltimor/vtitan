"""Klevor simulation — scenario and track generation."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

_PLATFORM_DIR = Path(__file__).parent.parent
_DEFAULT_TRAINING_DATA = str(_PLATFORM_DIR / "training_data")


def main() -> None:
    """CLI entry point for simulation generation tasks."""
    parser = argparse.ArgumentParser(
        description="WRO 2026 simulation generation tools.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    # generate subcommand
    gen = subparsers.add_parser("generate", help="Generate randomized scenario SDF files.")
    gen.add_argument(
        "--challenge",
        choices=["open", "obstacles"],
        default="open",
        help="Challenge type (default: open).",
    )
    gen.add_argument(
        "--num-scenarios",
        type=int,
        default=10,
        help="Number of scenarios to generate (default: 10).",
    )
    gen.add_argument(
        "--output-dir",
        default=_DEFAULT_TRAINING_DATA,
        help="Root output directory (default: ../training_data relative to platform/).",
    )
    gen.add_argument(
        "--base-world",
        default="./worlds/wro_track_2026.sdf",
        help="Base world SDF template path.",
    )
    gen.add_argument(
        "--randomize-all",
        action="store_true",
        help="Enable full randomization (lighting, widths, starting position).",
    )

    # generate-track subcommand
    track = subparsers.add_parser("generate-track", help="Generate the base track SDF.")
    track.add_argument(
        "--output",
        default="./worlds/wro_track_2026.sdf",
        help="Output SDF file path (default: ./worlds/wro_track_2026.sdf).",
    )

    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    logger = logging.getLogger(__name__)

    if args.command == "generate":
        from src.config.constants import DictKeys, FolderNames
        from src.config.enums import ScenarioType
        from src.generation.generator import ScenarioGenerator

        challenge_output_dir = Path(args.output_dir) / args.challenge / FolderNames.SCENARIOS
        generator = ScenarioGenerator(
            base_world_path=args.base_world,
            output_dir=challenge_output_dir,
            challenge_type=ScenarioType(args.challenge),
        )
        logger.info(
            "Generating %d '%s' scenarios → %s",
            args.num_scenarios,
            args.challenge,
            challenge_output_dir,
        )
        for index in range(args.num_scenarios):
            world_file, metadata = generator.create_scenario_world(
                index, randomize_all=args.randomize_all
            )
            logger.info(
                "[%d/%d] %s  signs=%d",
                index + 1,
                args.num_scenarios,
                world_file.name,
                metadata[DictKeys.NUM_SIGNS],
            )
        logger.info("Done.")

    elif args.command == "generate-track":
        from src.generation.track_generator import generate_track_sdf

        output_path = generate_track_sdf(args.output)
        logger.info("Track SDF written → %s", output_path)


if __name__ == "__main__":
    main()
