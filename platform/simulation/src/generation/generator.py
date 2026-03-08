"""ScenarioGenerator: orchestrates randomization and SDF world building.

Entry point for the training data generation pipeline.

Usage:
    python -m src.generation.generator --challenge open --num-scenarios 10
    python -m src.generation.generator --challenge obstacles --num-scenarios 50 --randomize-all
"""

from __future__ import annotations

import importlib
import importlib.util
import json
import logging
import math
from pathlib import Path
from types import ModuleType
from typing import Any
from xml.etree.ElementTree import Element, ElementTree

from defusedxml.ElementTree import parse as defused_parse

from src.config.constants import (
    ColorNames,
    CorridorDimensions,
    DictKeys,
    FileExtensions,
    FilePaths,
    FolderNames,
    GridSections,
    TrackDimensions,
    TrafficSignSpecs,
    WidthTypes,
)
from src.config.enums import Direction, ScenarioType, Section
from src.generation.randomizer import ScenarioRandomizer
from src.generation.sdf_builder import SDFBuilder

logger = logging.getLogger(__name__)

cv2: ModuleType | None = None
CvBridge: type[Any] | None = None
ROS2_AVAILABLE = False
if importlib.util.find_spec("cv2") and importlib.util.find_spec("cv_bridge"):
    cv2 = importlib.import_module("cv2")
    CvBridge = importlib.import_module("cv_bridge").CvBridge
    ROS2_AVAILABLE = True
else:
    logger.warning("ROS2 not available — running in standalone mode")


class ScenarioGenerator:
    """Orchestrates scenario randomization, SDF world building, and metadata output.

    Args:
        base_world_path: Path to the base WRO track SDF template.
        output_dir: Directory to write generated scenario files.
        challenge_type: ScenarioType.OPEN or ScenarioType.OBSTACLES.
    """

    def __init__(
        self,
        base_world_path: str,
        output_dir: str | Path,
        challenge_type: ScenarioType = ScenarioType.OPEN,
    ) -> None:
        self._base_world_path = base_world_path
        self._output_dir = Path(output_dir)
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._challenge_type = challenge_type

        self._randomizer = ScenarioRandomizer(_build_randomization_config())
        self._builder = SDFBuilder(challenge_type)

    def create_scenario_world(
        self,
        scenario_index: int,
        randomize_all: bool = True,
    ) -> tuple[Path, dict[str, Any]]:
        """Generate one randomized SDF world file and its metadata JSON.

        Args:
            scenario_index: Zero-based scenario counter used for filenames.
            randomize_all: If True, randomize lighting, widths, and starting
                position. If False, use deterministic defaults.

        Returns:
            Tuple of (world_file_path, metadata_dict).
        """
        tree = defused_parse(self._base_world_path)
        root: Element = tree.getroot()
        world = root.find("world")
        if world is None:
            raise ValueError("Base world SDF is missing the <world> element")

        self._builder.add_system_plugins(world)

        corridor_widths = self._resolve_corridor_widths(randomize_all)

        if randomize_all:
            lighting = self._randomizer.randomize_lighting()
            self._builder.apply_lighting(world, lighting)

        starting_conditions = self._resolve_starting_conditions(randomize_all, corridor_widths)

        (
            sign_positions,
            sign_colors,
            parking_config,
        ) = self._resolve_obstacles(corridor_widths, starting_conditions)

        self._builder.add_interior_walls(world, corridor_widths)
        self._builder.add_traffic_signs(world, sign_positions, sign_colors)

        if parking_config is not None:
            self._builder.add_parking_lot(world, parking_config)

        self._builder.add_starting_zone(
            world,
            starting_conditions,
            corridor_widths,
            parking_config,
            self._base_world_path,
        )
        self._builder.add_robot_model(world, starting_conditions)

        world_file = self._save_world(tree, scenario_index)
        metadata = self._build_metadata(
            scenario_index,
            corridor_widths,
            starting_conditions,
            sign_positions,
            sign_colors,
            parking_config,
        )
        self._save_metadata(metadata, scenario_index)

        return world_file, metadata

    # Private helpers
    def _resolve_corridor_widths(
        self,
        randomize_all: bool,
    ) -> dict[Section, dict[str, Any]]:
        if self._challenge_type == ScenarioType.OBSTACLES:
            # Obstacles: fixed 1.0 m corridor on all sides
            return {
                s: {
                    DictKeys.TYPE: WidthTypes.FIXED,
                    DictKeys.WIDTH: 1.0,
                }
                for s in GridSections.SECTIONS
            }
        if randomize_all:
            return self._randomizer.randomize_corridor_widths()
        # Open with no randomization: default wide corridors
        return {
            s: {
                DictKeys.TYPE: WidthTypes.WIDE,
                DictKeys.WIDTH: CorridorDimensions.WIDE,
            }
            for s in GridSections.SECTIONS
        }

    def _resolve_starting_conditions(
        self,
        randomize_all: bool,
        corridor_widths: dict[Section, dict[str, Any]],
    ) -> dict[str, Any]:
        if randomize_all:
            return self._randomizer.randomize_starting_conditions(corridor_widths)
        return {
            DictKeys.DIRECTION: Direction.CLOCKWISE,
            DictKeys.SECTION: Section.SOUTH,
            DictKeys.SECTION_NAME: "South",
            DictKeys.POSITION: (1.5, 0.4),
            DictKeys.YAW: math.pi,
        }

    def _resolve_obstacles(
        self,
        corridor_widths: dict[Section, dict[str, Any]],
        starting_conditions: dict[str, Any],
    ) -> tuple[
        list[tuple[float, float]],
        list[tuple[str, list[float]]],
        dict[str, Any] | None,
    ]:
        if self._challenge_type != ScenarioType.OBSTACLES:
            return [], [], None

        starting_section: Section = starting_conditions[DictKeys.SECTION]
        sign_positions, sign_colors = self._randomizer.generate_sign_positions(
            corridor_widths,
            exclude_section=starting_section,
        )
        parking_config = self._randomizer.generate_parking_lot_positions(starting_section)
        sign_positions, sign_colors = _adjust_signs_for_parking(
            sign_positions, sign_colors, starting_section, parking_config
        )
        return sign_positions, sign_colors, parking_config

    def _save_world(self, tree: ElementTree, scenario_index: int) -> Path:
        world_file = (
            self._output_dir
            / f"{FilePaths.SCENARIO_PREFIX}{scenario_index:04d}{FileExtensions.SDF}"
        )
        tree.write(world_file, encoding="utf-8", xml_declaration=True)
        return world_file

    def _save_metadata(self, metadata: dict[str, Any], scenario_index: int) -> None:
        metadata_file = (
            self._output_dir
            / f"{FilePaths.SCENARIO_PREFIX}{scenario_index:04d}{FilePaths.METADATA_SUFFIX}"
        )
        with metadata_file.open("w", encoding="utf-8") as fh:
            json.dump(metadata, fh, indent=2)

    def _build_metadata(
        self,
        scenario_index: int,
        corridor_widths: dict[Section, dict[str, Any]],
        starting_conditions: dict[str, Any],
        sign_positions: list[tuple[float, float]],
        sign_colors: list[tuple[str, list[float]]],
        parking_config: dict[str, Any] | None,
    ) -> dict[str, Any]:
        return {
            DictKeys.SCENARIO_ID: scenario_index,
            DictKeys.CHALLENGE_TYPE: self._challenge_type,
            DictKeys.CORRIDOR_WIDTHS: {
                str(section): {
                    DictKeys.TYPE: corridor_widths[section][DictKeys.TYPE],
                    DictKeys.WIDTH_MM: int(corridor_widths[section][DictKeys.WIDTH] * 1000),
                }
                for section in GridSections.SECTIONS
            },
            DictKeys.STARTING_CONDITIONS: {
                DictKeys.DIRECTION: str(starting_conditions[DictKeys.DIRECTION]),
                DictKeys.SECTION: starting_conditions[DictKeys.SECTION_NAME],
                DictKeys.POSITION: {
                    DictKeys.X: starting_conditions[DictKeys.POSITION][0],
                    DictKeys.Y: starting_conditions[DictKeys.POSITION][1],
                },
                DictKeys.YAW: starting_conditions[DictKeys.YAW],
            },
            DictKeys.NUM_SIGNS: len(sign_positions),
            DictKeys.HAS_PARKING_LOT: self._challenge_type == ScenarioType.OBSTACLES,
            DictKeys.SIGN_POSITIONS: [
                {DictKeys.X: x, DictKeys.Y: y, DictKeys.COLOR: color}
                for (x, y), (color, _) in zip(sign_positions, sign_colors, strict=True)
            ],
            DictKeys.PARKING_LOT: _serialize_parking(parking_config),
        }


class VideoRecorder:
    """Records camera frames from a ROS2 topic and saves as MP4.

    Args:
        output_dir: Directory for the output video file.
        scenario_index: Used to name the output video.
    """

    def __init__(self, output_dir: str | Path, scenario_index: int) -> None:
        if not ROS2_AVAILABLE or cv2 is None or CvBridge is None:
            raise RuntimeError("VideoRecorder requires ROS2 and cv2.")
        self._output_dir = Path(output_dir)
        self._scenario_index = scenario_index
        self._bridge = CvBridge()
        self._frames: list[Any] = []
        self.is_recording = False

    def start(self) -> None:
        """Begin accumulating frames."""
        self._frames.clear()
        self.is_recording = True

    def handle_image(self, msg: Any) -> None:
        """ROS2 image callback — appends the decoded frame if recording."""
        if not self.is_recording:
            return
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._frames.append(frame)

    def stop_and_save(self) -> Path | None:
        """Stop recording and write frames to an MP4 file.

        Returns:
            Path to the saved video, or None if no frames were captured.
        """
        self.is_recording = False
        if not self._frames:
            return None

        video_file = self._output_dir / (
            f"{FilePaths.SCENARIO_PREFIX}{self._scenario_index:04d}{FileExtensions.MP4}"
        )
        height, width = self._frames[0].shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(video_file), fourcc, 30.0, (width, height))
        try:
            for frame in self._frames:
                writer.write(frame)
        finally:
            writer.release()
        return video_file


# Private helpers
def _build_randomization_config() -> dict[str, Any]:
    return {
        DictKeys.COLORS: {
            ColorNames.RED: {
                DictKeys.MEAN: list(TrafficSignSpecs.RED_COLOR),
                DictKeys.STD: list(TrafficSignSpecs.RED_STD),
            },
            ColorNames.GREEN: {
                DictKeys.MEAN: list(TrafficSignSpecs.GREEN_COLOR),
                DictKeys.STD: list(TrafficSignSpecs.GREEN_STD),
            },
        },
    }


def _adjust_signs_for_parking(
    sign_positions: list[tuple[float, float]],
    sign_colors: list[tuple[str, list[float]]],
    starting_section: Section,
    parking_config: dict[str, Any],
) -> tuple[list[tuple[float, float]], list[tuple[str, list[float]]]]:
    """Move outer-position signs away from parking blocks to avoid collision."""
    outer_pos = TrafficSignSpecs.GRID_WIDTH_OUTER
    inner_pos = TrafficSignSpecs.GRID_WIDTH_INNER
    tolerance = 0.05

    adjusted_positions: list[tuple[float, float]] = []
    for x, y in sign_positions:
        adjusted_x, adjusted_y = x, y
        if starting_section is Section.SOUTH and 1.0 <= x <= 2.0:
            if abs(y - outer_pos) < tolerance:
                adjusted_y = inner_pos
        elif starting_section is Section.NORTH and 1.0 <= x <= 2.0:
            if abs(y - (TrackDimensions.MAX_COORD - outer_pos)) < tolerance:
                adjusted_y = TrackDimensions.MAX_COORD - inner_pos
        elif starting_section is Section.EAST and 1.0 <= y <= 2.0:
            if abs(x - (TrackDimensions.MAX_COORD - outer_pos)) < tolerance:
                adjusted_x = TrackDimensions.MAX_COORD - inner_pos
        elif starting_section is Section.WEST and 1.0 <= y <= 2.0:
            if abs(x - outer_pos) < tolerance:
                adjusted_x = inner_pos
        adjusted_positions.append((adjusted_x, adjusted_y))

    return adjusted_positions, sign_colors


def _serialize_parking(parking_config: dict[str, Any] | None) -> dict[str, Any] | None:
    if parking_config is None:
        return None
    return {
        DictKeys.BLOCK1_POSITION: {
            DictKeys.X: parking_config[DictKeys.BLOCK1_POS][0],
            DictKeys.Y: parking_config[DictKeys.BLOCK1_POS][1],
        },
        DictKeys.BLOCK2_POSITION: {
            DictKeys.X: parking_config[DictKeys.BLOCK2_POS][0],
            DictKeys.Y: parking_config[DictKeys.BLOCK2_POS][1],
        },
        DictKeys.DEPTH: parking_config[DictKeys.DEPTH],
    }


