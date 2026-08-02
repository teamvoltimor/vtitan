"""TrackNavigator ROS2 node: wires CoreNavigator to ROS2HardwareGateway.

Orchestrates navigation by wiring the CoreNavigator (pure Python)
with the ROS2 ecosystem via the HardwareGateway protocol.

Usage:
    python main.py --metadata scenario_0000_metadata.json --laps 3
"""

from __future__ import annotations

import argparse
import json
import logging
import statistics
from pathlib import Path
from typing import Any, cast, override

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from shared.config.constants import CorridorDimensions, DictKeys
from shared.config.enums import Direction, ScenarioType, Section
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import RobotState
from shared.domain.models import CorridorWidthEntry, CorridorWidths, Pose, ScenarioMetadata
from std_msgs.msg import String

from src.navigation.core_navigator import CoreNavigator
from src.navigation.corridor_estimator import (
    CorridorWidthEstimator,
    measure_corridor_width,
    section_from_heading,
)
from src.navigation.corridor_follower import follow_corridor
from src.navigation.direction_estimator import DirectionEstimator
from src.navigation.maneuvers.parking import ParkController, park_controller_from_metadata
from src.navigation.planning.sign_router import SignRouter, SignRouterConfig, signs_from_metadata
from src.navigation.planning.waypoints import calculate_waypoints
from src.navigation.ports import DriveCommand
from src.navigation.race_tracker import LapDetector
from src.navigation.start_conditions import assumed_start_conditions
from src.navigation.track_geometry import TrackWalls, corridor_geometry_from_widths, corridor_widths_from_metadata
from src.ros2.navigation.ros2_hardware_gateway import ROS2HardwareGateway
from src.ros2.resettable_node import ResettableNode

logger = logging.getLogger(__name__)

_MAX_START_SAMPLES = 20
"""Length of the rolling window of stationary width readings kept before start.

Enough to outvote the narrow prior comfortably (the estimator needs
MIN_SAMPLES agreeing readings), and at the 20 Hz control rate it spans the
last second before the button is pressed. A window rather than a total,
because the robot is often powered on well away from the track and only what
it sees once placed should count.
"""


def _load_json(path: str | Path) -> dict[str, Any]:
    """Load JSON file."""
    p = Path(path) if isinstance(path, str) else path
    with p.open(encoding="utf-8") as f:
        return cast("dict[str, Any]", json.load(f))


class TrackNavigator(Node, ResettableNode):
    """ROS2 node wrapping the pure Python CoreNavigator."""

    def __init__(
        self,
        metadata_path: str | Path | None = None,
        num_laps: int = 3,
        params_path: str | Path | None = None,
        tuning_path: str | Path | None = None,
        blind: bool = False,
        direction: Direction = Direction.CLOCKWISE,
    ) -> None:
        """Drive the track.

        Args:
            metadata_path: Scenario metadata. Omit it entirely to run with no
                scenario file at all, which is what competition requires: the
                robot knows the corridor width *possibilities* (60 or 100 cm)
                but not which corridor is which, and no file describing the
                round exists on the day. Omitting it implies ``blind``.
            num_laps: Laps to complete.
            params_path: Optional JSON of ROS parameter overrides.
            tuning_path: Optional navigation tuning YAML.
            blind: Estimate the corridor layout from LIDAR instead of being
                told it. Implied when ``metadata_path`` is omitted, since there
                is then nothing to be told.
            direction: Travel direction for the round. This is the one starting
                condition that cannot be assumed -- the section can, because
                assuming it only rotates the robot's private world frame, but
                the direction is a reflection and no amount of width learning
                recovers from getting it wrong. See
                :mod:`src.navigation.start_conditions`. Ignored when
                ``metadata_path`` supplies one.
        """
        super().__init__("track_navigator")

        # No file means nothing to be sighted with.
        self._blind = blind or metadata_path is None
        self._metadata = (
            _load_json(metadata_path)
            if metadata_path is not None
            else {DictKeys.STARTING_CONDITIONS: assumed_start_conditions(direction)}
        )
        self._is_open_challenge = self._metadata.get(DictKeys.CHALLENGE_TYPE, ScenarioType.OPEN) == ScenarioType.OPEN

        # Start conditions
        start_cond = self._metadata[DictKeys.STARTING_CONDITIONS]
        start_x = start_cond[DictKeys.POSITION][DictKeys.X]
        start_y = start_cond[DictKeys.POSITION][DictKeys.Y]
        start_yaw = start_cond[DictKeys.YAW]

        # Parameters. Defaults match the topics the deployed nodes actually
        # use (ackermann_motor_node's /ackermann_cmd, sllidar_ros2's /scan) —
        # not the pre-Ackermann-migration /wro_robot/cmd_vel and /lidar names
        # this node previously assumed. There is no odom_topic: no node
        # publishes nav_msgs/Odometry on real hardware, so position comes
        # from LIDAR localization instead (see ROS2HardwareGateway).
        self.declare_parameter("ackermann_cmd_topic", "/ackermann_cmd")
        self.declare_parameter("lidar_topic", "/scan")
        self.declare_parameter("vision_topic", "/vision/detections")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("is_simulation", value=False)

        if params_path is not None:
            self._apply_param_overrides(params_path)

        # Setup Tuning
        tuning = NavigationTuning.load_from_yaml(tuning_path) if tuning_path else NavigationTuning.load_default()

        start_section = Section.from_string(start_cond[DictKeys.SECTION])
        start_direction = Direction.from_string(start_cond[DictKeys.DIRECTION])

        # What the robot is allowed to believe about the layout. Sighted runs
        # read it from the metadata; blind runs start from a prior and correct
        # it from LIDAR as they drive.
        #
        # In the OPEN Challenge narrow is the safe prior: planning a 1.0 m
        # corridor as if it were 0.6 m puts the path nearer the outer wall,
        # which is still inside it. The converse puts the path 0.15 m from the
        # inner block face, inside the chassis half-diagonal, and clips it
        # mid-turn. That argument only applies where the width is genuinely
        # unknown -- see the estimator's construction below for why the
        # Obstacles Challenge is a different case.
        self._arc_radius = tuning.waypoints.ARC_RADIUS
        self._tuning = tuning
        self._direction = start_direction
        # Kept separately from self._direction, which _commit_direction
        # overwrites once LIDAR inference settles: reset() needs the original
        # provisional value to restart from, not whatever direction the
        # previous race happened to resolve to. See reset().
        self._initial_direction = start_direction
        self._start_xy = (start_x, start_y)
        self._start_section = start_section
        # The Obstacles Challenge fixes every corridor at 1.0 m, so a blind run
        # there starts from that rather than from the Open Challenge's
        # fail-safe narrow prior -- assuming narrow is not conservative when
        # the round's rules say it cannot be true. See CorridorWidthEstimator.
        self._width_estimator = (
            CorridorWidthEstimator(
                assumed_width=CorridorDimensions.NARROW
                if self._is_open_challenge
                else CorridorDimensions.OBSTACLES_WIDTH,
            )
            if self._blind
            else None
        )
        # Blind implies inferring the direction: it is drawn at random on the
        # day, so a blind robot cannot be handed it either. ``direction`` is
        # only the provisional the first path is built from, and is replaced
        # the moment the inference settles.
        self._direction_estimator = DirectionEstimator() if self._blind else None
        self._creep_widths: list[tuple[float, float]] = []
        self._creep_speed = tuning.speed.SLOW_SPEED
        self._told_geometry = corridor_widths_from_metadata(self._metadata) if not self._blind else None
        # _told_geometry is None exactly when blind (and then _width_estimator
        # is set instead), so geometry is never actually None here -- just not
        # provable to mypy across the two separately-computed conditions.
        geometry = (
            corridor_geometry_from_widths(self._width_estimator.widths)
            if self._width_estimator
            else self._told_geometry
        )
        assert geometry is not None

        self._gateway = ROS2HardwareGateway(
            self,
            start_x,
            start_y,
            start_yaw,
            geometry,
            stale_timeout_sec=tuning.sensor.STALE_TIMEOUT_SEC,
            localization=tuning.localization,
        )
        waypoints = self._plan(self._to_widths_dict())

        sign_router: SignRouter | None = None
        if not self._is_open_challenge:
            # A blind run has no metadata file, so ``signs_from_metadata`` comes
            # back empty and the router used to be left as None — which meant
            # blind operation shipped with no sign avoidance whatsoever, the one
            # thing the Obstacles Challenge is scored on. Build it regardless and
            # let it discover the layout from ``/vision/detections``, the same
            # way ``CorridorWidthEstimator`` recovers the corridor widths.
            signs = [] if self._blind else signs_from_metadata(self._metadata)
            sign_router = SignRouter(
                signs,
                config=SignRouterConfig.from_tuning(tuning.sign_router),
                direction=start_direction,
                discover=self._blind,
                discovery_config=tuning.sign_discovery,
            )

        lap_detector = LapDetector(
            start_pos=(start_x, start_y),
            start_section=start_section,
            direction=start_direction,
        )

        park_controller: ParkController | None = None
        if not self._is_open_challenge:
            park_controller = park_controller_from_metadata(
                self._metadata,
                start_section,
                start_direction,
                tuning=tuning,
            )

        self._core_navigator = CoreNavigator(
            gateway=self._gateway,
            waypoints=waypoints,
            num_laps=num_laps,
            tuning=tuning,
            sign_router=sign_router,
            lap_detector=lap_detector,
            park_controller=park_controller,
        )

        # Race-state gate. Without this the navigator drives the moment it has a
        # pose -- before the start button is pressed, and straight through an
        # E-STOP, since stopping the state machine does not stop this node. The
        # button is the operator's only physical control, so it has to gate the
        # thing that actually moves the robot.
        #
        # Both policies must match state_machine_node's _QOS_TRANSIENT publisher
        # or this subscription silently receives nothing at all.
        #
        # TRANSIENT_LOCAL so the current state arrives immediately rather than
        # only on the next transition -- otherwise launching mid-race would sit
        # idle until the state happened to change.
        #
        # BEST_EFFORT, not RELIABLE: a RELIABLE reader is incompatible with that
        # BEST_EFFORT writer, and DDS resolves the mismatch by never delivering.
        # This was live on hardware -- both this node and bag_recorder logged
        # "offering incompatible QoS. No messages will be received", meaning the
        # navigator could never observe RACING and the robot would never have
        # driven. The publisher is deliberately BEST_EFFORT (it must not block on
        # the Pi Zero's OLED, which stalls for 30+ s), so the reader is what has
        # to give. Losing reliability costs nothing here: _publish_state runs
        # every tick of the state machine loop, not only on transitions, so a
        # dropped sample is corrected within one tick.
        self._racing = False
        self.create_subscription(
            String,
            "/robot_state",
            self._on_robot_state,
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        # Control Loop
        self.create_timer(0.05, self._control_loop)  # 20 Hz

        self.get_logger().info(
            f"Navigator ready: {len(waypoints)} waypoints, {num_laps} lap(s) - "
            "holding until /robot_state reports racing",
        )

        # A blind round has to *assume* where on the track it is standing, and
        # nothing until now reported what it assumed. That matters because the
        # failure it produces looks like a steering fault rather than a
        # localisation one: if the assumed corridor is not the real one, the
        # first waypoints sit behind the robot, and the only way to reach
        # something behind you is to turn around. Observed on the track as a
        # U-turn followed by a lap driven backwards -- with the servo, the IMU,
        # the LIDAR bearing, the gateway and the inferred direction all checked
        # and correct.
        head = waypoints[:3]
        self.get_logger().info(
            f"Assumed start: section={start_section.value} direction={start_direction.value} "
            f"pose=({start_x:.2f}, {start_y:.2f}) - first waypoints "
            + ", ".join(f"({x:.2f}, {y:.2f})" for x, y in head),
        )

    def _resolve_direction(self) -> bool:
        """Creep along the corridor until the travel direction is inferable.

        Returns:
            ``True`` while the direction is still unknown, meaning this tick
            was driven by the corridor follower and there is no plan to step.
        """
        estimator = self._direction_estimator
        if estimator is None or estimator.is_settled:
            return False

        scan = self._gateway.get_lidar_scan()
        pose = self._gateway.get_current_pose()
        if scan is None or pose is None:
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))
            return True

        # Width readings taken now cannot be filed under a corridor yet -- that
        # needs the direction -- but they are the cleanest of the round, taken
        # driving straight down a corridor. Buffer and replay them, or the
        # first surviving readings are taken at a corner where the side rays
        # span the *next* corridor and get attributed to this one.
        if self._width_estimator is not None:
            m = measure_corridor_width(scan.ranges_m, scan.angles_rad, pose.yaw)
            if m is not None:
                self._creep_widths.append((pose.yaw, m.width_m))

        if estimator.observe(scan.ranges_m, scan.angles_rad, pose.yaw):
            inferred = estimator.direction
            if inferred is not None:
                self._commit_direction(inferred, pose)
            return False

        self._gateway.publish_drive(follow_corridor(scan.ranges_m, scan.angles_rad, self._creep_speed))
        return True

    def _sample_start_corridor(self) -> None:
        """Measure the starting corridor while the robot is still stationary.

        The width estimator starts from a narrow (60 cm) prior and only leaves
        it after repeated agreeing measurements. In a 100 cm corridor that prior
        is self-reinforcing on real hardware, and measurably so: believing the
        corridor is 60 cm wide while sitting centred in a 100 cm one makes the
        robot think it is badly off-centre, so it saturates steering to correct
        toward a centre that is not there, ends up skewed against a wall, and
        from that pose ``measure_corridor_width`` returns None -- so the belief
        that caused the pose can never be corrected by it. Measured on the
        track: 390 consecutive drive commands at full left lock, ending 14 cm
        from a wall, with the belief still reading 60 cm.

        The one moment the robot is guaranteed to be well placed is before it
        has moved: an operator sets it down centred and square in a corridor.
        That is exactly the geometry the measurement needs -- verified on
        hardware, the same function returns None when the robot is skewed 24
        degrees against a wall and 0.97 m when it is squarely placed in the same
        1 m corridor.

        Readings go into the same buffer the creep phase uses, so they are
        attributed to a section by the existing replay in _commit_direction
        once the travel direction is known. Nothing here needs to know which
        corridor it is sitting in.
        """
        if self._width_estimator is None:
            return
        scan = self._gateway.get_lidar_scan()
        pose = self._gateway.get_current_pose()
        if scan is None or pose is None:
            return
        m = measure_corridor_width(scan.ranges_m, scan.angles_rad, pose.yaw)
        if m is None:
            return

        # A rolling window, not a fill-once buffer. The robot is routinely
        # powered on somewhere other than the track -- a bench, a table, the
        # floor beside it -- and readings taken there describe a room, not a
        # corridor. Keeping the first N would mean whatever it happened to see
        # at power-on decides the layout, and no amount of correctly placing it
        # afterwards could displace them. What matters is the last second
        # before the operator presses start, so old readings age out.
        self._creep_widths.append((pose.yaw, m.width_m))
        if len(self._creep_widths) > _MAX_START_SAMPLES:
            del self._creep_widths[0]

        widths = [w for _, w in self._creep_widths]
        self.get_logger().info(
            f"Start corridor reads {statistics.fmean(widths):.2f}m over the last "
            f"{len(widths)} samples - this is what the layout belief will start from",
            throttle_duration_sec=5.0,
        )

    def _to_widths_dict(self) -> dict[Section, float]:
        """Current believed widths as a per-section dict (for _plan)."""
        if self._width_estimator:
            return self._width_estimator.widths
        g = self._told_geometry
        # Set exactly when not blind (see __init__), which is the only way to
        # reach this branch -- blind means _width_estimator is set instead.
        assert g is not None
        return {
            Section.NORTH: g.north_width_m,
            Section.SOUTH: g.south_width_m,
            Section.EAST: g.east_width_m,
            Section.WEST: g.west_width_m,
        }

    def _commit_direction(self, inferred: Direction, pose: Pose) -> None:
        """Adopt the inferred direction and rebuild everything derived from it."""
        changed = inferred is not self._direction
        self._direction = inferred
        if self._width_estimator is not None:
            for buffered_yaw, buffered_width in self._creep_widths:
                self._width_estimator.observe_measurement(
                    section_from_heading(buffered_yaw, inferred),
                    buffered_width,
                )
            self._creep_widths.clear()
            self._gateway.set_believed_walls(TrackWalls(corridor_geometry_from_widths(self._width_estimator.widths)))
        if changed:
            # The finish line's normal is the travel direction, so a detector
            # built for the provisional one counts crossings inverted.
            self._core_navigator.replace_lap_detector(
                LapDetector(
                    start_pos=(self._start_xy),
                    start_section=self._start_section,
                    direction=inferred,
                ),
            )
        # Resync unconditionally: the navigator did not step during the creep,
        # so its waypoint index is still 0 while the robot has driven a metre
        # past it, and it would resume by chasing a waypoint behind itself.
        self._core_navigator.replace_path(self._plan(self._to_widths_dict()), (pose.x, pose.y))
        self.get_logger().info(f"Travel direction inferred from LIDAR: {inferred}")

    def _plan(self, widths: dict[Section, float]) -> list[tuple[float, float]]:
        """Build a one-lap path for the layout the robot believes it is on.

        num_laps=1 is intentional: calculate_waypoints bakes the lap count into
        the list, but CoreNavigator already cycles one canonical lap `num_laps`
        times (see step() waypoint-wrap). Passing the real count would multiply
        laps (e.g. 3 -> 9). Keep this at 1.
        """
        # self._metadata is a plain dict (from _load_json, or the assumed-start
        # fallback literal) everywhere else in this class -- never actually a
        # ScenarioMetadata. Validating it here (rather than just annotating it
        # as one) is what the .model_copy() calls below need to not crash with
        # AttributeError: 'dict' object has no attribute 'starting_conditions'.
        metadata = ScenarioMetadata.model_validate(self._metadata)
        new_widths = CorridorWidths(
            **{s.value: CorridorWidthEntry(width_mm=round(width * 1000)) for s, width in widths.items()},
        )
        new_starting = metadata.starting_conditions.model_copy(
            update={"direction": str(self._direction)},
        )
        planning_metadata = metadata.model_copy(
            update={
                "corridor_widths": new_widths,
                "starting_conditions": new_starting,
            },
        )
        return calculate_waypoints(planning_metadata, num_laps=1, arc_radius=self._arc_radius)

    def _update_layout_belief(self) -> bool:
        """Fold the latest scan into the width estimate; replan if it moved.

        Returns:
            ``True`` if the belief changed and the path was rebuilt.
        """
        estimator = self._width_estimator
        if estimator is None:
            return False
        scan = self._gateway.get_lidar_scan()
        pose = self._gateway.get_current_pose()
        if scan is None or pose is None:
            return False

        # Attribute the reading by HEADING, not position. Position would be
        # circular -- it comes from matching against a wall model built from the
        # very widths being estimated, so a wrong belief mis-attributes the
        # reading that would have corrected it and the error locks in. Heading
        # comes from the IMU and owes nothing to the map.
        section = section_from_heading(pose.yaw, self._direction)
        if not estimator.observe(section, scan.ranges_m, scan.angles_rad, pose.yaw):
            return False

        believed = estimator.widths
        self._gateway.set_believed_walls(TrackWalls(corridor_geometry_from_widths(believed)))
        self._core_navigator.replace_path(self._plan(believed), (pose.x, pose.y))
        self.get_logger().info(
            "Layout belief updated: "
            + ", ".join(f"{s.value}={w * 100:.0f}cm" for s, w in sorted(believed.items(), key=lambda kv: kv[0].value)),
        )
        return True

    def _on_robot_state(self, msg: String) -> None:
        """Track whether the state machine says we are racing."""
        was_racing = self._racing
        self._racing = msg.data.strip().lower() == RobotState.RACING.value
        if was_racing and not self._racing:
            # Left RACING (finished, or E-STOP). Command a stop immediately
            # rather than waiting for the next control tick.
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))
            self.get_logger().info(f"Race state '{msg.data}' - navigator holding, motors stopped")
        elif not was_racing and self._racing:
            # This is the one instant the robot is known to be in its
            # starting pose -- whether that's the very first race, or a
            # re-run cycled purely from the button (FINISHED -> BOOT_CHECK ->
            # READY -> RACING, no process restart), so reset() has to run
            # here every time, not just once at node startup.
            self.reset()
            self.get_logger().info("Race started - heading reference zeroed, navigator driving")

    @override
    def reset(self) -> None:
        """Clear this node's race-scoped state ahead of a new race.

        Re-zeros the heading reference (RVC yaw is relative to power-on, and
        the robot is carried to the track after that, so the offset latched
        by the first IMU reading refers to whatever orientation it happened
        to be held in) and clears CoreNavigator's own per-race state. The
        ParkController can't be rewound once its phase reaches DONE, so a
        fresh one is built the same way the first one was, from the same
        section/direction/metadata.

        A blind round also rebuilds the width and direction estimators here.
        The state machine can cycle FINISHED -> BOOT_CHECK -> READY -> RACING
        purely from the button, with no process restart, so without this a
        re-run inherits the previous race's learned corridor widths and
        resolved travel direction: _direction_estimator.is_settled stays True
        forever once the first race infers a direction, so _resolve_direction
        would short-circuit and the robot would drive the new race believing
        it is still going whichever way the last one went -- observed as
        "it keeps the old track" and, if the new race is actually the other
        direction, as steering the wrong way from the first waypoint.
        """
        self._gateway.reset_heading_reference()
        # Re-stamp anything measured before the start to the heading frame that
        # reset just established. The yaws recorded against the old reference
        # would otherwise file those readings under the wrong section, since
        # _commit_direction attributes them by heading. Zero is correct rather
        # than approximate: this method runs at the one instant the robot is
        # known to be sitting at its starting pose, so "the heading it has now"
        # and "the heading those readings were taken at" are the same.
        self._creep_widths = [(0.0, width) for _, width in self._creep_widths]

        self._direction = self._initial_direction
        if self._blind:
            self._width_estimator = CorridorWidthEstimator(
                assumed_width=CorridorDimensions.NARROW
                if self._is_open_challenge
                else CorridorDimensions.OBSTACLES_WIDTH,
            )
            self._direction_estimator = DirectionEstimator()
            self._gateway.set_believed_walls(TrackWalls(corridor_geometry_from_widths(self._width_estimator.widths)))

        park_controller: ParkController | None = None
        if not self._is_open_challenge:
            park_controller = park_controller_from_metadata(
                self._metadata,
                self._start_section,
                self._direction,
                tuning=self._tuning,
            )
        self._core_navigator.replace_park_controller(park_controller)
        # The previous race's lap detector counts crossings against whatever
        # direction it resolved to, and carries a pending-waypoint flag from
        # wherever the robot last was on the loop -- neither belongs to a
        # race that hasn't started yet.
        self._core_navigator.replace_lap_detector(
            LapDetector(
                start_pos=self._start_xy,
                start_section=self._start_section,
                direction=self._direction,
            ),
        )
        self._core_navigator.replace_path(self._plan(self._to_widths_dict()), self._start_xy)
        self._core_navigator.reset()

    def _control_loop(self) -> None:
        """Execute one control step, or hold the robot stopped when not racing."""
        if not self._racing:
            # Keep publishing zeros rather than going silent: ackermann_motor_node
            # has a 1 s command watchdog, and silence would let it latch a stop
            # only after that delay.
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))
            self._sample_start_corridor()
            return
        try:
            if self._resolve_direction():
                # Direction unknown: the corridor follower drove this tick and
                # there is no usable plan to step yet.
                return
            if self._blind:
                self._update_layout_belief()
            self._core_navigator.step()
        except RuntimeError as e:
            self.get_logger().error(f"Runtime error in control loop: {e}")
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))
        except ValueError as e:
            self.get_logger().error(f"Value error in control loop: {e}")
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))

    def _apply_param_overrides(self, params_path: str | Path) -> None:
        """Load a JSON file of {param_name: value} overrides and apply to this node."""
        import rclpy.parameter as rp  # noqa: PLC0415

        try:
            data = _load_json(params_path)
        except FileNotFoundError as e:
            self.get_logger().error(f"Param file not found: {e}")
            return
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode params JSON: {e}")
            return

        params = []
        for name, value in data.items():
            if isinstance(value, bool):
                params.append(rp.Parameter(name, rp.Parameter.Type.BOOL, value))
            elif isinstance(value, int):
                # rclpy's Parameter stub only resolves the bool overload;
                # runtime dispatch is on the Type enum, not the stub's
                # positional-arg overload, so this is a stub limitation.
                params.append(rp.Parameter(name, rp.Parameter.Type.INTEGER, value))  # type: ignore[arg-type]
            elif isinstance(value, float):
                params.append(rp.Parameter(name, rp.Parameter.Type.DOUBLE, value))  # type: ignore[arg-type]
            elif isinstance(value, str):
                params.append(rp.Parameter(name, rp.Parameter.Type.STRING, value))  # type: ignore[arg-type]
            else:
                self.get_logger().warning(f"Skipping param '{name}': unsupported type {type(value)}")

        if params:
            self.set_parameters(params)
            self.get_logger().info(f"Applied {len(params)} param override(s) from {params_path}")


def main(args: list[str] | None = None) -> None:
    """Run the ROS2 track navigator node (``ros2 run voldemorbot_navigation track_navigator_node``)."""
    parser = argparse.ArgumentParser(description="WRO 2026 track navigator ROS2 node.")
    parser.add_argument(
        "--metadata",
        help="Path to scenario metadata JSON. Omit to run with no scenario file at "
        "all, which is what competition requires -- the layout is then estimated "
        "from LIDAR and --direction supplies the only start condition that cannot "
        "be assumed.",
    )
    parser.add_argument("--laps", type=int, default=3, help="Laps to complete (default: 3).")
    parser.add_argument("--params", help="Optional navigator_params.json for runtime overrides.")
    parser.add_argument("--tuning", help="Optional navigation tuning YAML.")
    parser.add_argument(
        "--blind",
        action="store_true",
        help="Estimate the corridor layout from LIDAR instead of reading it from "
        "--metadata, which is what competition requires: WRO randomises the inner "
        "walls before each round, so the widths in a file cannot be known on the "
        "day. Only the start conditions are then read from --metadata.",
    )
    parser.add_argument(
        "--direction",
        choices=["cw", "ccw"],
        default="cw",
        help="Travel direction for the round (default: cw). Used only when --metadata "
        "is omitted. The starting section does not need one of these because assuming "
        "it merely rotates the robot's own world frame; the direction is a reflection "
        "and has to be right.",
    )
    parsed, _ = parser.parse_known_args(args)

    metadata_path: Path | None = None
    if parsed.metadata:
        metadata_path = Path(parsed.metadata)
        if not metadata_path.exists():
            logger.error("Metadata file not found: %s", metadata_path)
            raise SystemExit(1)

    rclpy.init(args=args)
    navigator: TrackNavigator | None = None
    try:
        navigator = TrackNavigator(
            metadata_path=metadata_path,
            num_laps=parsed.laps,
            params_path=parsed.params,
            tuning_path=parsed.tuning,
            blind=parsed.blind,
            direction=Direction.CLOCKWISE if parsed.direction == "cw" else Direction.COUNTERCLOCKWISE,
        )
        while rclpy.ok() and not getattr(navigator, "shutdown_requested", False):
            rclpy.spin_once(navigator, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if navigator is not None:
            navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
