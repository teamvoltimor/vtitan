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
import math
import statistics
from pathlib import Path
from typing import Any, cast, override

import rclpy
from rclpy.node import Node
from shared.config.constants import CompetitionSpecs, CorridorDimensions, DictKeys
from shared.config.navigation_tuning import NavigationTuning
from shared.config.ros_topics import RosTopicConfig
from shared.domain.enums import Direction, NavigatorPhase, RobotState, ScenarioType, Section
from shared.domain.models import (
    NavigatorDebugSnapshot,
    Pose,
    ScenarioMetadata,
    SignColor,
    TrafficSignObservation,
    Waypoint,
)
from std_msgs.msg import Int32, String

from src.config.tuning_helpers import get_tuning
from src.navigation.core_navigator import CoreNavigator
from src.navigation.corridor_estimator import (
    CorridorWidthEstimator,
    measure_corridor_width,
    section_from_heading,
)
from src.navigation.corridor_follower import TurnSide, follow_corridor
from src.navigation.direction_estimator import DirectionEstimator
from src.navigation.maneuvers.parking import ParkController, park_controller_from_metadata
from src.navigation.planning.sign_router import (
    Axis,
    SignRouter,
    SignRouterConfig,
    outward_lateral_axis,
    signs_from_metadata,
)
from src.navigation.planning.waypoints import corridor_for_position, corridor_widths_dict_to_model, plan_believed_path
from src.navigation.ports import DriveCommand, LidarScan
from src.navigation.race_tracker import TRAVEL_DIRS, LapDetector
from src.navigation.start_conditions import assumed_start_conditions
from src.navigation.start_measurement import MeasuredStart, measure_start_pose
from src.navigation.track_geometry import TrackWalls, corridor_geometry_from_widths, corridor_widths_from_metadata
from src.navigation.utils import _nearest_ray, axis_error_rad, wrap_angle
from src.ros2.navigation.ros2_hardware_gateway import ROS2HardwareGateway
from src.ros2.params import declare_param
from src.ros2.qos import QOS_LATCHED_STATE, QOS_LIVE_READOUT, QOS_STREAM
from src.ros2.resettable_node import ResettableNode

logger = logging.getLogger(__name__)


def _direction_gate_verdict(
    ranges_m: Any,
    angles_rad: Any,
    yaw: float,
    tuning: NavigationTuning | None = None,
) -> str:
    """Name which gate in ``infer_direction`` would refuse this scan, for logging.

    Mirrors ``scripts/sim/diag_open_direction_gates.py``'s ``_GateTracer._verdict``
    (a sim-only tool) so a live run's log can show the same diagnosis without
    needing a bag replay -- see that script's docstring for what each gate means.
    """
    tuning = get_tuning(tuning)

    alignment_tol = tuning.direction_estimator.ALIGNMENT_TOLERANCE_RAD
    max_in_track = tuning.direction_estimator.MAX_IN_TRACK_RANGE_M
    min_asymmetry = tuning.direction_estimator.MIN_ASYMMETRY_M
    plausible_span = tuning.direction_estimator.PLAUSIBLE_SPAN_THRESHOLD_M

    axis_error = axis_error_rad(yaw)
    left = _nearest_ray(ranges_m, angles_rad, math.pi / 2)
    right = _nearest_ray(ranges_m, angles_rad, -math.pi / 2)
    if left > max_in_track or right > max_in_track:
        return f"dropout (left={left:.2f} right={right:.2f})"
    if axis_error > alignment_tol:
        return f"align-fail (axis_error={math.degrees(axis_error):.1f}deg)"
    if left + right <= plausible_span:
        return f"span-fail (span={left + right:.2f})"
    if abs(left - right) < min_asymmetry:
        return f"asym-fail (|left-right|={abs(left - right):.3f})"
    return f"vote-pending (left={left:.2f} right={right:.2f})"


_DIRECTION_CHOICES: dict[str, Direction | None] = {
    "cw": Direction.CLOCKWISE,
    "ccw": Direction.COUNTERCLOCKWISE,
    "undetermined": None,
}
"""The round's direction as the operator can state it, including not knowing.

``None`` rather than a third :class:`Direction` member: Direction indexes track
geometry (``TRAVEL_DIRS[(section, direction)]``, ``start_pose``, the estimator's
per-direction vote tally), so a member with no travel vector would make every one
of those lookups fail on a value the type says is legal. "Not known" is already
spelled ``None`` throughout -- ``DirectionEstimator.direction`` returns
``Direction | None`` for exactly this. The three-way choice belongs at the
interface, where the operator speaks, not in the geometry.
"""


def _load_json(path: str | Path) -> dict[str, Any]:
    """Load JSON file."""
    p = Path(path) if isinstance(path, str) else path
    with p.open(encoding="utf-8") as f:
        return cast("dict[str, Any]", json.load(f))


class TrackNavigator(Node, ResettableNode):
    """ROS2 node wrapping the pure Python CoreNavigator."""

    def __init__(  # noqa: PLR0915 - constructor wires every navigation subsystem together
        self,
        metadata_path: str | Path | None = None,
        num_laps: int = CompetitionSpecs.OPEN_CHALLENGE_LAPS,
        params_path: str | Path | None = None,
        tuning_path: str | Path | None = None,
        blind: bool = False,
        direction: Direction | None = None,
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
            direction: Travel direction for the round, or ``None`` for
                undetermined. This is the one starting condition that cannot be
                assumed -- the section can, because assuming it only rotates the
                robot's private world frame, but the direction is a reflection
                and no amount of width learning recovers from getting it wrong.
                See :mod:`src.navigation.start_conditions`. Ignored when
                ``metadata_path`` supplies one.

                ``None`` and a direction are genuinely different instructions,
                which is why this is not simply defaulted. A value means the
                operator KNOWS which way the round runs and blind inference is
                skipped entirely -- no creep, no gate to starve, nothing to
                overturn. ``None`` means nobody said, and the robot infers as
                before. Previously this defaulted to CLOCKWISE, so "told
                clockwise" and "nobody said anything" were the same value: the
                code could not trust it, so it had to infer even when the answer
                had already been supplied. Measured on 2026-08-07, all four
                blind rounds ran on that silent default -- it was right twice
                and wrong twice, and the two rounds where it was RIGHT are the
                two that scored zero (one never confirmed it, one overturned it
                to the wrong answer and drove a mirrored plan into a corner).
        """
        super().__init__("track_navigator")

        # No file means nothing to be sighted with.
        self._blind = blind or metadata_path is None
        # Whether the round's direction was SUPPLIED, as opposed to guessed.
        # Kept as its own flag because the provisional below erases the
        # difference: once undetermined has been resolved to a starting pose,
        # self._direction alone can no longer say whether anyone chose it.
        self._direction_known = direction is not None
        # Undetermined still needs a starting pose to plan from, and the pose is
        # paired with a direction (the two travel vectors for a section are
        # exact opposites). Clockwise is the provisional purely because it is
        # what this defaulted to before; it carries no claim, and the estimator
        # below is free to overturn it.
        provisional = direction if direction is not None else Direction.CLOCKWISE
        self._metadata = (
            _load_json(metadata_path)
            if metadata_path is not None
            else {DictKeys.STARTING_CONDITIONS: assumed_start_conditions(provisional)}
        )
        self._is_open_challenge = self._metadata.get(DictKeys.CHALLENGE_TYPE, ScenarioType.OPEN) == ScenarioType.OPEN

        # Start conditions
        start_cond = self._metadata[DictKeys.STARTING_CONDITIONS]
        start_x = start_cond[DictKeys.POSITION][DictKeys.X]
        start_y = start_cond[DictKeys.POSITION][DictKeys.Y]
        start_yaw = start_cond[DictKeys.YAW]

        # Parameters. Defaults come from RosTopicConfig, the single source of
        # truth for the topics the deployed nodes actually use
        # (ackermann_motor_node's /ackermann_cmd, sllidar_ros2's /scan) — not
        # the pre-Ackermann-migration /wro_robot/cmd_vel and /lidar names this
        # node previously assumed. There is no odom_topic: no node publishes
        # nav_msgs/Odometry on real hardware, so position comes from LIDAR
        # localization instead (see ROS2HardwareGateway).
        self._topics = RosTopicConfig.load_default()
        self._declare_parameters()

        if params_path is not None:
            self._apply_param_overrides(params_path)

        # Setup Tuning. Both challenge profiles are loaded eagerly when blind
        # (no metadata file): the real robot only learns which challenge it is
        # running from the jumper, resolved by state_machine_node and forwarded
        # over /challenge_mode/active well after this constructor runs (at the
        # BOOT_CHECK -> READY transition), so the active one can't be chosen
        # yet here -- only picked, in reset(), once it is known. A metadata- or
        # tuning-file-driven run (sim/test) already knows its challenge for the
        # whole process, so it loads once and never needs the dict.
        self._tuning_by_challenge: dict[ScenarioType, NavigationTuning] | None = None
        if tuning_path:
            tuning = NavigationTuning.load_from_yaml(tuning_path)
        elif self._blind:
            self._tuning_by_challenge = {
                ScenarioType.OPEN: NavigationTuning.load_default(challenge=ScenarioType.OPEN),
                ScenarioType.OBSTACLES: NavigationTuning.load_default(challenge=ScenarioType.OBSTACLES),
            }
            tuning = self._tuning_by_challenge[
                ScenarioType.OPEN if self._is_open_challenge else ScenarioType.OBSTACLES
            ]
        else:
            tuning = NavigationTuning.load_default(
                challenge=ScenarioType.OPEN if self._is_open_challenge else ScenarioType.OBSTACLES,
            )

        raw_section = start_cond[DictKeys.SECTION]
        raw_direction = start_cond[DictKeys.DIRECTION]
        if raw_section is None or raw_direction is None:
            msg = (
                "starting_conditions.section/direction must be resolved by the time "
                "TrackNavigator starts -- real metadata and assumed_start_conditions() "
                "both always supply them; a None here means malformed input"
            )
            raise ValueError(msg)
        start_section = Section.from_string(raw_section)
        start_direction = Direction.from_string(raw_direction)

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
        self._start_xy = Waypoint(start_x, start_y)
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
                tuning=self._tuning,
                # Obstacles corridors are 1.0 m by rule, not by discovery -- a
                # sign/pillar hugging a wall can otherwise feed the voting a
                # run of falsely-narrow readings with nothing to correct it
                # back. See CorridorWidthEstimator's own docstring.
                fixed=not self._is_open_challenge,
            )
            if self._blind
            else None
        )
        # Blind implies inferring the direction: it is drawn at random on the
        # day, so a blind robot cannot be handed it either. ``direction`` is
        # only the provisional the first path is built from, and is replaced
        # the moment the inference settles.
        # Inference exists to answer a question nobody answered. Told the
        # direction, there is nothing to infer -- and inferring anyway is not
        # free: it forces the blind creep, whose alignment gate needs the
        # chassis square to a corridor at the moment one side opens, and those
        # two coincided on 0 of 1763 scans in run 141814 (177 s of creep, zero
        # laps, on a round whose direction had in fact been supplied correctly).
        self._direction_estimator = (
            DirectionEstimator(tuning=self._tuning) if self._blind and not self._direction_known else None
        )
        # One-shot: take the start measurement on the first tick that has a scan
        # when there is no inference to carry it. Sighted runs read their start
        # from metadata and need neither.
        self._pending_known_commit = self._blind and self._direction_known
        self._direction_gate_log_counter = 0
        self._creep_widths: list[tuple[float, float]] = []
        # Set once direction inference settles; None until then, and left
        # None for a scan the measurement refused (see _commit_direction).
        self._measured_start: MeasuredStart | None = None
        # Ticks of retry budget left for a measurement that refused at the
        # commit. Zero means either "not armed" or "spent" -- both are the
        # same thing to _retry_start_measurement, which also stops on the
        # first success, so the budget is only consulted while still refusing.
        self._start_measurement_ticks_left = 0
        # Speed for the blind corridor-follow that runs before the travel
        # direction settles. Named _creep_speed until 2026-08-09, which was
        # doubly misleading: it is not the creep tier, and it never was --
        # it read the slow tier. The medium tier is the closest match to the
        # 0.150 m/s this phase actually ran at, so keeping it here avoids
        # slowing every race start as a side effect of grading the ladder.
        self._blind_follow_speed = tuning.speed.medium_mps()
        self._told_geometry = corridor_widths_from_metadata(self._metadata) if not self._blind else None
        # _told_geometry is None exactly when blind (and then _width_estimator
        # is set instead), so geometry is never actually None here -- just not
        # provable to mypy across the two separately-computed conditions.
        geometry = (
            corridor_geometry_from_widths(self._width_estimator.widths)
            if self._width_estimator
            else self._told_geometry
        )
        assert geometry is not None  # noqa: S101 - either branch above guarantees a value

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

        self._core_navigator = self._build_core_navigator(
            start_xy=Waypoint(start_x, start_y),
            start_section=start_section,
            start_direction=start_direction,
            tuning=tuning,
            num_laps=num_laps,
            waypoints=waypoints,
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
            self._topics.state_machine.state,
            self._on_robot_state,
            QOS_LATCHED_STATE,
        )

        # Jumper-resolved challenge, forwarded by state_machine_node once
        # BOOT_CHECK latches it (see the tuning setup above). Only meaningful
        # when blind -- a metadata/tuning-file run already knows its challenge
        # for the whole process and never subscribes to this. None until the
        # first value arrives; reset() falls back to Open if it never does,
        # matching state_machine_node's own timeout-to-Open fallback so both
        # sides always agree on what "unresolved" means.
        self._active_challenge: ScenarioType | None = None
        if self._blind:
            self.create_subscription(
                String,
                self._topics.challenge_mode.active,
                self._on_challenge_mode_active,
                QOS_LATCHED_STATE,
            )

        # state_machine_node owns /race_metrics (what the OLED and FINISHED
        # transition read) but has no way to count laps itself -- only this
        # node's CoreNavigator/LapDetector actually detects a crossing. BEST_
        # EFFORT, published every control tick regardless of outcome, same
        # rationale as /robot_state above: a dropped sample is corrected
        # within one tick, so nothing here can be allowed to block this loop.
        self._laps_pub = self.create_publisher(
            Int32,
            self._topics.navigation.laps_completed,
            QOS_LIVE_READOUT,
        )

        # Full internal navigation state, every tick, regardless of phase --
        # see NavigatorDebugSnapshot's own docstring. Recorded into every bag
        # (see bag_recorder_node's topic list) so a real-hardware run's
        # crosstrack error, chosen lookahead/target, risk state, direction-gate
        # verdict etc. never again have to be reconstructed by hand from raw
        # /motor/* and /imu/data topics after the fact.
        self._debug_pub = self.create_publisher(String, self._topics.navigation.nav_debug, QOS_STREAM)
        self._latest_debug = NavigatorDebugSnapshot()

        # Control Loop
        control_period = 1.0 / self._tuning.control.CONTROL_HZ
        self.create_timer(control_period, self._control_loop)

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

    def _declare_parameters(self) -> None:
        """Declare this node's ROS2 parameters with their default values."""
        declare_param(self, "ackermann_cmd_topic", self._topics.commands.ackermann_cmd)
        declare_param(self, "lidar_topic", self._topics.sensors.scan)
        declare_param(self, "vision_topic", self._topics.sensors.vision_detections)
        declare_param(self, "imu_topic", self._topics.sensors.imu)
        declare_param(self, "joint_states_topic", self._topics.actuators.joint_states)
        declare_param(self, "is_simulation", default=False)

    def _build_sign_router(self, *, direction: Direction, tuning: NavigationTuning) -> SignRouter | None:
        """Obstacles-only collaborator; ``None`` for Open.

        Shared by ``_build_core_navigator`` and ``reset()`` so a challenge
        switch mid-process (purely from the button, no restart -- see
        ``CoreNavigator.replace_sign_router``) rebuilds it exactly the way the
        first one was built, rather than a second, drifted construction path.
        """
        if self._is_open_challenge:
            return None
        # A blind run has no metadata file, so ``signs_from_metadata`` comes
        # back empty and the router used to be left as None — which meant
        # blind operation shipped with no sign avoidance whatsoever, the one
        # thing the Obstacles Challenge is scored on. Build it regardless and
        # let it discover the layout from ``/vision/detections``, the same
        # way ``CorridorWidthEstimator`` recovers the corridor widths.
        signs = [] if self._blind else signs_from_metadata(self._metadata)
        return SignRouter(
            signs,
            config=SignRouterConfig.from_tuning(tuning.sign_router),
            direction=direction,
            discover=self._blind,
            discovery_config=tuning.sign_discovery,
            tuning=tuning,
        )

    def _build_core_navigator(
        self,
        *,
        start_xy: Waypoint,
        start_section: Section,
        start_direction: Direction,
        tuning: NavigationTuning,
        num_laps: int,
        waypoints: list[Waypoint],
    ) -> CoreNavigator:
        """Build the waypoints, sign router, lap detector, park controller and navigator."""
        sign_router = self._build_sign_router(direction=start_direction, tuning=tuning)

        lap_detector = LapDetector(
            start_pos=start_xy,
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

        return CoreNavigator(
            gateway=self._gateway,
            waypoints=waypoints,
            num_laps=num_laps,
            tuning=tuning,
            sign_router=sign_router,
            lap_detector=lap_detector,
            park_controller=park_controller,
            direction=start_direction,
        )

    def _commit_told_direction(self) -> bool:
        """Take the start measurement for a round whose direction was supplied.

        Skipping inference must not also skip measuring where the robot stands:
        the assumed start is the midpoint of the mat's side, out by 0.35-0.80 m
        on real rounds, and correcting it is what the creep's commit was
        carrying besides the direction itself. ``_commit_direction`` called with
        the direction already held takes its unchanged branch -- reseed position
        to the measurement and replan, leaving heading and the lap line alone.

        Returns:
            ``True`` if this tick had no scan yet and the robot was held, meaning
            there is no plan to step; ``False`` once the measurement is done.
        """
        scan = self._gateway.get_lidar_scan()
        pose = self._gateway.get_current_pose()
        if scan is None or pose is None:
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))
            self._latest_debug = NavigatorDebugSnapshot(
                phase=NavigatorPhase.NO_POSE,
                commanded_speed_mps=0.0,
                commanded_steering_norm=0.0,
            )
            return True
        self._pending_known_commit = False
        self._commit_direction(self._direction, pose, scan)
        return False

    def _sign_dodge_side(self, pose: Pose) -> TurnSide | None:
        """Which side BLIND_CREEP should turn toward to honour the WRO pass-side rule.

        follow_corridor() treats every close obstacle the same way -- turn
        toward whichever side has more LIDAR clearance -- because it never
        sees vision detections and has no notion of sign color. That is
        correct for a plain wall but wrong for a red/green traffic sign,
        which has a fixed pass-side rule (red outward, green inward) instead.
        This resolves that rule from the nearest active sign detection using
        ``outward_lateral_axis`` -- direction-agnostic, so it works even
        though BLIND_CREEP's whole reason for existing is that the travel
        direction is not known yet.

        Returns:
            A :class:`TurnSide` to override follow_corridor's clearance
            heuristic, or ``None`` to defer to it (Open Challenge has no
            signs; no sign is close enough to matter otherwise).
        """
        if self._is_open_challenge:
            return None
        sign_cfg = self._tuning.sign_router
        nearest: TrafficSignObservation | None = None
        nearest_dist = math.inf
        for obs in self._gateway.get_vision_detections():
            if obs.color not in (SignColor.RED, SignColor.GREEN):
                continue
            if obs.confidence < sign_cfg.MIN_CONFIDENCE:
                continue
            dist = math.hypot(obs.world_x_m - pose.x, obs.world_y_m - pose.y)
            if dist < nearest_dist:
                nearest, nearest_dist = obs, dist
        if nearest is None or nearest_dist > sign_cfg.ACTIVATION_DIST_M:
            return None
        routing = outward_lateral_axis(corridor_for_position(nearest.world_x_m, nearest.world_y_m), nearest.color)
        if routing is None:
            return None
        axis, mult = routing
        outward_x, outward_y = (mult, 0.0) if axis is Axis.X else (0.0, mult)
        left_x, left_y = -math.sin(pose.yaw), math.cos(pose.yaw)
        return TurnSide.LEFT if (outward_x * left_x + outward_y * left_y) > 0 else TurnSide.RIGHT

    def _resolve_direction(self) -> bool:
        """Creep along the corridor until the travel direction is inferable.

        Returns:
            ``True`` while the direction is still unknown, meaning this tick
            was driven by the corridor follower and there is no plan to step.
        """
        estimator = self._direction_estimator
        if estimator is None:
            return self._commit_told_direction() if self._pending_known_commit else False
        if estimator.is_settled:
            return False

        scan = self._gateway.get_lidar_scan()
        pose = self._gateway.get_current_pose()
        if scan is None or pose is None:
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))
            self._latest_debug = NavigatorDebugSnapshot(
                phase=NavigatorPhase.NO_POSE,
                commanded_speed_mps=0.0,
                commanded_steering_norm=0.0,
            )
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

        if estimator.observe(scan.ranges_m, scan.angles_rad, pose.yaw, self._tuning):
            inferred = estimator.direction
            if inferred is not None:
                self._commit_direction(inferred, pose, scan)
            return False

        verdict = _direction_gate_verdict(scan.ranges_m, scan.angles_rad, pose.yaw, self._tuning)
        self._direction_gate_log_counter += 1
        if self._direction_gate_log_counter % self._tuning.direction_estimator.GATE_LOG_PERIOD_TICKS == 0:
            logger.info("direction not yet settled: %s (pose=(%.2f, %.2f))", verdict, pose.x, pose.y)

        corridor_width_belief_m = statistics.fmean(w for _, w in self._creep_widths) if self._creep_widths else None
        drive = follow_corridor(
            scan.ranges_m,
            scan.angles_rad,
            self._blind_follow_speed,
            pose.yaw,
            self._tuning,
            forced_turn_side=self._sign_dodge_side(pose),
            believed_width_m=corridor_width_belief_m,
        )
        self._gateway.publish_drive(drive)
        votes = estimator.votes
        self._latest_debug = NavigatorDebugSnapshot(
            phase=NavigatorPhase.BLIND_CREEP,
            pose_x=pose.x,
            pose_y=pose.y,
            pose_yaw=pose.yaw,
            direction_gate_verdict=verdict,
            direction_left_range_m=_nearest_ray(scan.ranges_m, scan.angles_rad, math.pi / 2),
            direction_right_range_m=_nearest_ray(scan.ranges_m, scan.angles_rad, -math.pi / 2),
            direction_votes_clockwise=votes.get(Direction.CLOCKWISE),
            direction_votes_counterclockwise=votes.get(Direction.COUNTERCLOCKWISE),
            corridor_width_belief_m=corridor_width_belief_m,
            commanded_speed_mps=drive.speed_mps,
            commanded_steering_norm=drive.steering_norm,
        )
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
        if len(self._creep_widths) > self._tuning.corridor_estimator.MAX_START_SAMPLES:
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
        assert g is not None  # noqa: S101 - guaranteed non-None in the non-blind branch
        return {
            Section.NORTH: g.north_width_m,
            Section.SOUTH: g.south_width_m,
            Section.EAST: g.east_width_m,
            Section.WEST: g.west_width_m,
        }

    def _commit_direction(self, inferred: Direction, pose: Pose, scan: LidarScan) -> None:
        """Adopt the inferred direction and rebuild everything derived from it.

        Args:
            inferred: The travel direction LIDAR inference settled on.
            pose: Pose as read before any of this method's corrections land.
            scan: The scan inference settled on, reused to measure where the
                robot actually is rather than assume it.
        """
        previous = self._direction
        changed = inferred is not previous
        self._direction = inferred
        # Computed before the width readings are filed, not with the heading
        # correction further down where it is applied. The buffered yaws were
        # recorded against the *old* direction's reference frame, and CW and
        # CCW differ by exactly pi, so filing them by heading against the newly
        # inferred direction attributes every one of them to the opposite
        # corridor -- a reading taken driving down one corridor becomes evidence
        # about the one across the track. reset() already re-stamps the same
        # buffer for exactly this reason (see its comment); this path, which is
        # the only other place those yaws are consumed, did not.
        heading_delta = 0.0
        if changed:
            old_yaw = math.atan2(*TRAVEL_DIRS[(self._start_section, previous)][::-1])
            new_yaw = math.atan2(*TRAVEL_DIRS[(self._start_section, inferred)][::-1])
            heading_delta = wrap_angle(new_yaw - old_yaw)
        if self._width_estimator is not None:
            for buffered_yaw, buffered_width in self._creep_widths:
                self._width_estimator.observe_measurement(
                    section_from_heading(wrap_angle(buffered_yaw + heading_delta), inferred),
                    buffered_width,
                )
            self._creep_widths.clear()
            self._gateway.set_believed_walls(TrackWalls(corridor_geometry_from_widths(self._width_estimator.widths)))

        # Read the starting pose off the track rather than asserting it. The
        # assumed start is the middle of the mat's side, which is not even a
        # legal placement -- the marked square's two cells are centred at 1.25
        # and 1.75, so the assumption sits exactly on the boundary between
        # them. Measured against three real rounds this recovers the true pose
        # to within 5 cm where the assumption was out by 0.35-0.80 m, and the
        # 0.80 m case is the one that drove into a wall with 0.69 m of track
        # ahead while planning for 1.5 m. See src/navigation/start_measurement.py.
        #
        # Done here rather than at the button press because the measurement
        # needs the travel direction (it decides which way "ahead" points and
        # which side the outer wall is on), and this is the moment that becomes
        # known. It reads the current scan, so it yields where the robot is
        # *now* -- the creep displacement this method used to discard is simply
        # never introduced.
        measured = measure_start_pose(
            scan.ranges_m, scan.angles_rad, inferred, self._start_section, tuning=self._tuning,
        )
        seed_xy = Waypoint(measured.x, measured.y) if measured is not None else self._start_xy
        if measured is None:
            # Refusing to guess. Opposite rays that do not span the mat mean
            # something is standing in one of them -- an operator still over
            # the robot is the ordinary case -- and a measurement taken through
            # an obstruction is worse than none.
            #
            # The assumption is not a substitute, and racing on it is what cost
            # both refused rounds on 2026-08-08: it names the middle of the
            # mat's side, so committing after four seconds of creep reseeds the
            # position estimate roughly 0.8 m behind where the robot actually
            # is, and the round then drives into the corner. So arm a retry
            # rather than settle for it. The obstruction is transient by
            # nature -- replaying both bags' whole scan stream, the rearward
            # ray was pinned at 0.10-0.19 m by someone standing behind the
            # robot and cleared 0.6 s and 1.5 s after the commit.
            #
            # The robot keeps driving meanwhile, which is not a compromise but
            # the point: what clears the ray is the robot leaving from under
            # the operator, so holding still would preserve the very
            # obstruction being waited out.
            self._start_measurement_ticks_left = round(
                self._tuning.start_measurement.RETRY_WINDOW_S * self._tuning.control.CONTROL_HZ,
            )
            self.get_logger().warning(
                "Start pose could not be measured from the scan (blocked ray, or not on the track) - "
                "driving on the assumed start, which is only ever approximately right, and retrying "
                f"the measurement for {self._tuning.start_measurement.RETRY_WINDOW_S:.0f} s",
            )
        else:
            self.get_logger().info(
                f"Start pose measured: ({measured.x:.2f}, {measured.y:.2f}), "
                f"{measured.distance_ahead_m:.2f} m of track ahead, "
                f"assumed was ({self._start_xy.x:.2f}, {self._start_xy.y:.2f})",
            )
        self._measured_start = measured

        if changed:
            # assumed_start_conditions paired a starting yaw with whichever
            # direction was assumed at construction (the two travel-direction
            # unit vectors for a section are exact opposites, so CW and CCW
            # always differ by exactly pi). Overturning that assumption
            # rebuilds the path below in the now-correct frame, but leaves the
            # heading estimate anchored to the old, wrong half of that pair --
            # a fixed bias that never decays, since nothing else in the control
            # loop can correct a wrong reference frame, only a wrong response
            # to a correctly-known one. Correcting by exactly the delta
            # between what was assumed and what's now known keeps the
            # estimate's reference matched to the direction it's paired with.
            self._gateway.correct_heading_for_direction_change(heading_delta)
            # The LIDAR localizer takes yaw as given (it only solves for
            # position), so every position fix computed during the creep --
            # while yaw was still anchored to the assumption that just turned
            # out wrong -- was matched against the walls at the wrong
            # orientation and cannot be trusted, however plausible any single
            # fix looked. Its own coarse-to-fine search is bounded to
            # search_radius_m per call, but each call reseeds from the
            # previous (already wrong) fix, so the error compounds across the
            # whole creep instead of correcting once yaw does. Confirmed on
            # real hardware 2026-08-04: two CCW races' pose_x/pose_y showed
            # physically impossible implied speeds (2.8-6.4 m/s against a
            # ~0.156 m/s real maximum) throughout blind_creep and right after
            # this method's yaw correction landed -- the yaw fix alone wasn't
            # enough because it doesn't touch the position estimate the wrong
            # yaw already corrupted. Re-seeding position the same way a new
            # race does (see reset()) is safe here: the blind corridor-follow
            # is capped at the medium tier and this fires within a second or
            # two of race start
            # (both real captures committed by t=1.2s), so the true
            # displacement being discarded is at most ~0.2m -- far smaller
            # than the corruption it replaces. See
            # docs/known-issues-backlog.md.
            self._gateway.reset_position(seed_xy.x, seed_xy.y)
            # ``pose`` was read from the gateway before the corrections above
            # landed, so it still carries the old, now-stale yaw and position
            # -- replan below with the corrected values or the heading-aware
            # reseek in replace_path would use the wrong heading, and resync
            # against a position replace_path won't have caught up to yet.
            pose = Pose(x=seed_xy.x, y=seed_xy.y, yaw=wrap_angle(pose.yaw + heading_delta))
            # The finish line's normal is the travel direction, so a detector
            # built for the provisional one counts crossings inverted. Only the
            # direction is rebuilt: the origin stays the ASSUMED start, not
            # seed_xy.
            #
            # LapDetector fires on a sign flip of (pose - origin).normal that
            # has to coincide with current_section == start_section, and
            # corridor_for_position classifies by the inner square (1.0-2.0 in
            # both axes). The marked starting squares sit at the corridor ends,
            # straddling that square's corner, so a measured start lands in the
            # NEIGHBOURING corridor: all three real starts recorded on
            # 2026-08-06 classify that way -- (2.099, 0.484) and (2.101, 0.487)
            # as EAST, (0.656, 0.596) as WEST, none as the SOUTH they are gated
            # to. Anchoring the line there makes the gate unsatisfiable however
            # many laps are driven. Run 180154 crossed it four times, every
            # crossing labelled east, and scored 0 laps; replayed against the
            # assumed origin the same bag counts 4 (see
            # scripts/bag/diag_bag_lap_origin.py).
            #
            # Nothing is lost by not measuring here. The normal is the travel
            # direction, so the origin's cross-track component has no effect at
            # all, and the along-track component -- the only one that matters --
            # is exactly the one the measurement pushes out of the section. The
            # assumed start is mid-corridor, which is where a lap line belongs.
            # Phase is unaffected: a robot starting past the line still crosses
            # it first after one complete loop. reset() builds it the same way.
            self._core_navigator.replace_lap_detector(
                LapDetector(
                    start_pos=self._start_xy,
                    start_section=self._start_section,
                    direction=inferred,
                ),
            )
            self._core_navigator.set_travel_direction(inferred)
        elif measured is not None:
            # Same correction, for the direction that was assumed correctly.
            # This branch used to do nothing at all, so a run whose inference
            # agreed with the launch default kept the assumed start for the
            # whole race -- measured on the 2026-08-05 clockwise round as a
            # standing 0.35 m error that the localizer's local search can never
            # remove. That round finished, so the error was invisible; it is
            # the same error that ends a counterclockwise round against a wall.
            self._gateway.reset_position(seed_xy.x, seed_xy.y)
            pose = Pose(x=seed_xy.x, y=seed_xy.y, yaw=pose.yaw)
        # Resync unconditionally: the navigator did not step during the creep,
        # so its waypoint index is still 0 while the robot has driven a metre
        # past it, and it would resume by chasing a waypoint behind itself.
        self._core_navigator.replace_path(self._plan(self._to_widths_dict()), (pose.x, pose.y), pose.yaw)
        self.get_logger().info(f"Travel direction inferred from LIDAR: {inferred}")

    def _retry_start_measurement(self) -> None:
        """Re-attempt a start measurement the commit refused, once per tick.

        A refusal at the commit is a statement about that one scan, not about
        the round: what blocks a cardinal ray is a person standing in it, and
        they stop blocking it as soon as the robot has driven clear. Both
        rounds that refused on 2026-08-08 had a valid measurement available
        within 1.5 s of the commit, and raced the whole round on the assumed
        start regardless -- roughly 0.8 m out, which is the error that put them
        into the corner. So the measurement is retried until one lands rather
        than abandoned after one look.

        Latched on the first success: this corrects the *starting* pose, and
        once it is corrected the LIDAR localizer owns the position estimate.
        The budget bounds it to the first seconds after the commit, because
        the pose it computes is expressed in the starting section's frame and
        the robot leaves that section for good at the first corner.
        """
        if self._measured_start is not None or self._start_measurement_ticks_left <= 0:
            return
        self._start_measurement_ticks_left -= 1
        scan = self._gateway.get_lidar_scan()
        pose = self._gateway.get_current_pose()
        if scan is None or pose is None:
            return

        # Unlike the commit, which measures a scan the direction was just
        # inferred from, a retry happens while driving and has to establish for
        # itself that the chassis is still square to the starting corridor.
        # measure_start_pose cannot: its closing check asks only that forward
        # and back span the mat, which a chassis turned through 180 degrees
        # does just as well, and then ``along`` comes out mirrored about the
        # mat's centre. Comparing against the corridor's travel bearing is what
        # tells the two apart.
        travel = TRAVEL_DIRS[(self._start_section, self._direction)]
        misalignment = abs(wrap_angle(pose.yaw - math.atan2(travel[1], travel[0])))
        if misalignment > math.radians(self._tuning.start_measurement.RETRY_ALIGN_TOLERANCE_DEG):
            return

        measured = measure_start_pose(
            scan.ranges_m, scan.angles_rad, self._direction, self._start_section, tuning=self._tuning,
        )
        if measured is None:
            return

        self._measured_start = measured
        self._start_measurement_ticks_left = 0
        self._gateway.reset_position(measured.x, measured.y)
        # Same resync the commit does, and for the same reason: the path was
        # built around a position that has just been replaced, so the waypoint
        # index has to be re-sought against the corrected pose rather than
        # carried over.
        self._core_navigator.replace_path(
            self._plan(self._to_widths_dict()), (measured.x, measured.y), pose.yaw,
        )
        self.get_logger().info(
            f"Start pose measured on retry: ({measured.x:.2f}, {measured.y:.2f}), "
            f"{measured.distance_ahead_m:.2f} m of track ahead - "
            f"position estimate corrected from ({pose.x:.2f}, {pose.y:.2f})",
        )

    def _plan(self, widths: dict[Section, float]) -> list[Waypoint]:
        """Build a one-lap path for the layout the robot believes it is on."""
        # self._metadata is a plain dict (from _load_json, or the assumed-start
        # fallback literal) everywhere else in this class -- never actually a
        # ScenarioMetadata. Validating it here (rather than just annotating it
        # as one) is what plan_believed_path's .replanned_at()/.replanned_with()
        # calls need to not crash with AttributeError: 'dict' object has no
        # attribute 'starting_conditions'.
        #
        # corridor_widths has no default on ScenarioMetadata (deliberately --
        # see its docstring), and a blind run's self._metadata never carries
        # one at all: there is no scenario file to read it from, only the
        # live width estimate this method receives as `widths`. Validating
        # self._metadata as-is therefore raised on every blind run before
        # plan_believed_path ever got a chance to supply the real value --
        # merge it in up front instead of patching it in after.
        new_widths = corridor_widths_dict_to_model(widths)
        metadata = ScenarioMetadata.model_validate({**self._metadata, DictKeys.CORRIDOR_WIDTHS: new_widths})
        # direction is a typed kwarg on plan_believed_path, not a raw dict: a
        # past str(self._direction) here type-checked and passed silently while
        # every `direction is Direction.CLOCKWISE` test downstream
        # (calculate_waypoints, _build_corridor_order, start_measurement,
        # parking, collision avoidance) read False. A clockwise round was
        # therefore planned counterclockwise -- the path ran the opposite way
        # around the mat, so every waypoint "ahead" in path order sat behind the
        # chassis, the lookahead search skipped most of a lap to the first
        # barely-forward point ~2.5 m away, and pure pursuit's 1/distance^2
        # curvature answered an 86 deg bearing error with 5% of full lock.
        # Measured on 2026-08-08 runs 140300/140513: 0 laps, speed pinned at the
        # 0.05 m/s creep floor for 100% of ticks. Counterclockwise rounds were
        # unaffected, which is why this survived: the wrong branch is the CCW one.
        starting = metadata.starting_conditions
        return plan_believed_path(
            metadata,
            widths,
            direction=self._direction,
            believed_section=starting.section,
            believed_position=starting.position,
            believed_yaw=starting.yaw,
            arc_radius=self._arc_radius,
            tuning=self._tuning,
        )

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

    def _on_challenge_mode_active(self, msg: String) -> None:
        """Cache the jumper-resolved challenge; reset() applies it at the next RACING entry."""
        self._active_challenge = ScenarioType.from_string(msg.data)

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

        Also re-seeds the position estimate to this race's starting pose, for
        the same reason: the state machine can cycle FINISHED -> BOOT_CHECK ->
        READY -> RACING purely from the button, with no process restart, so
        without this a new race's very first tick starts from wherever the
        *previous* race's LIDAR localizer last drifted to, not from the new
        race's actual starting pose -- confirmed on real hardware 2026-08-04,
        pose_x/pose_y in the hundreds of metres on a 3m track, continuous
        across a race boundary (see docs/known-issues-backlog.md). The
        localizer's own drift during a single race is a separate, still-open
        question; this only stops it from compounding across races.

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
        self._gateway.reset_position(self._start_xy.x, self._start_xy.y)
        # Re-stamp anything measured before the start to the heading frame that
        # reset just established. The yaws recorded against the old reference
        # would otherwise file those readings under the wrong section, since
        # _commit_direction attributes them by heading. Zero is correct rather
        # than approximate: this method runs at the one instant the robot is
        # known to be sitting at its starting pose, so "the heading it has now"
        # and "the heading those readings were taken at" are the same.
        self._creep_widths = [(0.0, width) for _, width in self._creep_widths]
        # Belongs to the round that just ended: the robot is picked up and put
        # down between rounds, so the next one measures its own. The retry
        # budget goes with it -- a round that never refused leaves it at zero,
        # and one that spent it must not start the next round already spent.
        self._measured_start = None
        self._start_measurement_ticks_left = 0

        # A blind round may be running a different challenge than the last one
        # -- the operator can move the jumper and long-press reset between
        # rounds with no process restart (see CoreNavigator.replace_sign_router
        # below). Re-resolve which challenge is active before anything past
        # this point reads self._is_open_challenge/self._tuning/self._arc_radius.
        # Falls back to Open -- matching state_machine_node's own
        # timeout-to-Open fallback -- if no value has arrived yet, e.g. this is
        # the very first race and BOOT_CHECK hasn't published.
        if self._tuning_by_challenge is not None:
            active = self._active_challenge or ScenarioType.OPEN
            self._is_open_challenge = active == ScenarioType.OPEN
            self._tuning = self._tuning_by_challenge[active]
            self._arc_radius = self._tuning.waypoints.ARC_RADIUS

        self._direction = self._initial_direction
        if self._blind:
            self._width_estimator = CorridorWidthEstimator(
                assumed_width=CorridorDimensions.NARROW
                if self._is_open_challenge
                else CorridorDimensions.OBSTACLES_WIDTH,
                tuning=self._tuning,
                fixed=not self._is_open_challenge,
            )
            # Mirrors construction: a told direction is still told on the next
            # round, so rebuilding an estimator here would put the creep back
            # for every race after the first.
            self._direction_estimator = (
                DirectionEstimator(tuning=self._tuning) if not self._direction_known else None
            )
            self._pending_known_commit = self._direction_known
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
        # Same reasoning as the park controller above: SignRouter is built
        # once at __init__ time (when the challenge may still be a guess) and
        # never rebuilt on its own, so a challenge switch resolved just above
        # has to be threaded through here too, or a round that switches
        # Open<->Obstacles mid-process would keep running the previous
        # round's sign-avoidance behaviour (or lack of it).
        self._core_navigator.replace_sign_router(
            self._build_sign_router(direction=self._direction, tuning=self._tuning),
        )
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
        self._core_navigator.set_travel_direction(self._direction)
        self._core_navigator.replace_path(
            self._plan(self._to_widths_dict()),
            (self._start_xy.x, self._start_xy.y),
        )
        self._core_navigator.reset()

    def _control_loop(self) -> None:
        """Execute one control step, or hold the robot stopped when not racing."""
        self._laps_pub.publish(Int32(data=self._core_navigator.laps_completed))
        try:
            if not self._racing:
                # Keep publishing zeros rather than going silent: ackermann_motor_node
                # has a 1 s command watchdog, and silence would let it latch a stop
                # only after that delay.
                self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))
                self._sample_start_corridor()
                self._latest_debug = NavigatorDebugSnapshot(
                    phase=NavigatorPhase.NOT_YET_STEPPED,
                    commanded_speed_mps=0.0,
                    commanded_steering_norm=0.0,
                )
                return
            if self._resolve_direction():
                # Direction unknown: the corridor follower drove this tick and
                # there is no usable plan to step yet. _resolve_direction already
                # set self._latest_debug.
                return
            # Before the belief update and the step, so a landed measurement
            # replans from the corrected position rather than letting this
            # tick's step chase waypoints laid out around the wrong one.
            self._retry_start_measurement()
            if self._blind:
                self._update_layout_belief()
            self._core_navigator.step()
            self._latest_debug = self._core_navigator.debug_snapshot
        except RuntimeError as e:
            self.get_logger().error(f"Runtime error in control loop: {e}")
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))
        except ValueError as e:
            self.get_logger().error(f"Value error in control loop: {e}")
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))
        finally:
            if self._width_estimator is not None:
                believed = self._width_estimator.widths
                self._latest_debug.belief_north_m = believed.get(Section.NORTH)
                self._latest_debug.belief_south_m = believed.get(Section.SOUTH)
                self._latest_debug.belief_east_m = believed.get(Section.EAST)
                self._latest_debug.belief_west_m = believed.get(Section.WEST)
            if self._measured_start is not None:
                self._latest_debug.start_measured_x = self._measured_start.x
                self._latest_debug.start_measured_y = self._measured_start.y
                self._latest_debug.start_measurement_ahead_m = self._measured_start.distance_ahead_m
                self._latest_debug.start_measured_corridor_width_m = self._measured_start.corridor_width_m
            localizer_inputs = self._gateway.get_localizer_inputs()
            if localizer_inputs is not None:
                yaw, prior_x, prior_y = localizer_inputs
                self._latest_debug.localizer_input_yaw_rad = yaw
                self._latest_debug.localizer_prior_x = prior_x
                self._latest_debug.localizer_prior_y = prior_y
            self._debug_pub.publish(String(data=self._latest_debug.model_dump_json()))

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
    """Run the ROS2 track navigator node (``ros2 run vtitan_navigation track_navigator_node``)."""
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
        choices=["cw", "ccw", "undetermined"],
        default="undetermined",
        help="Travel direction for the round (default: undetermined). Used only when "
        "--metadata is omitted. The starting section does not need one of these because "
        "assuming it merely rotates the robot's own world frame; the direction is a "
        "reflection and has to be right. 'undetermined' is a real third answer, not a "
        "missing one: cw/ccw mean the operator KNOWS, so blind inference is skipped "
        "outright, while undetermined means nobody said and the robot creeps and infers. "
        "This used to default to cw, which made 'told cw' indistinguishable from "
        "'unset', so the value could never be trusted and inference ran regardless.",
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
            direction=_DIRECTION_CHOICES[parsed.direction],
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
