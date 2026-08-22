"""Publish the headless simulator's ground-truth state to ROS2 for RViz/Gazebo.

Lets the exact same closed-loop scenario the headless test battery runs
(``ScenarioSimulator`` driving the real ``CoreNavigator``) be watched live: the
pose becomes a TF frame + ``nav_msgs/Odometry``, the raycast LIDAR becomes a
``sensor_msgs/LaserScan``, and the track walls become a one-shot
``visualization_msgs/MarkerArray`` built from the same ``TrackModel`` geometry
the collision checks use — so what you see is what was actually tested, not a
static approximation. The car itself is a second MarkerArray: chassis, sensor
mounts, and four road wheels carrying the live steering angle, so the
counter-phase both axles actually steer with is visible rather than implied.

Requires the ``robot`` pixi environment (ROS2 Kilted / RoboStack); not
importable from the plain ``uv`` env the headless tests run in.

Watch it with ``task sim:navigate:rviz``, which loads
``config/live_visualization.rviz``. NOT ``task sim:rviz`` -- that is a bare
RViz with no saved config, so it comes up as an empty grid. (Both now run out
of the robot platform's ``sim`` pixi environment; until 2026-08-22
``gazebo/runtime`` carried a duplicate ROS2 install of its own, which made the
empty grid a DDS discovery problem as well as a missing-config one.)
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from shared.config.constants import (
    ParkingLotSpecs,
    RobotSpecs,
    TfFrames,
    TrackDimensions,
    TrafficSignSpecs,
    WallSpecs,
)
from shared.config.ros_topics import RosTopicConfig
from shared.domain.models import BlockPosition, ParkingLot, SignColor, SignPosition
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from src.config.tuning_helpers import get_tuning
from src.navigation.utils import wrap_angle
from src.ros2.qos import QOS_STREAM
from src.simulation.kinematics import wheel_poses

if TYPE_CHECKING:
    from src.navigation.ports import LidarScan
    from src.simulation.kinematics import AckermannState, WheelPose
    from src.simulation.track_model import TrackModel

_FRONT_AXLE_COLOR = (0.0, 0.9, 0.9)
_REAR_AXLE_COLOR = (1.0, 0.5, 0.0)
"""Steer-arrow colours, per axle. The two axles turn in OPPOSITE directions on
this chassis, so colouring them apart is what makes the counter-phase legible
at a glance instead of looking like one axle drawn twice."""


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def _pitch_to_quaternion(pitch: float) -> Quaternion:
    return Quaternion(x=0.0, y=math.sin(pitch / 2.0), z=0.0, w=math.cos(pitch / 2.0))


def _is_masked_bearing(angle_rad: float, sectors: object) -> bool:
    """True where the mount occludes its own sensor, by ANGLE not by range.

    These rays self-collide as a matter of geometry, so whatever range comes
    back is meaningless regardless of how large it is -- which is exactly why
    the navigator's own rear-sector read excludes them by bearing rather than
    filtering on distance. Same two wedges, same config.
    """
    degrees = math.degrees(wrap_angle(angle_rad))
    return (
        sectors.BLIND_WEDGE_LEFT_MIN_DEG <= degrees <= sectors.BLIND_WEDGE_LEFT_MAX_DEG
        or sectors.BLIND_WEDGE_RIGHT_MIN_DEG <= degrees <= sectors.BLIND_WEDGE_RIGHT_MAX_DEG
    )


def _wheel_to_quaternion(steer: float, roll: float = 0.0) -> Quaternion:
    """Orient a marker as a road wheel, steered by ``steer`` and rolled by ``roll``.

    A Marker CYLINDER extrudes along its own z, but a wheel's axle is lateral,
    so this is Rz(steer) * Rx(90deg) * Rz(-roll): the ``rpy="${pi/2} 0 0"`` the
    URDF applies to its wheel visuals, steered about the world vertical, then
    spun about the wheel's own axle. That last rotation is post-multiplied
    because after Rx(90deg) the marker's local z IS the axle.

    ``roll`` is positive rolling forward. It is negated inside because Rx(90deg)
    lays the axle along -y, so a naive positive rotation would spin the wheel
    backwards while the robot drove forwards.

    Written out as the closed-form product rather than by composing three
    Quaternion objects: expanding it collapses to half-angle sums (s =
    sqrt(2)/2 is Rx(90deg)'s shared term), which is both shorter and cheaper
    than the general multiply, and matches the two helpers above.
    """
    s = math.sqrt(2.0) / 2.0
    steered, spun = (steer + roll) / 2.0, (steer - roll) / 2.0
    return Quaternion(
        x=s * math.cos(steered),
        y=s * math.sin(steered),
        z=s * math.sin(spun),
        w=s * math.cos(spun),
    )


class LiveScenarioVisualizer(Node):
    """Publishes one running scenario's pose/LIDAR/track to ROS2 topics."""

    _TRACK_REPUBLISH_EVERY_N_TICKS = 20
    """Re-send the cached track/robot MarkerArray about once a second (at the default 20Hz
    sim rate) instead of relying on a single one-shot publish. A late-connecting or
    reconnecting RViz subscriber can otherwise miss that first publish entirely and never
    show the markers, since a plain volatile-QoS publish isn't retained for late joiners."""

    def __init__(self, track: TrackModel, node_name: str = "sim_live_visualizer") -> None:
        super().__init__(node_name)
        topics = RosTopicConfig.load_default()
        self._odom_pub = self.create_publisher(Odometry, topics.simulation.odom, QOS_STREAM)
        self._scan_pub = self.create_publisher(
            LaserScan,
            topics.sensors.scan,
            QOS_STREAM,
        )
        self._track_pub = self.create_publisher(MarkerArray, topics.simulation.track, 1)
        self._robot_model_pub = self.create_publisher(MarkerArray, topics.simulation.robot_model, 1)
        self._plan_pub = self.create_publisher(Path, topics.simulation.plan, 1)
        self._sign_estimate_pub = self.create_publisher(MarkerArray, topics.simulation.sign_estimates, 1)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._cached_track_markers: MarkerArray | None = None
        self._tick_count = 0
        self._wheel_roll = 0.0
        self._previous_position: tuple[float, float] | None = None
        self._belief_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.set_track(track)

    def set_belief_frame(
        self,
        believed_start: tuple[float, float, float],
        true_start: tuple[float, float, float],
    ) -> None:
        """Register how far the robot's world is rotated from the real one.

        A blind run seeds the believed start from ``assumed_start_conditions``,
        which always guesses SOUTH, while the chassis is placed at the
        scenario's true start. The believed-vs-true offset is therefore a rigid
        rotation by the section-relabelling angle -- NORTH reads 180 deg, EAST
        90, WEST -90 -- measured stable to within 1.4 deg over a whole run.

        The plan and the sign estimates are expressed in that believed frame,
        so drawing them against ``map`` puts them on a track the robot is not
        driving: on ``go_obstacles_0002`` (true section WEST) the whole path and
        every sign estimate appear square to the real layout. This publishes
        the offset as a transform instead of rewriting the numbers, so RViz
        composes it and the navigator keeps reasoning in its own frame.

        Both poses are ``(x, y, yaw)``. Pass equal poses -- or never call this
        -- for a sighted run, where the belief is already correct.
        """
        believed_x, believed_y, believed_yaw = believed_start
        true_x, true_y, true_yaw = true_start
        delta_yaw = wrap_angle(true_yaw - believed_yaw)
        # Rotate the believed origin into the true frame, then offset so the
        # believed start lands exactly on the true one.
        cos_d, sin_d = math.cos(delta_yaw), math.sin(delta_yaw)
        self._belief_offset = (
            true_x - (believed_x * cos_d - believed_y * sin_d),
            true_y - (believed_x * sin_d + believed_y * cos_d),
            delta_yaw,
        )

    def _broadcast_belief_frame(self, stamp: object) -> None:
        offset_x, offset_y, offset_yaw = self._belief_offset
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = TfFrames.MAP
        tf.child_frame_id = TfFrames.BELIEF
        tf.transform.translation.x = offset_x
        tf.transform.translation.y = offset_y
        tf.transform.rotation = _yaw_to_quaternion(offset_yaw)
        self._tf_broadcaster.sendTransform(tf)

    def set_track(
        self,
        track: TrackModel,
        sign_positions: list[SignPosition] | None = None,
        parking_lot: ParkingLot | None = None,
    ) -> None:
        """(Re)publish the track walls, signs, and parking lot — call again per scenario.

        Takes the validated models, not the raw metadata dicts these used to
        be. That moves the int-to-float coercion the marker builders each did
        by hand onto the model boundary, where it happens once and cannot be
        forgotten — see :meth:`_sign_marker`.
        """
        markers = MarkerArray()
        # Clear every previously published marker first. Sign/parking marker IDs are just
        # 0..N-1 within their namespace — switching to a scenario with fewer signs (or no
        # parking lot) would otherwise leave the previous scenario's markers stuck on screen,
        # rendered at positions that belong to a different track layout entirely (e.g. sitting
        # on/through the new track's wall).
        markers.markers.append(Marker(action=Marker.DELETEALL))
        markers.markers.extend(self._outer_wall_markers())
        markers.markers.append(self._inner_block_marker(track))
        for i, sign in enumerate(sign_positions or []):
            markers.markers.append(self._sign_marker(i, sign))
        if parking_lot is not None:
            markers.markers.append(
                self._parking_block_marker(10, parking_lot.block1_position, parking_lot.block1_yaw),
            )
            markers.markers.append(
                self._parking_block_marker(11, parking_lot.block2_position, parking_lot.block2_yaw),
            )
        self._cached_track_markers = markers
        # A new scenario teleports the robot to a new start pose. Wheel roll is
        # integrated from the distance between consecutive poses, so carrying
        # the old one over would bill that jump as travel.
        self._previous_position = None
        self._track_pub.publish(markers)

    def publish(self, state: AckermannState, scan: LidarScan | None) -> None:
        """Publish one tick's pose (odom + TF) and LIDAR sweep."""
        self._tick_count += 1
        if self._cached_track_markers is not None and (self._tick_count % self._TRACK_REPUBLISH_EVERY_N_TICKS == 0):
            self._track_pub.publish(self._cached_track_markers)

        stamp = self.get_clock().now().to_msg()
        quat = _yaw_to_quaternion(state.yaw)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = TfFrames.MAP
        tf.child_frame_id = TfFrames.BASE_LINK
        tf.transform.translation.x = state.x
        tf.transform.translation.y = state.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = quat
        self._tf_broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = TfFrames.MAP
        odom.child_frame_id = TfFrames.BASE_LINK
        odom.pose.pose.position.x = state.x
        odom.pose.pose.position.y = state.y
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = state.v
        self._odom_pub.publish(odom)

        self._advance_wheel_roll(state)
        self._robot_model_pub.publish(self._robot_model_markers(state.steer))

        if scan is not None and scan.ranges_m:
            self._scan_pub.publish(self._build_laserscan(scan, stamp))

    def publish_belief(self, navigator: object) -> None:
        """Publish the plan the navigator holds and the sign estimates behind it.

        Separate from :meth:`publish`, which takes simulator ground truth. These
        two are beliefs, and in a blind Obstacles run they are the algorithm
        itself: the planned polyline IS the avoidance manoeuvre, since
        ``apply_sign_lanes`` rewrites the corridor's waypoints onto a pass-side
        lane rather than steering off a fixed path. Watching a run without them
        shows the chassis moving and the true signs standing still, with the
        decision that connects the two invisible.

        Republished every tick rather than cached like the track markers: the
        plan is rebuilt whenever discovery refines a sign estimate, so a stale
        one would show a lane the robot is no longer driving.

        Typed loosely because ``live_visualizer`` is importable only from the
        ROS2 pixi env while ``CoreNavigator`` is not, and a hard import here
        would drag the whole navigation package into that env's import graph.
        """
        stamp = self.get_clock().now().to_msg()
        # Beliefs go out in the BELIEF frame, and the transform that carries the
        # believed-vs-true offset goes with them. Both, every tick: a marker
        # whose frame RViz has no transform for is dropped silently, so
        # publishing the geometry without the frame would just make the plan
        # disappear rather than draw it in the wrong place.
        self._broadcast_belief_frame(stamp)
        waypoints = getattr(navigator, "_waypoints", None)
        if waypoints:
            path = Path()
            path.header.stamp = stamp
            path.header.frame_id = TfFrames.BELIEF
            for waypoint in waypoints:
                pose = PoseStamped()
                pose.header.stamp = stamp
                pose.header.frame_id = TfFrames.BELIEF
                pose.pose.position.x = float(waypoint.x)
                pose.pose.position.y = float(waypoint.y)
                path.poses.append(pose)
            self._plan_pub.publish(path)

        router = getattr(navigator, "sign_router", None)
        specs = getattr(router, "lane_specs", None) if router is not None else None
        if specs is None:
            return
        markers = MarkerArray()
        markers.markers.append(Marker(action=Marker.DELETEALL))
        for index, entry in enumerate(specs):
            markers.markers.append(self._sign_estimate_marker(index, entry[0]))
        self._sign_estimate_pub.publish(markers)

    def _sign_estimate_marker(self, index: int, spec: object) -> Marker:
        """One believed sign, drawn so it cannot be mistaken for the real one.

        Deliberately taller and translucent rather than a different colour: the
        colour carries the pass side, which is the whole reason the estimate
        matters, so overriding it to mean "estimate" would hide the field being
        checked. Height and alpha are free to carry that instead.
        """
        marker = Marker()
        # BELIEF, not MAP: this is where the router THINKS the sign is, and on a
        # blind run that frame is rotated from the real track. Drawing it in map
        # is what put the translucent signs at square-to-reality positions.
        marker.header.frame_id = TfFrames.BELIEF
        marker.ns = "sign_estimates"
        marker.id = index
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        # float() for the same reason `_sign_marker` needs it: an int assigned to
        # a Point field survives in memory and is reinterpreted bit-for-bit as a
        # float64 on the wire, dropping the marker to ~0.0.
        marker.pose.position.x = float(spec.x)
        marker.pose.position.y = float(spec.y)
        marker.pose.position.z = TrafficSignSpecs.Z_POSITION + TrafficSignSpecs.HEIGHT
        marker.pose.orientation.w = 1.0
        marker.scale.x = TrafficSignSpecs.WIDTH
        marker.scale.y = TrafficSignSpecs.DEPTH
        marker.scale.z = TrafficSignSpecs.HEIGHT
        color = TrafficSignSpecs.RED_COLOR if spec.color == SignColor.RED else TrafficSignSpecs.GREEN_COLOR
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = *color, 0.35
        return marker

    def _build_laserscan(self, scan: LidarScan, stamp: object) -> LaserScan:
        """The sweep, with the bearings the mount cannot see blanked out.

        The raycast model casts a full circle from the LIDAR against walls and
        obstacles; it does not model the chassis occluding its own sensor. So
        the rear rays come back as clean wall returns where the real C1 sees
        nothing but its own body — the view claimed sensing the robot does not
        have, and the rear is now masked EDGE TO EDGE (the ~25 deg slot that
        used to exist straight back is gone on the current mount).

        Blanked with NaN, which is the LaserScan convention for "no return" and
        which RViz skips rather than drawing at zero. Masked here rather than in
        the raycast because this is a display concern: the navigator applies the
        same wedges itself when it reads a sector.
        """
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = TfFrames.BASE_LINK
        angles = scan.angles_rad
        msg.angle_min = angles[0]
        msg.angle_max = angles[-1]
        msg.angle_increment = (angles[-1] - angles[0]) / max(1, len(angles) - 1)
        msg.range_min = RobotSpecs.LIDAR_MIN_RANGE
        msg.range_max = RobotSpecs.LIDAR_MAX_RANGE
        sectors = get_tuning().lidar_sectors
        msg.ranges = [
            math.nan if _is_masked_bearing(angle, sectors) or range_m <= sectors.SELF_DETECTION_THRESHOLD_M else range_m
            for angle, range_m in zip(angles, scan.ranges_m, strict=True)
        ]
        return msg

    def _outer_wall_markers(self) -> list[Marker]:
        """The four boundary walls, as real boxes standing on the ground.

        Was a single LINE_STRIP traced along the boundary, which got the walls
        wrong twice over: a line strip is flat, so they had no height at all
        and vanished the moment you left the top-down view; and it was centred
        ON the boundary, drawing half a wall thickness INTO the drivable area.

        Geometry mirrors ``addExteriorWalls`` in
        ``gazebo/generator/internal/sdf/world.go`` so RViz and the generated
        SDF describe the same track: centre half a thickness beyond
        MIN/MAX_COORD, which puts the inner face exactly on the boundary the
        collision checks use, and length spanning the mat rather than the
        track, so the corners close.
        """
        outer = TrackDimensions.MAX_COORD + WallSpecs.THICKNESS / 2.0
        inner = TrackDimensions.MIN_COORD - WallSpecs.THICKNESS / 2.0
        centre = TrackDimensions.CENTER_COORD
        span = TrackDimensions.MAT_SIZE + WallSpecs.THICKNESS
        placements = (
            (centre, outer, span, WallSpecs.THICKNESS),  # north
            (centre, inner, span, WallSpecs.THICKNESS),  # south
            (outer, centre, WallSpecs.THICKNESS, span),  # east
            (inner, centre, WallSpecs.THICKNESS, span),  # west
        )
        return [self._wall_marker(index, *placement) for index, placement in enumerate(placements)]

    def _wall_marker(self, index: int, x: float, y: float, size_x: float, size_y: float) -> Marker:
        m = Marker()
        m.header.frame_id = TfFrames.MAP
        m.ns = "track"
        m.id = index
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = WallSpecs.HEIGHT / 2.0
        m.pose.orientation.w = 1.0
        m.scale.x = size_x
        m.scale.y = size_y
        m.scale.z = WallSpecs.HEIGHT
        # Deliberately NOT WallSpecs.COLOR. That is black, which is right for
        # the Gazebo render and invisible against RViz's dark grey background.
        m.color.r, m.color.g, m.color.b, m.color.a = 0.8, 0.8, 0.8, 1.0
        return m

    def _inner_block_marker(self, track: TrackModel) -> Marker:
        x_min, y_min, x_max, y_max = track.inner_block_visual
        m = Marker()
        m.header.frame_id = TfFrames.MAP
        m.ns = "track"
        # 4, not 1: ids 0..3 are the four outer walls now.
        m.id = 4
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = (x_min + x_max) / 2.0
        m.pose.position.y = (y_min + y_max) / 2.0
        # Same height as the outer walls, from the same constant — these were
        # 0.05/0.10 literals, which happened to agree with WallSpecs and would
        # not have followed it if track.toml changed.
        m.pose.position.z = WallSpecs.HEIGHT / 2.0
        m.pose.orientation.w = 1.0
        m.scale.x = x_max - x_min
        m.scale.y = y_max - y_min
        m.scale.z = WallSpecs.HEIGHT
        m.color.r, m.color.g, m.color.b, m.color.a = 0.6, 0.2, 0.2, 1.0
        return m

    def _sign_marker(self, index: int, sign: SignPosition) -> Marker:
        """One ground-truth sign.

        The x/y here used to be ``float(sign["x"])`` against a raw dict, and the
        cast was load-bearing: whole-number coordinates parse from JSON as
        Python ``int``, and an int assigned to a Point field looks fine
        in-memory but is reinterpreted bit-for-bit as a float64 by CDR on the
        wire, collapsing the sign to a near-zero subnormal on the far side.
        ``SignPosition`` declares them ``float``, so pydantic converts once at
        the boundary and no marker builder has to remember.
        """
        m = Marker()
        m.header.frame_id = TfFrames.MAP
        m.ns = "signs"
        m.id = index
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = sign.x
        m.pose.position.y = sign.y
        m.pose.position.z = TrafficSignSpecs.Z_POSITION
        m.pose.orientation.w = 1.0
        m.scale.x = TrafficSignSpecs.WIDTH
        m.scale.y = TrafficSignSpecs.DEPTH
        m.scale.z = TrafficSignSpecs.HEIGHT
        color = TrafficSignSpecs.RED_COLOR if sign.color == SignColor.RED else TrafficSignSpecs.GREEN_COLOR
        m.color.r, m.color.g, m.color.b, m.color.a = *color, 1.0
        return m

    def _parking_block_marker(self, index: int, block: BlockPosition, yaw: float) -> Marker:
        m = Marker()
        m.header.frame_id = TfFrames.MAP
        m.ns = "parking"
        m.id = index
        m.type = Marker.CUBE
        m.action = Marker.ADD
        # See _sign_marker for why these are no longer float()-cast by hand.
        m.pose.position.x = block.x
        m.pose.position.y = block.y
        m.pose.position.z = ParkingLotSpecs.Z_POSITION
        m.pose.orientation = _yaw_to_quaternion(yaw)
        m.scale.x = ParkingLotSpecs.LENGTH
        m.scale.y = ParkingLotSpecs.WIDTH
        m.scale.z = ParkingLotSpecs.HEIGHT
        m.color.r, m.color.g, m.color.b, m.color.a = *ParkingLotSpecs.COLOR, 1.0
        return m

    def _advance_wheel_roll(self, state: AckermannState) -> None:
        """Integrate how far the wheels have rotated, from ground distance travelled.

        Odometry rather than ``v * dt``: :meth:`publish` is handed a state, not
        an interval, and the visualizer can be paced at any wall-clock rate
        (``--rate 5``), so anything clocked off real time would spin the wheels
        at the wrong speed. Distance between consecutive poses has neither
        problem -- it is what the wheel actually rolled over.

        Sign comes from ``state.v`` because distance is unsigned: reversing has
        to unwind the wheels, and reverse is not incidental here -- escape-thrash
        (latched reverse into a wall) is a live failure mode this view is used
        to diagnose, and wheels that kept spinning forward would hide it.
        """
        previous, self._previous_position = self._previous_position, (state.x, state.y)
        if previous is None:
            return
        travelled = math.hypot(state.x - previous[0], state.y - previous[1])
        if state.v < 0.0:
            travelled = -travelled
        # Wrapped, not accumulated: a long run would otherwise grow the angle
        # without bound and start losing float precision in the fractional part.
        self._wheel_roll = math.remainder(
            self._wheel_roll + travelled / RobotSpecs.WHEEL_RADIUS,
            math.tau,
        )

    def _robot_model_markers(self, steer: float = 0.0) -> MarkerArray:
        """Chassis/LIDAR/camera geometry plus the four road wheels, in ``base_link``.

        This sim/RViz path has no URDF or Gazebo mesh — this is the only place these
        mount offsets (LIDAR front-mount, camera-over-LIDAR pitch) are visible without
        launching Gazebo. Positioned directly from ``RobotSpecs`` (not a duplicate copy)
        so it can't itself drift from the values it's meant to let you check.

        The body is static, but the wheels take the live ``steer`` and the
        integrated roll, which is why the whole array is republished every tick
        rather than cached like the track. Marker IDs are stable per part, so no
        DELETEALL is needed — each publish overwrites the previous one in place.
        """
        cam_quat = _pitch_to_quaternion(math.radians(RobotSpecs.CAMERA_MOUNT_PITCH_DEG))
        markers = MarkerArray()
        markers.markers.extend([
            self._chassis_marker(),
            self._robot_lidar_marker(),
            self._camera_marker(cam_quat),
            self._camera_facing_marker(cam_quat),
        ])
        for index, wheel in enumerate(wheel_poses(steer)):
            markers.markers.append(self._wheel_marker(index, wheel))
            markers.markers.append(self._wheel_spoke_marker(index, wheel))
            markers.markers.append(self._wheel_steer_marker(index, wheel))
        return markers

    def _wheel_marker(self, index: int, wheel: WheelPose) -> Marker:
        """One road wheel at its true track/wheelbase position, steered and rolling.

        Note where this sits vertically: ``base_link`` here is the ground plane
        (the chassis cube runs 0..HEIGHT), so a wheel centred at WHEEL_RADIUS is
        *inside* the chassis box — and TRACK_WIDTH is narrower than the chassis
        WIDTH, so it is enclosed laterally too. That is the real geometry, not a
        drawing error, and it is why :meth:`_wheel_steer_marker` exists: from the
        saved config's top-down view these cylinders are behind the body.
        """
        m = Marker()
        m.header.frame_id = TfFrames.BASE_LINK
        m.ns = "wheels"
        m.id = index
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = wheel.x
        m.pose.position.y = wheel.y
        m.pose.position.z = RobotSpecs.WHEEL_RADIUS
        m.pose.orientation = _wheel_to_quaternion(wheel.steer, self._wheel_roll)
        m.scale.x = m.scale.y = 2.0 * RobotSpecs.WHEEL_RADIUS
        m.scale.z = RobotSpecs.WHEEL_WIDTH
        # Same dark_rubber the URDF gives its wheel visuals.
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.1, 0.1, 1.0
        return m

    def _wheel_spoke_marker(self, index: int, wheel: WheelPose) -> Marker:
        """A diameter stripe across the wheel, so the rolling is actually visible.

        The cylinder in :meth:`_wheel_marker` carries the true roll angle, but a
        smooth surface of revolution spinning about its own axis renders
        identically at every angle — correct and completely invisible. This
        stripe is the reference mark that makes it read, like a tyre's balance
        dot. It is deliberately a hair longer than the wheel is wide so its ends
        poke out of both sidewalls rather than z-fighting with them.
        """
        m = Marker()
        m.header.frame_id = TfFrames.BASE_LINK
        m.ns = "wheels"
        # Offset past the cylinders' 0..3 — same namespace so one RViz checkbox
        # toggles a wheel and its mark together.
        m.id = index + 4
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = wheel.x
        m.pose.position.y = wheel.y
        m.pose.position.z = RobotSpecs.WHEEL_RADIUS
        m.pose.orientation = _wheel_to_quaternion(wheel.steer, self._wheel_roll)
        m.scale.x = 2.0 * RobotSpecs.WHEEL_RADIUS  # a full diameter, in the wheel's plane
        m.scale.y = 0.008  # stripe width
        m.scale.z = RobotSpecs.WHEEL_WIDTH * 1.1  # proud of both sidewalls
        m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.9, 0.35, 1.0
        return m

    def _wheel_steer_marker(self, index: int, wheel: WheelPose) -> Marker:
        """The wheel's pointing direction, drawn clear of the chassis roof.

        Same idea as :meth:`_camera_facing_marker`: a rotated primitive is hard
        to read, and here it is worse — the cylinder is buried inside the body
        (see :meth:`_wheel_marker`), and the saved RViz config opens in
        TopDownOrtho, looking straight down at that body. The arrow floats just
        above the roof at the wheel's own x/y, so the front pair and the rear
        pair visibly swing opposite ways as the robot turns.
        """
        m = Marker()
        m.header.frame_id = TfFrames.BASE_LINK
        m.ns = "wheel_steer"
        m.id = index
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = wheel.x
        m.pose.position.y = wheel.y
        m.pose.position.z = RobotSpecs.HEIGHT + 0.005
        m.pose.orientation = _yaw_to_quaternion(wheel.steer)
        m.scale.x = 2.0 * RobotSpecs.WHEEL_RADIUS  # shaft length: one wheel diameter
        m.scale.y = 0.012  # shaft diameter
        m.scale.z = 0.012  # head diameter
        color = _FRONT_AXLE_COLOR if wheel.name.startswith("front") else _REAR_AXLE_COLOR
        m.color.r, m.color.g, m.color.b, m.color.a = *color, 1.0
        return m

    def _chassis_marker(self) -> Marker:
        m = Marker()
        m.header.frame_id = TfFrames.BASE_LINK
        m.ns = "robot"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.z = RobotSpecs.HEIGHT / 2.0
        m.pose.orientation.w = 1.0
        m.scale.x = RobotSpecs.LENGTH
        m.scale.y = RobotSpecs.WIDTH
        m.scale.z = RobotSpecs.HEIGHT
        # Alpha well below half on purpose: the wheels sit INSIDE this box (see
        # _wheel_marker), so at the old 0.6 the running gear was there but not
        # readable through the body.
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.0, 0.8, 0.3
        return m

    def _robot_lidar_marker(self) -> Marker:
        # HEIGHT + LIDAR_MOUNT_Z_OFFSET = 0.12, matching static_tfs.launch.py /
        # the Go SDF generator. Read from config, not the 0.02 literal that used
        # to sit here beside a comment naming the value it duplicated.
        m = Marker()
        m.header.frame_id = TfFrames.BASE_LINK
        m.ns = "robot"
        m.id = 1
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = RobotSpecs.LIDAR_MOUNT_X_OFFSET
        m.pose.position.z = RobotSpecs.HEIGHT + RobotSpecs.LIDAR_MOUNT_Z_OFFSET
        m.pose.orientation.w = 1.0
        m.scale.x = RobotSpecs.LIDAR_DIAMETER
        m.scale.y = RobotSpecs.LIDAR_DIAMETER
        m.scale.z = RobotSpecs.LIDAR_HEIGHT
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.1, 0.1, 1.0
        return m

    def _camera_marker(self, orientation: Quaternion) -> Marker:
        m = Marker()
        m.header.frame_id = TfFrames.BASE_LINK
        m.ns = "robot"
        m.id = 2
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = RobotSpecs.CAMERA_MOUNT_X_OFFSET
        m.pose.position.z = RobotSpecs.CAMERA_MOUNT_Z_OFFSET
        m.pose.orientation = orientation
        m.scale.x = m.scale.y = m.scale.z = 0.025
        m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.2, 0.2, 1.0
        return m

    def _camera_facing_marker(self, orientation: Quaternion) -> Marker:
        # A plain cube's own rotation is hard to read at a glance -- this arrow makes the
        # camera's actual look direction (including the downward pitch) unambiguous.
        m = Marker()
        m.header.frame_id = TfFrames.BASE_LINK
        m.ns = "robot"
        m.id = 3
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = RobotSpecs.CAMERA_MOUNT_X_OFFSET
        m.pose.position.z = RobotSpecs.CAMERA_MOUNT_Z_OFFSET
        m.pose.orientation = orientation
        m.scale.x = 0.15  # shaft length
        m.scale.y = 0.02  # shaft diameter
        m.scale.z = 0.02  # head diameter
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 1.0
        return m


class RealTimePacer:
    """Sleeps between ticks so a headless-speed loop plays back at wall-clock rate."""

    def __init__(self, dt: float, rate: float = 1.0) -> None:
        self._tick_budget = dt / rate if rate > 0 else 0.0
        self._next_tick: float | None = None

    def wait(self) -> None:
        """Block until the next tick's wall-clock deadline (no-op if unthrottled)."""
        if self._tick_budget <= 0.0:
            return
        now = time.monotonic()
        if self._next_tick is None:
            self._next_tick = now
        self._next_tick += self._tick_budget
        delay = self._next_tick - now
        if delay > 0.0:
            time.sleep(delay)
        else:
            self._next_tick = now


def init_rclpy_once() -> None:
    """Initialize rclpy if it hasn't been already (idempotent for script reuse)."""
    if not rclpy.ok():
        rclpy.init()
