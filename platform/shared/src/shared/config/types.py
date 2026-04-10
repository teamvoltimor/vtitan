"""TypedDict definitions for inter-function data structures."""

from typing import TypedDict

from shared.domain.enums import Direction, LightingScenario, Section


class StartingPosition(TypedDict):
    """Robot starting position coordinates."""

    x: float
    y: float


class StartingConditions(TypedDict):
    """Metadata for the robot's starting configuration."""

    direction: Direction
    section: Section
    section_name: str
    position: tuple[float, float]
    yaw: float


class CorridorWidthConfig(TypedDict):
    """Configuration for a single corridor's width.

    Replaces the old ambiguous CorridorWidthTypeDict with clear semantics.
    """

    type: str  # WidthTypes.NARROW or WidthTypes.WIDE
    width: float  # Width in meters (0.6 or 1.0)


class CorridorWidths(TypedDict):
    """Mapping of corridor sections to their width configurations.

    Use as: dict[Section, CorridorWidthConfig]
    """

    pass  # This is just a marker; actual usage is dict[Section, CorridorWidthConfig]


class LightingConfig(TypedDict):
    """Configuration for lighting in a scenario.

    Represents all lighting parameters that affect the simulation
    environment appearance and camera behavior.
    """

    intensity: float  # Sun intensity [0.0, 1.0]
    ambient_intensity: float  # Ambient light [0.0, 1.0]
    direction: list[float]  # Sun direction [x, y, z] (3 elements)
    cast_shadows: bool  # Whether sun casts shadows
    scenario: str  # Lighting scenario name (LightingScenario value)


class ParkingBlock(TypedDict):
    """Configuration for a single parking block."""

    position: tuple[float, float]  # (x, y) world coordinates
    yaw: float  # Rotation angle in radians


class ParkingLotConfig(TypedDict):
    """Parking lot configuration for obstacles challenge.

    Contains positions and orientations for two parking blocks.
    """

    block1_pos: tuple[float, float]  # First block (x, y)
    block2_pos: tuple[float, float]  # Second block (x, y)
    block1_yaw: float  # First block rotation
    block2_yaw: float  # Second block rotation
    depth: float  # Depth of blocks from grid entry


class StartingZoneConfig(TypedDict):
    """Configuration for the starting zone placement."""

    length: float  # Length of starting zone
    x: float  # X position (world coordinates)
    y: float  # Y position (world coordinates)


class TrafficSignColor(TypedDict):
    """Traffic sign color specification."""

    name: str  # ColorNames.RED or ColorNames.GREEN
    rgb: list[float]  # Normalized RGB [0.0, 1.0]


# ─────────────────────────────────────────────────────────────────────────────
# Runtime Data Structures
# ─────────────────────────────────────────────────────────────────────────────


class DetectionDict(TypedDict, total=False):
    """Structure for a single detection from computer vision model.

    Fields:
        class_name: Name of detected class (e.g., 'sign', 'obstacle')
        confidence: Detection confidence score [0.0, 1.0]
        bbox: Bounding box as [x_min, y_min, x_max, y_max]
        x: Center x-coordinate (pixel)
        y: Center y-coordinate (pixel)
        width: Bounding box width (pixel)
        height: Bounding box height (pixel)
        area: Bounding box area (pixel²)
    """

    class_name: str
    confidence: float
    bbox: list[float]
    x: float
    y: float
    width: float
    height: float
    area: float


class TopicUpdateDict(TypedDict, total=False):
    """Structure for ROS2 topic update message.

    Fields:
        topic_name: Name of ROS2 topic
        timestamp: Message timestamp (seconds)
        data: Raw message fields as nested dict
    """

    topic_name: str
    timestamp: float
    data: dict  # Generic dict for flexible message content


class HealthCheckResponseDict(TypedDict):
    """API response for health check endpoint.

    Fields:
        status: Health status ('healthy', 'degraded', 'unhealthy')
        uptime_seconds: System uptime in seconds
        active_nodes: Number of active ROS2 nodes
    """

    status: str
    uptime_seconds: float
    active_nodes: int


class ConfigResponseDict(TypedDict):
    """API response for configuration endpoint.

    Fields:
        speed_profiles: Available speed profiles
        current_profile: Currently active profile
        params: Current parameter values
    """

    speed_profiles: list[str]
    current_profile: str
    params: dict


class CameraParamsDict(TypedDict, total=False):
    """Camera calibration and configuration parameters.

    Fields:
        focal_length: Camera focal length
        principal_point: [cx, cy] principal point
        image_width: Image width in pixels
        image_height: Image height in pixels
        distortion_coefficients: Camera distortion model coefficients
    """

    focal_length: float
    principal_point: list[float]
    image_width: int
    image_height: int
    distortion_coefficients: list[float]


class SystemStatusDict(TypedDict, total=False):
    """System health status snapshot.

    Fields:
        battery_voltage: Battery voltage (V)
        battery_current: Battery current (A)
        cpu_temp: CPU temperature (°C)
        motor_temps: Temperature of each motor (°C)
        ros_nodes_active: Number of active ROS2 nodes
    """

    battery_voltage: float
    battery_current: float
    cpu_temp: float
    motor_temps: list[float]
    ros_nodes_active: int


class RaceMetricsDict(TypedDict, total=False):
    """Racing metrics during autonomous run.

    Fields:
        lap_number: Current lap (1-indexed)
        lap_time: Time for current lap (seconds)
        total_time: Total elapsed time (seconds)
        waypoint_index: Current waypoint index
        collision_count: Number of collisions detected
        forward_clearance: Current forward clearance (meters)
    """

    lap_number: int
    lap_time: float
    total_time: float
    waypoint_index: int
    collision_count: int
    forward_clearance: float


class MetricsDict(TypedDict, total=False):
    """WebSocket/telemetry metrics summary.

    Fields:
        broadcast_failures: Number of failed broadcasts
        client_disconnects: Number of client disconnections
        broadcast_timeouts: Number of broadcast timeouts
        messages_sent: Total messages sent
        bytes_transmitted: Total bytes transmitted
    """

    broadcast_failures: int
    client_disconnects: int
    broadcast_timeouts: int
    messages_sent: int
    bytes_transmitted: int


class RobotSnapshotDict(TypedDict, total=False):
    """Complete snapshot of robot state and telemetry.

    Fields:
        timestamp: Snapshot timestamp (seconds)
        position: [x, y] position in world frame
        yaw: Robot heading (radians)
        linear_velocity: Forward velocity (m/s)
        angular_velocity: Rotational velocity (rad/s)
        lidar_ranges: LIDAR scan ranges (meters)
        vision_detections: List of detected objects
        system_status: System health metrics
        race_metrics: Current race metrics
    """

    timestamp: float
    position: list[float]
    yaw: float
    linear_velocity: float
    angular_velocity: float
    lidar_ranges: list[float]
    vision_detections: list[DetectionDict]
    system_status: SystemStatusDict
    race_metrics: RaceMetricsDict


class ParamPatchDict(TypedDict, total=False):
    """Single parameter patch for tuning.

    Fields:
        name: Parameter name (e.g., 'max_speed', 'steering_kp')
        old_value: Previous value
        new_value: New value
        timestamp: When patch was applied
    """

    name: str
    old_value: float | str | bool
    new_value: float | str | bool
    timestamp: float


class SpeedUpdateResponseDict(TypedDict):
    """API response for speed configuration update.

    Fields:
        status: Update status ('success', 'failed')
        max_linear_speed: Updated maximum linear speed (m/s)
    """

    status: str
    max_linear_speed: float
