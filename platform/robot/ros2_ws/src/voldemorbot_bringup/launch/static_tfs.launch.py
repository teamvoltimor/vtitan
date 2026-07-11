"""Static TF transforms for WRO robot sensor frames.

Publishes fixed transforms so RViz, nav2, and any node that needs
sensor-frame coordinates can resolve them from TF without a running URDF publisher.

Poses match the Go Gazebo SDF generator (platform/gazebo/generator/internal/sdf/robot.go)
and the URDF in robot_description/wro_robot.urdf (measured 2026-07-11, see
../../../../docs/robot-physical-constants.md):
  camera_link : over the LIDAR, angled down  x=+0.1222  y=0   z=+0.16  pitch=+30deg
  lidar_link  : front of chassis, centered   x=+0.1222  y=0   z=+0.12
  imu_link    : near chassis bottom          x=0        y=0   z=+0.01

lidar_link's yaw defaults to 180 deg (LIDAR_YAW_OFFSET_DEG) because the C1 is mounted
inverted -- its raw angle-zero points opposite robot-front. Confirmed empirically during
sensor verification: the sector with the largest ranges (open space, robot-front) lands
at +-180 deg in the raw /scan data, not 0 deg.

camera_link's x/z position is an estimate pending a real measurement -- verify visually in
RViz and correct RobotSpecs.CAMERA_MOUNT_X_OFFSET/CAMERA_MOUNT_Z_OFFSET if wrong. Its pitch
SIGN follows precedent already in this repo: wro_robot.urdf.xacro's camera_joint used
`rpy="0 0.2 0"` (positive Y) for what its own "camera would be over the lidar, tilted down"
intent implies is a downward tilt, so positive pitch = down is treated as this codebase's
established convention here too, not re-derived from first principles.
"""

import math

from launch import LaunchDescription
from launch_ros.actions import Node
from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict
from shared.config.constants import RobotSpecs


class Config(BaseSettings):
    """Static TF configuration."""

    model_config = SettingsConfigDict(env_prefix="")

    lidar_yaw_offset_deg: float = Field(default=180.0, validation_alias="LIDAR_YAW_OFFSET_DEG")


_config = Config()


def _static_tf(
    name: str,
    x: float,
    y: float,
    z: float,
    yaw_rad: float,
    frame_id: str,
    child_frame_id: str,
    pitch_rad: float = 0.0,
) -> Node:
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--x", str(x),
            "--y", str(y),
            "--z", str(z),
            "--yaw", str(yaw_rad),
            "--pitch", str(pitch_rad),
            "--roll", "0",
            "--frame-id", frame_id,
            "--child-frame-id", child_frame_id,
        ],
        output="screen",
    )


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for static sensor-frame transforms."""
    lidar_yaw_rad = math.radians(_config.lidar_yaw_offset_deg)
    camera_pitch_rad = math.radians(RobotSpecs.CAMERA_MOUNT_PITCH_DEG)

    return LaunchDescription(
        [
            _static_tf(
                "tf_base_to_camera",
                RobotSpecs.CAMERA_MOUNT_X_OFFSET, 0, RobotSpecs.CAMERA_MOUNT_Z_OFFSET,
                0.0, "base_link", "camera_link", pitch_rad=camera_pitch_rad,
            ),
            _static_tf(
                "tf_base_to_lidar",
                RobotSpecs.LIDAR_MOUNT_X_OFFSET, 0, 0.12,
                lidar_yaw_rad, "base_link", "lidar_link",
            ),
            _static_tf("tf_base_to_imu", 0, 0, 0.01, 0.0, "base_link", "imu_link"),
        ],
    )
