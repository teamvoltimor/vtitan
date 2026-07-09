"""Static TF transforms for WRO robot sensor frames.

Publishes fixed transforms so RViz, nav2, and any node that needs
sensor-frame coordinates can resolve them from TF without a running URDF publisher.

Poses match the Gazebo SDF in sdf_robot_builder.py and the URDF in
robot_description/wro_robot.urdf:
  camera_link : front-top of chassis  x=+0.14  y=0   z=+0.10
  lidar_link  : top of chassis         x=0      y=0   z=+0.12
  imu_link    : near chassis bottom    x=0      y=0   z=+0.01

lidar_link's yaw defaults to 180 deg (LIDAR_YAW_OFFSET_DEG) because the C1 is mounted
inverted -- its raw angle-zero points opposite robot-front. Confirmed empirically during
sensor verification: the sector with the largest ranges (open space, robot-front) lands
at +-180 deg in the raw /scan data, not 0 deg.
"""

import math

from launch import LaunchDescription
from launch_ros.actions import Node
from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict


class Config(BaseSettings):
    """Static TF configuration."""

    model_config = SettingsConfigDict(env_prefix="")

    lidar_yaw_offset_deg: float = Field(default=180.0, validation_alias="LIDAR_YAW_OFFSET_DEG")


_config = Config()


def _static_tf(name: str, x: float, y: float, z: float, yaw_rad: float, frame_id: str, child_frame_id: str) -> Node:
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--x", str(x),
            "--y", str(y),
            "--z", str(z),
            "--yaw", str(yaw_rad),
            "--pitch", "0",
            "--roll", "0",
            "--frame-id", frame_id,
            "--child-frame-id", child_frame_id,
        ],
        output="screen",
    )


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for static sensor-frame transforms."""
    lidar_yaw_rad = math.radians(_config.lidar_yaw_offset_deg)

    return LaunchDescription(
        [
            _static_tf("tf_base_to_camera", 0.14, 0, 0.10, 0.0, "base_link", "camera_link"),
            _static_tf("tf_base_to_lidar", 0, 0, 0.12, lidar_yaw_rad, "base_link", "lidar_link"),
            _static_tf("tf_base_to_imu", 0, 0, 0.01, 0.0, "base_link", "imu_link"),
        ],
    )
