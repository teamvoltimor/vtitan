"""Static TF transforms for WRO robot sensor frames.

Publishes fixed transforms so RViz, nav2, and any node that needs
sensor-frame coordinates can resolve them from TF without a running URDF publisher.

Poses match the Gazebo SDF in sdf_robot_builder.py and the URDF in
robot_description/wro_robot.urdf:
  camera_link : front-top of chassis  x=+0.14  y=0   z=+0.10
  lidar_link  : top of chassis         x=0      y=0   z=+0.12
  imu_link    : near chassis bottom    x=0      y=0   z=+0.01
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_base_to_camera",
                arguments=["0.14", "0", "0.10", "0", "0", "0", "base_link", "camera_link"],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_base_to_lidar",
                arguments=["0", "0", "0.12", "0", "0", "0", "base_link", "lidar_link"],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_base_to_imu",
                arguments=["0", "0", "0.01", "0", "0", "0", "base_link", "imu_link"],
                output="screen",
            ),
        ],
    )
