"""Static TF transforms for WRO robot sensor frames.

Publishes fixed transforms so RViz, nav2, and any node that needs
sensor-frame coordinates can resolve them from TF without a running URDF publisher.

Poses match the Go Gazebo SDF generator (platform/gazebo/generator/internal/sdf/robot.go)
and the URDF in robot_description/wro_robot.urdf (measured 2026-07-11, see
../../../../docs/robot-physical-constants.md):
  camera_link : over the LIDAR, angled down  x=+0.1222  y=0   z=+0.16  pitch=+30deg
  lidar_link  : front of chassis, centered   x=+0.1222  y=0   z=+0.12
  imu_link    : near chassis bottom          x=0        y=0   z=+0.01

lidar_link's yaw comes from RobotSpecs.LIDAR_MOUNT_YAW_OFFSET_DEG (180 deg) because the C1
is mounted inverted -- its raw angle-zero points opposite robot-front. Confirmed empirically
during sensor verification: the sector with the largest ranges (open space, robot-front)
lands at +-180 deg in the raw /scan data, not 0 deg.

camera_link's x/z position is still an estimate pending a real measurement -- if it looks off
in RViz, correct RobotSpecs.CAMERA_MOUNT_X_OFFSET/CAMERA_MOUNT_Z_OFFSET. Its pitch SIGN
(positive = tilts the view down) has been visually confirmed against the live sim/RViz robot
markers in src/simulation/live_visualizer.py -- positive pitch really does point the camera
down and forward, not up.
"""

import math

from launch import LaunchDescription
from launch_ros.actions import Node
from shared.config.constants import RobotSpecs


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
            "--x",
            str(x),
            "--y",
            str(y),
            "--z",
            str(z),
            "--yaw",
            str(yaw_rad),
            "--pitch",
            str(pitch_rad),
            "--roll",
            "0",
            "--frame-id",
            frame_id,
            "--child-frame-id",
            child_frame_id,
        ],
        output="screen",
    )


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for static sensor-frame transforms."""
    lidar_yaw_rad = math.radians(RobotSpecs.LIDAR_MOUNT_YAW_OFFSET_DEG)
    camera_pitch_rad = math.radians(RobotSpecs.CAMERA_MOUNT_PITCH_DEG)

    return LaunchDescription(
        [
            _static_tf(
                "tf_base_to_camera",
                RobotSpecs.CAMERA_MOUNT_X_OFFSET,
                0,
                RobotSpecs.CAMERA_MOUNT_Z_OFFSET,
                0.0,
                "base_link",
                "camera_link",
                pitch_rad=camera_pitch_rad,
            ),
            _static_tf(
                "tf_base_to_lidar",
                RobotSpecs.LIDAR_MOUNT_X_OFFSET,
                0,
                RobotSpecs.HEIGHT + RobotSpecs.LIDAR_MOUNT_Z_OFFSET,
                lidar_yaw_rad,
                "base_link",
                "lidar_link",
            ),
            _static_tf(
                "tf_base_to_imu",
                0,
                0,
                RobotSpecs.IMU_MOUNT_Z_OFFSET,
                0.0,
                "base_link",
                "imu_link",
            ),
        ],
    )
