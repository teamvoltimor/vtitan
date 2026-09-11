"""Static TF transforms for WRO robot sensor frames.

Publishes fixed transforms so RViz, nav2, and any node that needs
sensor-frame coordinates can resolve them from TF without a running URDF publisher.

Poses match the Go Gazebo SDF generator (src/go/internal/simgen/sdf/robot.go)
and the URDF in robot_description/wro_robot.urdf (measured 2026-07-11, see
../../../../docs/robot-physical-constants.md):
  camera_link : over the LIDAR, angled down  x=+0.1222  y=0   z=+0.16  pitch=+30deg
  lidar_link  : front of chassis, centered   x=+0.1222  y=0   z=+0.12
  imu_link    : near chassis bottom          x=0        y=0   z=+0.01

lidar_link's yaw comes from RobotSpecs.lidar_yaw_offset_rad() -- the mandatory
rotation from an upside-down mount plus any independent residual miscalibration,
matching ros2_hardware_gateway.py's _LIDAR_YAW_OFFSET_RAD (same classmethod
call). Re-verified 2026-08-02 at 0 deg total against a known object placed at
chassis front/back (an earlier "confirmed empirically... 180 deg" finding no
longer matched the mounting as it exists today). See src/config/robot.toml's
[lidar] section and RobotSpecs.lidar_yaw_offset_rad()'s docstring for the full
history and why every consumer calls the shared classmethod instead of
recomputing the formula locally.

camera_link's x/z position is still an estimate pending a real measurement -- if it looks off
in RViz, correct RobotSpecs.CAMERA_MOUNT_X_OFFSET/CAMERA_MOUNT_Z_OFFSET. Its pitch SIGN
(positive = tilts the view down) has been visually confirmed against the live sim/RViz robot
markers in src/simulation/live_visualizer.py -- positive pitch really does point the camera
down and forward, not up.
"""

import math

from launch import LaunchDescription
from launch_ros.actions import Node
from shared.config.constants import RobotSpecs, TfFrames


def _static_tf(
    name: str,
    x: float,
    y: float,
    z: float,
    yaw_rad: float,
    frame_id: TfFrames,
    child_frame_id: TfFrames,
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
            # str() like the numeric arguments above: launch substitutes every
            # entry in this list, and TfFrames is a StrEnum, so what reaches
            # static_transform_publisher should be the bare frame name and
            # nothing that merely compares equal to it.
            "--frame-id",
            str(frame_id),
            "--child-frame-id",
            str(child_frame_id),
        ],
        output="screen",
    )


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for static sensor-frame transforms."""
    # Same RobotSpecs.lidar_yaw_offset_rad() call as ros2_hardware_gateway.py's
    # _LIDAR_YAW_OFFSET_RAD -- see that classmethod's docstring for what it
    # combines and why it lives there instead of being recomputed per-consumer.
    lidar_yaw_rad = RobotSpecs.lidar_yaw_offset_rad()
    camera_pitch_rad = math.radians(RobotSpecs.CAMERA_MOUNT_PITCH_DEG)

    return LaunchDescription(
        [
            _static_tf(
                "tf_base_to_camera",
                RobotSpecs.CAMERA_MOUNT_X_OFFSET,
                0,
                RobotSpecs.CAMERA_MOUNT_Z_OFFSET,
                0.0,
                TfFrames.BASE_LINK,
                TfFrames.CAMERA_LINK,
                pitch_rad=camera_pitch_rad,
            ),
            _static_tf(
                "tf_base_to_lidar",
                RobotSpecs.LIDAR_MOUNT_X_OFFSET,
                0,
                RobotSpecs.HEIGHT + RobotSpecs.LIDAR_MOUNT_Z_OFFSET,
                lidar_yaw_rad,
                TfFrames.BASE_LINK,
                TfFrames.LIDAR_LINK,
            ),
            _static_tf(
                "tf_base_to_imu",
                0,
                0,
                RobotSpecs.IMU_MOUNT_Z_OFFSET,
                0.0,
                TfFrames.BASE_LINK,
                TfFrames.IMU_LINK,
            ),
        ],
    )
