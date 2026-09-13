#!/usr/bin/env python3
"""
WRO Simulation Launch File.

Launches Gazebo Ionic with a WRO track scenario and spawns the Ackermann robot.
Starts Zenoh router for rmw_zenoh_cpp middleware.

Usage:
    ros2 launch wro_simulation wro_simulation.launch.py
    ros2 launch wro_simulation wro_simulation.launch.py world:=scenario_0001.sdf
"""

from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the WRO simulation."""
    # Declare arguments
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="wro_track_base.sdf",
        description="World SDF file name (relative to worlds directory)",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time",
    )

    robot_x_arg = DeclareLaunchArgument(
        "robot_x",
        default_value="0.0",
        description="Robot spawn X position",
    )

    robot_y_arg = DeclareLaunchArgument(
        "robot_y",
        default_value="-1.2",
        description="Robot spawn Y position",
    )

    robot_z_arg = DeclareLaunchArgument(
        "robot_z",
        default_value="0.1",
        description="Robot spawn Z position",
    )

    robot_yaw_arg = DeclareLaunchArgument(
        "robot_yaw",
        default_value="1.5708",
        description="Robot spawn yaw orientation (radians)",
    )

    # Get paths
    pkg_share = FindPackageShare("wro_simulation").find("wro_simulation")
    world_file = PathJoinSubstitution(
        [
            pkg_share,
            "worlds",
            LaunchConfiguration("world"),
        ],
    )

    urdf_xacro_file = PathJoinSubstitution(
        [
            pkg_share,
            "urdf",
            "wro_robot.urdf.xacro",
        ],
    )

    # Process xacro to produce robot_description
    robot_description_content = Command(
        [
            "xacro",
            urdf_xacro_file,
        ],
    )

    # Zenoh router (must start before any ROS 2 nodes using rmw_zenoh_cpp)
    zenoh_router = ExecuteProcess(
        cmd=["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
        name="zenoh_router",
        output="screen",
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ],
                ),
            ],
        ),
        launch_arguments={
            "gz_args": ["-r -v 4 ", world_file],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Robot State Publisher (receives processed xacro content)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "robot_description": robot_description_content,
            },
        ],
    )

    # Spawn robot
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "wro_robot",
            "-string",
            robot_description_content,
            "-x",
            LaunchConfiguration("robot_x"),
            "-y",
            LaunchConfiguration("robot_y"),
            "-z",
            LaunchConfiguration("robot_z"),
            "-Y",
            LaunchConfiguration("robot_yaw"),
        ],
        output="screen",
    )

    # Bridge Gazebo topics to ROS2
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/wro_robot/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/wro_robot/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/wro_robot/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/wro_robot/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/wro_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/wro_robot/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            world_arg,
            use_sim_time_arg,
            robot_x_arg,
            robot_y_arg,
            robot_z_arg,
            robot_yaw_arg,
            zenoh_router,
            gazebo,
            robot_state_publisher,
            spawn_robot,
            bridge,
        ],
    )
