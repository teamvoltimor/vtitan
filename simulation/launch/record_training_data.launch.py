#!/usr/bin/env python3
"""
Launch file for recording training data from WRO scenarios

This launch file:
1. Launches Gazebo with a specified scenario
2. Spawns the robot at the correct starting position
3. Records the camera feed to a ROS2 bag OR directly to video

Usage:
    # Record scenario to ROS2 bag
    ros2 launch simulation record_training_data.launch.py \
        scenario_file:=/path/to/scenario_0001.sdf \
        metadata_file:=/path/to/scenario_0001_metadata.json \
        duration:=30

    # Record with video output
    ros2 launch simulation record_training_data.launch.py \
        scenario_file:=/path/to/scenario_0001.sdf \
        metadata_file:=/path/to/scenario_0001_metadata.json \
        output_video:=/path/to/output.mp4 \
        duration:=30
"""

import os
import json
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
    Shutdown
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    # Launch arguments
    scenario_file_arg = DeclareLaunchArgument(
        'scenario_file',
        description='Path to scenario SDF file'
    )

    metadata_file_arg = DeclareLaunchArgument(
        'metadata_file',
        default_value='',
        description='Path to scenario metadata JSON file'
    )

    duration_arg = DeclareLaunchArgument(
        'duration',
        default_value='30',
        description='Recording duration in seconds'
    )

    output_video_arg = DeclareLaunchArgument(
        'output_video',
        default_value='',
        description='Output video file path (optional, otherwise uses ROS2 bag)'
    )

    output_bag_arg = DeclareLaunchArgument(
        'output_bag',
        default_value='./wro_recording',
        description='Output ROS2 bag directory (used if output_video not specified)'
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Run Gazebo headless (no GUI)'
    )

    # Get launch configurations
    scenario_file = LaunchConfiguration('scenario_file')
    duration = LaunchConfiguration('duration')
    headless = LaunchConfiguration('headless')

    # Launch Gazebo with scenario
    # Build gz sim command based on headless flag
    gz_args = PythonExpression([
        '"', '-r -s ', scenario_file, '"',  # Headless
        ' if "', headless, '" == "true" else ',
        '"', '-r ', scenario_file, '"'  # With GUI
    ])

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', gz_args],
        name='gazebo',
        output='screen',
        shell=True
    )

    # ROS2 bridge for camera topic
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Record to ROS2 bag (default method)
    bag_recorder = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/camera/image_raw',
            '/camera/camera_info',
            '/clock',
            '-o', LaunchConfiguration('output_bag'),
            '--max-bag-duration', duration,
        ],
        name='bag_recorder',
        output='screen'
    )

    # Timer to shutdown after duration
    shutdown_timer = TimerAction(
        period=float(LaunchConfiguration('duration')) + 5.0,  # Add 5s buffer
        actions=[
            ExecuteProcess(
                cmd=['echo', '"Recording complete, shutting down..."'],
                shell=True
            ),
            Shutdown(reason='Recording duration completed')
        ]
    )

    # Event handler to start recording after Gazebo is ready
    start_recording_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=gazebo,
            on_start=[
                TimerAction(
                    period=5.0,  # Wait 5 seconds for Gazebo to initialize
                    actions=[bag_recorder]
                )
            ]
        )
    )

    return LaunchDescription([
        # Arguments
        scenario_file_arg,
        metadata_file_arg,
        duration_arg,
        output_video_arg,
        output_bag_arg,
        headless_arg,

        # Processes
        gazebo,
        bridge,

        # Event handlers
        start_recording_handler,
        shutdown_timer,
    ])
