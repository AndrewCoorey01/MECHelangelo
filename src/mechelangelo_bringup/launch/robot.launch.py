#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    launch_lidar = LaunchConfiguration('launch_lidar')
    launch_camera = LaunchConfiguration('launch_camera')
    launch_base_driver = LaunchConfiguration('launch_base_driver')

    description_pkg = FindPackageShare('mechelangelo_description')
    base_driver_pkg = FindPackageShare('mechelangelo_base_driver')

    robot_description_file = PathJoinSubstitution([
        description_pkg,
        'urdf',
        'mechelangelo.urdf.xacro'
    ])

    base_driver_params = PathJoinSubstitution([
        base_driver_pkg,
        'config',
        'base_driver.yaml'
    ])

    robot_description = ParameterValue(
        Command(['xacro ', robot_description_file]),
        value_type=str
    )

    # Adjust this if your lidar package or launch file is different.
    # For YDLIDAR packages, the exact launch filename can vary.
    ydlidar_launch_file = PathJoinSubstitution([
        FindPackageShare('ydlidar_ros2_driver'),
        'launch',
        'ydlidar_launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the robot'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),

        DeclareLaunchArgument(
            'launch_lidar',
            default_value='true',
            description='Start lidar driver'
        ),

        DeclareLaunchArgument(
            'launch_camera',
            default_value='true',
            description='Start camera driver'
        ),

        DeclareLaunchArgument(
            'launch_base_driver',
            default_value='true',
            description='Start physical motor/encoder base driver'
        ),

        PushRosNamespace(namespace),

        # Publishes robot_description and fixed transforms from the URDF.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'robot_description': robot_description,
                }
            ],
        ),

        # Physical base driver.
        # This is the MECHelangelo equivalent of turtlebot3_node/turtlebot3_ros.
        Node(
            condition=IfCondition(launch_base_driver),
            package='mechelangelo_base_driver',
            executable='base_driver',
            name='mechelangelo_base_driver',
            output='screen',
            parameters=[
                base_driver_params,
                {'use_sim_time': use_sim_time},
            ],
        ),

        # Lidar driver.
        # Change this section if your YDLIDAR package uses a different launch file.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ydlidar_launch_file),
            condition=IfCondition(launch_lidar),
        ),

        # Camera driver.
        # This replaces:
        # ros2 run camera_ros camera_node --ros-args -p format:=MJPEG -p width:=640 -p height:=480
        Node(
            condition=IfCondition(launch_camera),
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                {
                    'format': 'MJPEG',
                    'width': 640,
                    'height': 480,
                    'use_sim_time': use_sim_time,
                }
            ],
        ),
    ])