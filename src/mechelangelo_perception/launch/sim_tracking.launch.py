#!/usr/bin/env python3
"""
sim_tracking.launch.py — Approach tracking in simulation.

Starts the perception node in TRACKING mode using the robot's simulated
Gazebo camera.  The node:
  - Subscribes to /mechelangelo/camera/image_raw (Gazebo camera topic)
  - Detects and locks onto a human using MediaPipe
  - Publishes /human_tracking [detected, centre_offset, distance_m]
  - Does NOT extract or publish arm joint angles

The mechelangelo_behaviour node reads /human_tracking and drives /cmd_vel
to steer the robot toward the human.

Usage:
    ros2 launch mechelangelo_perception sim_tracking.launch.py

Optional overrides:
    ros2 launch mechelangelo_perception sim_tracking.launch.py \\
        camera_topic:=/mechelangelo/camera/image_raw \\
        flask_port:=5000

Debug stream (annotated camera view):
    http://localhost:5000
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_topic = LaunchConfiguration('camera_topic')
    tracking_topic = LaunchConfiguration('tracking_topic')
    flask_port = LaunchConfiguration('flask_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/mechelangelo/camera/image_raw',
            description='ROS image topic from the Gazebo simulation camera',
        ),
        DeclareLaunchArgument(
            'tracking_topic',
            default_value='/human_tracking',
            description='ROS topic to publish human tracking data on',
        ),
        DeclareLaunchArgument(
            'flask_port',
            default_value='5000',
            description='Port for the Flask MJPEG debug stream',
        ),

        Node(
            package='mechelangelo_perception',
            executable='pose_tracking_human_ros',
            name='human_pose_tracking',
            output='screen',
            arguments=[
                '--mode', 'tracking',
                '--camera', 'sim',
                '--topic', camera_topic,
                '--tracking-topic', tracking_topic,
                '--flask-port', flask_port,
            ],
        ),
    ])
