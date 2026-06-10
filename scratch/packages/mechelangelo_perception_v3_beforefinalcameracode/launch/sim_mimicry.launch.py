#!/usr/bin/env python3
"""
sim_mimicry.launch.py — Arm mimicry tracking using the laptop webcam.

Starts the perception node in MIMICRY mode using the laptop's USB camera.
Because the human model in Gazebo cannot move, this node lets the tester
move their own arms in front of the laptop camera to test the mimicry
pipeline while the simulation is running.

The node:
  - Reads frames from the laptop USB camera (device 0 by default)
  - Detects and locks onto the tester's skeleton using MediaPipe
  - Extracts L/R shoulder and elbow angles + Z-depth deltas
  - Publishes /arm_pose [l_shoulder, l_elbow, l_z_fwd, 0,
                          r_shoulder, r_elbow, r_z_fwd, 0]
  - Also publishes to MQTT arm/angles (for the physical robot path)
  - Does NOT publish /human_tracking (no robot navigation in this mode)

Usage:
    ros2 launch mechelangelo_perception sim_mimicry.launch.py

Optional overrides:
    ros2 launch mechelangelo_perception sim_mimicry.launch.py \\
        usb_device:=0 \\
        arm_topic:=/arm_pose \\
        flask_port:=5001

Debug stream (annotated webcam view with angle overlays):
    http://localhost:5001
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    usb_device = LaunchConfiguration('usb_device')
    arm_topic = LaunchConfiguration('arm_topic')
    flask_port = LaunchConfiguration('flask_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'usb_device',
            default_value='0',
            description='USB camera device index (0 = first laptop webcam)',
        ),
        DeclareLaunchArgument(
            'arm_topic',
            default_value='/arm_pose',
            description='ROS topic to publish arm joint angles on',
        ),
        DeclareLaunchArgument(
            'flask_port',
            default_value='5001',
            description=(
                'Port for the Flask MJPEG debug stream. '
                'Defaults to 5001 so it does not clash with sim_tracking on 5000 '
                'when both are running together.'
            ),
        ),

        Node(
            package='mechelangelo_perception',
            executable='pose_tracking_human_ros',
            name='human_pose_mimicry',
            output='screen',
            arguments=[
                '--mode', 'mimicry',
                '--camera', 'usb',
                '--device', usb_device,
                '--arm-topic', arm_topic,
                '--flask-port', flask_port,
            ],
        ),
    ])
