#!/usr/bin/env python3
"""
sim_combined.launch.py — Full simulation perception pipeline.

Starts BOTH perception nodes simultaneously for end-to-end sim testing:

  Node 1: human_pose_tracking  (TRACKING mode, sim camera)
    - Gazebo camera → human detection → /human_tracking
    - The behaviour node uses /human_tracking to approach the human
    - Debug stream on :5000

  Node 2: human_pose_mimicry   (MIMICRY mode, USB/laptop camera)
    - Laptop webcam → tester's skeleton → /arm_pose
    - The arm controller uses /arm_pose to move the robot's arms
    - Debug stream on :5001

This replicates the intended behaviour for the physical robot in sim:
  - Robot camera tracks and approaches the sim human model
  - When at the human, the tester's arm movements (via laptop) drive mimicry
  - The two nodes are independent — no mode-switching needed

Usage:
    ros2 launch mechelangelo_perception sim_combined.launch.py

With overrides:
    ros2 launch mechelangelo_perception sim_combined.launch.py \\
        usb_device:=1 \\
        camera_topic:=/mechelangelo/camera/image_raw

Debug streams:
    Approach tracking (sim camera):  http://localhost:5000
    Arm mimicry (laptop camera):     http://localhost:5001
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_topic = LaunchConfiguration('camera_topic')
    tracking_topic = LaunchConfiguration('tracking_topic')
    arm_topic = LaunchConfiguration('arm_topic')
    usb_device = LaunchConfiguration('usb_device')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/mechelangelo/camera/image_raw',
            description='Gazebo camera ROS image topic for the tracking node',
        ),
        DeclareLaunchArgument(
            'tracking_topic',
            default_value='/human_tracking',
            description='ROS topic for human approach tracking data',
        ),
        DeclareLaunchArgument(
            'arm_topic',
            default_value='/arm_pose',
            description='ROS topic for arm joint angles (mimicry output)',
        ),
        DeclareLaunchArgument(
            'usb_device',
            default_value='0',
            description='Laptop USB camera device index for mimicry node',
        ),

        # ── Node 1: Approach tracking via Gazebo robot camera ─────────
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
                '--flask-port', '5000',
            ],
        ),

        # ── Node 2: Arm mimicry via laptop webcam ─────────────────────
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
                '--flask-port', '5001',
            ],
        ),
    ])
