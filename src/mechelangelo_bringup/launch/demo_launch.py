from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Pi 4 bridge — provides /human_detected and /human_tracking from
        # the Pi 4 camera, and forwards /arm/mimicry_*_pose to the behaviour.
        Node(
            package='mechelangelo_perception',
            executable='pi4_bridge',
            name='pi4_bridge',
            output='screen',
        ),

        # Demo behaviour — stationary robot, arm mimicry and voice only.
        # No LiDAR, base driver, or IMU are launched: the robot does not move.
        Node(
            package='mechelangelo_behaviour',
            executable='mechelangelo_demo_behaviour',
            name='mechelangelo_demo_behaviour',
            output='screen',
        ),
    ])
