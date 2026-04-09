import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('mechelangelo_gazebo')

    sdf_file = os.path.join(pkg, 'models', 'mechelangelo', 'model.sdf')

    x_pose = LaunchConfiguration('x_pose', default='3.125')
    y_pose = LaunchConfiguration('y_pose', default='2.85')
    z_pose = LaunchConfiguration('z_pose', default='0.2')
    yaw = LaunchConfiguration('yaw', default='4.712')  # radians

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mechelangelo',
            '-file', sdf_file,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('x_pose', default_value=x_pose.perform({})))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value=y_pose.perform({})))
    ld.add_action(DeclareLaunchArgument('z_pose', default_value=z_pose.perform({})))
    ld.add_action(DeclareLaunchArgument('yaw', default_value=yaw.perform({})))

    ld.add_action(spawn)
    return ld
