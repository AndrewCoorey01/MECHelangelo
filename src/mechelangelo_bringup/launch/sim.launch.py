from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    description_pkg = FindPackageShare('mechelangelo_description')
    gazebo_pkg = FindPackageShare('gazebo_ros')

    # Load URDF
    robot_description = Command([
        'cat ',
        PathJoinSubstitution([description_pkg, 'urdf', 'mechelangelo.urdf'])
    ])

    return LaunchDescription([

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([gazebo_pkg, 'launch', 'gazebo.launch.py'])
            )
        ),

        # Publish robot state (TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),

        # Spawn robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'mechelangelo'
            ],
            output='screen'
        )
    ])