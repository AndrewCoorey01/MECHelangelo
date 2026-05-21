from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lidar_config = PathJoinSubstitution([
        FindPackageShare('mechelangelo_lidar_driver'),
        'config',
        'lidar_driver.yaml'
    ])

    base_config = PathJoinSubstitution([
        FindPackageShare('mechelangelo_base_driver'),
        'config',
        'thunderborg_base_driver.yaml'
    ])

    return LaunchDescription([
        Node(
            package='mechelangelo_lidar_driver',
            executable='lidar_driver',
            name='ydlidar_x4_node',
            parameters=[lidar_config],
            output='screen',
        ),

        Node(
            package='mechelangelo_base_driver',
            executable='thunderborg_base_driver',
            name='mechelangelo_base_driver',
            parameters=[base_config],
            output='screen',
        ),

        Node(
            package='mechelangelo_behaviour',
            executable='mechelangelo_behaviour',
            name='mechelangelo_behaviour',
            output='screen',
        ),
    ])