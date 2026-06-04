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
        'base_driver.yaml'
    ])

    imu_config = PathJoinSubstitution([
        FindPackageShare('mechelangelo_imu_driver'),
        'config',
        'imu_driver.yaml'
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
            package='mechelangelo_imu_driver',
            executable='imu_driver',
            name='sensehat_imu',
            parameters=[imu_config],
            output='screen',
        ),

        Node(
            package='mechelangelo_base_driver',
            executable='base_driver',
            name='mechelangelo_base_driver',
            parameters=[base_config],
            output='screen',
        ),

        Node(
            package='mechelangelo_behaviour',
            executable='mechelangelo_behaviour',
            name='mechelangelo_behaviour',
            parameters=[{'stop_distance_m': 1.5}],
            output='screen',
        ),
    ])
