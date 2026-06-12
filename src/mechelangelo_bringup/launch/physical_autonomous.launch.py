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
            package='mechelangelo_perception',
            executable='dual_ultrasonic',
            name='dual_ultrasonic_node',
            output='screen',
            parameters=[{
                'left_trigger_gpio': 17,
                'left_echo_gpio': 22,
                'right_trigger_gpio': 27,
                'right_echo_gpio': 23,
                'left_topic': '/ultrasonic/left',
                'right_topic': '/ultrasonic/right',
                'left_frame_id': 'ultrasonic_left',
                'right_frame_id': 'ultrasonic_right',
                'min_range_m': 0.02,
                'max_range_m': 4.0,
                'field_of_view_rad': 0.45,
                'inter_trigger_period_s': 0.07,
                'echo_timeout_s': 0.03,
                'median_window': 5,
            }],
        ),

        Node(
            package='mechelangelo_behaviour',
            executable='mechelangelo_behaviour_physical',
            name='mechelangelo_behaviour',
            parameters=[{'stop_distance_m': 0.75}],
            output='screen',
        ),
    ])
