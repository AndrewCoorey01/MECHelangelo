from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('mechelangelo_imu_driver'), 'config', 'imu_driver.yaml']
    )

    imu_node = Node(
        package='mechelangelo_imu_driver',
        executable='imu_driver',
        name='sensehat_imu',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([imu_node])
