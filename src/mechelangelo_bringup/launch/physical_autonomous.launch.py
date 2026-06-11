# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():
#     lidar_config = PathJoinSubstitution([
#         FindPackageShare('mechelangelo_lidar_driver'),
#         'config',
#         'lidar_driver.yaml'
#     ])

#     base_config = PathJoinSubstitution([
#         FindPackageShare('mechelangelo_base_driver'),
#         'config',
#         'base_driver.yaml'
#     ])

#     imu_config = PathJoinSubstitution([
#         FindPackageShare('mechelangelo_imu_driver'),
#         'config',
#         'imu_driver.yaml'
#     ])

#     return LaunchDescription([
#         Node(
#             package='mechelangelo_lidar_driver',
#             executable='lidar_driver',
#             name='ydlidar_x4_node',
#             parameters=[lidar_config],
#             output='screen',
#         ),

#         Node(
#             package='mechelangelo_imu_driver',
#             executable='imu_driver',
#             name='sensehat_imu',
#             parameters=[imu_config],
#             output='screen',
#         ),

#         Node(
#             package='mechelangelo_base_driver',
#             executable='base_driver',
#             name='mechelangelo_base_driver',
#             parameters=[base_config],
#             output='screen',
#         ),

#         Node(
#             package='mechelangelo_behaviour',
#             executable='mechelangelo_behaviour',
#             name='mechelangelo_behaviour',
#             parameters=[{'stop_distance_m': 1.5}],
#             output='screen',
#         ),
#     ])



#!/usr/bin/env python3

"""
Physical autonomous launch for MECHelangelo.

Startup is deliberately staggered to avoid simultaneous hardware
initialisation, particularly contention between the ThunderBorg and
Sense HAT on I2C bus 1.

Startup sequence:
    0 seconds  - ThunderBorg base driver
    2 seconds  - Sense HAT IMU driver
    4 seconds  - YDLIDAR driver
    12 seconds - Behaviour node

The behaviour node starts last so it does not begin producing movement
commands until the motor driver, IMU and LiDAR have had time to initialise.
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ------------------------------------------------------------
    # Configuration-file paths
    # ------------------------------------------------------------

    lidar_config = PathJoinSubstitution([
        FindPackageShare("mechelangelo_lidar_driver"),
        "config",
        "lidar_driver.yaml",
    ])

    base_config = PathJoinSubstitution([
        FindPackageShare("mechelangelo_base_driver"),
        "config",
        "base_driver.yaml",
    ])

    imu_config = PathJoinSubstitution([
        FindPackageShare("mechelangelo_imu_driver"),
        "config",
        "imu_driver.yaml",
    ])

    # ------------------------------------------------------------
    # ThunderBorg base driver
    #
    # Start this first because both the ThunderBorg and Sense HAT
    # communicate through I2C bus 1.
    # ------------------------------------------------------------

    base_node = Node(
        package="mechelangelo_base_driver",
        executable="base_driver",
        name="mechelangelo_base_driver",
        parameters=[base_config],
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
    )

    # ------------------------------------------------------------
    # Sense HAT IMU
    #
    # Start after the ThunderBorg has completed its initial I2C
    # connection and failsafe configuration.
    # ------------------------------------------------------------

    imu_node = Node(
        package="mechelangelo_imu_driver",
        executable="imu_driver",
        name="sensehat_imu",
        parameters=[imu_config],
        output="screen",
    )

    delayed_imu_node = TimerAction(
        period=2.0,
        actions=[imu_node],
    )

    # ------------------------------------------------------------
    # YDLIDAR X4
    #
    # The LiDAR is USB rather than I2C, but delaying its startup
    # prevents all hardware drivers from initialising simultaneously.
    # ------------------------------------------------------------

    lidar_node = Node(
        package="mechelangelo_lidar_driver",
        executable="lidar_driver",
        name="ydlidar_x4_node",
        parameters=[lidar_config],
        output="screen",
    )

    delayed_lidar_node = TimerAction(
        period=4.0,
        actions=[lidar_node],
    )

    # ------------------------------------------------------------
    # Autonomous behaviour
    #
    # Start last. The LiDAR has recently required approximately
    # 6-7 seconds to connect, so starting behaviour at 12 seconds
    # gives all hardware drivers time to become available.
    # ------------------------------------------------------------

    behaviour_node = Node(
        package="mechelangelo_behaviour",
        executable="mechelangelo_behaviour",
        name="mechelangelo_behaviour",
        parameters=[
            {
                "stop_distance_m": 1.5,
            }
        ],
        output="screen",
    )

    delayed_behaviour_node = TimerAction(
        period=12.0,
        actions=[behaviour_node],
    )

    # ------------------------------------------------------------
    # Launch order
    # ------------------------------------------------------------

    return LaunchDescription([
        base_node,
        delayed_imu_node,
        delayed_lidar_node,
        delayed_behaviour_node,
    ])

