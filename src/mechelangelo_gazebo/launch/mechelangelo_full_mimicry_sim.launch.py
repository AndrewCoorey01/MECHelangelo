#!/usr/bin/env python3
"""
mechelangelo_full_mimicry_sim.launch.py

Starts the complete MECHelangelo simulation pipeline:

  Gazebo robot + arm trajectory plugin
      -> /scan, /imu, /mechelangelo/camera/image_raw,
         /joint_states, /set_joint_trajectory

  Gazebo robot camera
      -> sim_pi4_state_camera --role tracking
      -> /sim_pi4/tracking_state
      -> sim_pi4_state_bridge
      -> /human_tracking + /human_detected
      -> mechelangelo_behaviour

  mechelangelo_behaviour
      -> /interaction_active
      -> holds both arms at arm_down before interaction
      -> forwards verified mimicry poses during interaction

  Laptop camera
      -> sim_pi4_state_camera --role mimicry
      -> /sim_pi4/mimicry_state
      -> sim_pi4_state_bridge
      -> /arm/mimicry_right_pose + /arm/mimicry_left_pose
      -> mechelangelo_behaviour
      -> /arm/right_pose + /arm/left_pose
      -> sim_named_arm_pose_bridge
      -> /set_joint_trajectory

IMPORTANT:
  Do not also launch sim_interaction_gate.py with this file.
  behaviour.cpp is the authoritative publisher of /interaction_active.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # -------------------------------------------------------------------------
    # Launch configurations
    # -------------------------------------------------------------------------
    gazebo_launch_file = LaunchConfiguration("gazebo_launch_file")
    behaviour_executable = LaunchConfiguration("behaviour_executable")

    camera_topic = LaunchConfiguration("camera_topic")
    usb_device = LaunchConfiguration("usb_device")
    right_knn_file = LaunchConfiguration("right_knn_file")
    left_knn_file = LaunchConfiguration("left_knn_file")
    trajectory_topic = LaunchConfiguration("trajectory_topic")

    stop_distance_m = LaunchConfiguration("stop_distance_m")
    approach_arm_pose_name = LaunchConfiguration("approach_arm_pose_name")
    arm_move_time_sec = LaunchConfiguration("arm_move_time_sec")

    behaviour_delay_sec = LaunchConfiguration("behaviour_delay_sec")
    perception_delay_sec = LaunchConfiguration("perception_delay_sec")
    arm_bridge_delay_sec = LaunchConfiguration("arm_bridge_delay_sec")

    start_behaviour = LaunchConfiguration("start_behaviour")
    start_tracking_camera = LaunchConfiguration("start_tracking_camera")
    start_mimicry_camera = LaunchConfiguration("start_mimicry_camera")
    start_arm_bridge = LaunchConfiguration("start_arm_bridge")

    # -------------------------------------------------------------------------
    # Gazebo package launch
    # -------------------------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("mechelangelo_gazebo"),
                "launch",
                gazebo_launch_file,
            ])
        )
    )

    # -------------------------------------------------------------------------
    # Behaviour node
    # -------------------------------------------------------------------------
    behaviour = TimerAction(
        period=behaviour_delay_sec,
        actions=[
            Node(
                package="mechelangelo_behaviour",
                executable=behaviour_executable,
                name="mechelangelo_behaviour",
                output="screen",
                condition=IfCondition(start_behaviour),
                parameters=[{
                    "use_sim_time": True,
                    "stop_distance_m": stop_distance_m,
                    "approach_arm_pose_name": approach_arm_pose_name,
                }],
            )
        ],
    )

    # -------------------------------------------------------------------------
    # Perception pipeline
    # -------------------------------------------------------------------------
    perception = TimerAction(
        period=perception_delay_sec,
        actions=[
            # Gazebo camera -> tracking state.
            Node(
                package="mechelangelo_perception",
                executable="sim_pi4_state_camera",
                name="sim_pi4_tracking_camera",
                output="screen",
                condition=IfCondition(start_tracking_camera),
                arguments=[
                    "--role", "tracking",
                    "--camera", "sim",
                    "--image-topic", camera_topic,
                    "--state-topic", "/sim_pi4/tracking_state",
                    "--activation-topic", "/interaction_active",
                    "--flask-port", "5010",
                    "--width", "640",
                    "--height", "480",
                ],
            ),

            # Tracking state -> /human_tracking and /human_detected.
            #
            # Mimicry pose output is deliberately remapped through behaviour.cpp,
            # which only forwards poses while its verified interaction session
            # is active.
            Node(
                package="mechelangelo_perception",
                executable="sim_pi4_state_bridge",
                name="sim_pi4_state_bridge",
                output="screen",
                parameters=[{
                    "tracking_state_topic": "/sim_pi4/tracking_state",
                    "mimicry_state_topic": "/sim_pi4/mimicry_state",
                    "state_timeout_sec": 1.0,
                }],
                remappings=[
                    ("/arm/right_pose", "/arm/mimicry_right_pose"),
                    ("/arm/left_pose", "/arm/mimicry_left_pose"),
                ],
            ),

            # Laptop camera -> confirmed named arm poses.
            Node(
                package="mechelangelo_perception",
                executable="sim_pi4_state_camera",
                name="sim_pi4_mimicry_camera",
                output="screen",
                condition=IfCondition(start_mimicry_camera),
                arguments=[
                    "--role", "mimicry",
                    "--camera", "usb",
                    "--device", usb_device,
                    "--state-topic", "/sim_pi4/mimicry_state",
                    "--activation-topic", "/interaction_active",
                    "--flask-port", "5011",
                    "--width", "640",
                    "--height", "480",
                    "--right-knn-file", right_knn_file,
                    "--left-knn-file", left_knn_file,
                ],
            ),
        ],
    )

    # -------------------------------------------------------------------------
    # Named arm pose -> JointTrajectory
    # -------------------------------------------------------------------------
    arm_bridge = TimerAction(
        period=arm_bridge_delay_sec,
        actions=[
            Node(
                package="mechelangelo_perception",
                executable="sim_named_arm_pose_bridge",
                name="sim_named_arm_pose_bridge",
                output="screen",
                condition=IfCondition(start_arm_bridge),
                parameters=[{
                    "right_pose_topic": "/arm/right_pose",
                    "left_pose_topic": "/arm/left_pose",
                    "trajectory_topic": trajectory_topic,
                    "move_time_sec": arm_move_time_sec,
                    "suppress_repeated_pose": True,
                }],
            )
        ],
    )

    return LaunchDescription([
        # Main package/executable settings.
        DeclareLaunchArgument(
            "gazebo_launch_file",
            default_value="empty_gallery.launch.py",
            description="Existing launch file in mechelangelo_gazebo/launch.",
        ),
        DeclareLaunchArgument(
            "behaviour_executable",
            default_value="mechelangelo_behaviour",
            description=(
                "Installed executable in mechelangelo_behaviour. "
                "Check with: ros2 pkg executables mechelangelo_behaviour"
            ),
        ),

        # Camera and topic settings.
        DeclareLaunchArgument(
            "camera_topic",
            default_value="/mechelangelo/camera/image_raw",
            description="Gazebo robot camera image topic.",
        ),
        DeclareLaunchArgument(
            "usb_device",
            default_value="0",
            description="Laptop webcam index used for mimicry.",
        ),
        DeclareLaunchArgument(
            "right_knn_file",
            default_value="/home/andy/ros2_ws/src/MECHelangelo/src/mechelangelo_perception/config/pose_knn_right.json",
            description="Laptop-side right-arm KNN training file.",
        ),
        DeclareLaunchArgument(
            "left_knn_file",
            default_value="/home/andy/ros2_ws/src/MECHelangelo/src/mechelangelo_perception/config/pose_knn_left.json",
            description="Laptop-side left-arm KNN training file.",
        ),
        DeclareLaunchArgument(
            "trajectory_topic",
            default_value="/set_joint_trajectory",
            description="Gazebo arm trajectory input topic.",
        ),

        # Behaviour and arm settings.
        DeclareLaunchArgument(
            "stop_distance_m",
            default_value="1.5",
            description="Autonomous wall stopping distance in simulation.",
        ),
        DeclareLaunchArgument(
            "approach_arm_pose_name",
            default_value="arm_down",
            description="Named pose held before interaction starts.",
        ),
        DeclareLaunchArgument(
            "arm_move_time_sec",
            default_value="2.0",
            description="Movement duration for each named arm pose.",
        ),

        # Startup timing.
        DeclareLaunchArgument(
            "behaviour_delay_sec",
            default_value="3.0",
            description="Delay allowing Gazebo sensors to start before behaviour.",
        ),
        DeclareLaunchArgument(
            "perception_delay_sec",
            default_value="4.0",
            description="Delay before starting tracking and laptop camera nodes.",
        ),
        DeclareLaunchArgument(
            "arm_bridge_delay_sec",
            default_value="5.0",
            description="Delay before starting the named arm pose bridge.",
        ),

        # Optional component switches. All are enabled by default.
        DeclareLaunchArgument("start_behaviour", default_value="true"),
        DeclareLaunchArgument("start_tracking_camera", default_value="true"),
        DeclareLaunchArgument("start_mimicry_camera", default_value="true"),
        DeclareLaunchArgument("start_arm_bridge", default_value="true"),

        gazebo,
        behaviour,
        perception,
        arm_bridge,
    ])