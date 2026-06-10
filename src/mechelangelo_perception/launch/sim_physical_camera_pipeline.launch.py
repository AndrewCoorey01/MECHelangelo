# #!/usr/bin/env python3
# """
# sim_physical_camera_pipeline.launch.py

# Simulation pipeline that mirrors the current physical camera architecture.

# Use this instead of the old sim_combined.launch.py when you want simulation to
# test the current Pi 4 camera/state interface.

# Pipeline
# --------
# Gazebo robot camera
#   -> sim_pi4_state_camera.py --role tracking
#   -> /sim_pi4/tracking_state
#   -> sim_pi4_state_bridge.py
#   -> /human_tracking + /human_detected
#   -> behaviour.cpp drives robot toward human using LiDAR distance

# /scan + /human_tracking
#   -> sim_interaction_gate.py
#   -> /interaction_active

# Laptop USB camera
#   -> sim_pi4_state_camera.py --role mimicry
#   -> waits until /interaction_active == true
#   -> publishes confirmed named poses in /sim_pi4/mimicry_state
#   -> sim_pi4_state_bridge.py
#   -> /arm/right_pose + /arm/left_pose
#   -> sim_named_arm_pose_bridge.py
#   -> Gazebo arm joints

# Debug streams
# -------------
# Tracking/Gazebo camera: http://localhost:5010
# Laptop mimicry camera:  http://localhost:5011

# Typical usage
# -------------
# Start Gazebo + robot + behaviour first, then:

#   ros2 launch mechelangelo_perception sim_physical_camera_pipeline.launch.py

# With a different laptop webcam:

#   ros2 launch mechelangelo_perception sim_physical_camera_pipeline.launch.py usb_device:=1
# """

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, TimerAction
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node


# def generate_launch_description():
#     camera_topic = LaunchConfiguration("camera_topic")
#     scan_topic = LaunchConfiguration("scan_topic")
#     usb_device = LaunchConfiguration("usb_device")
#     model_name = LaunchConfiguration("model_name")
#     right_knn_file = LaunchConfiguration("right_knn_file")
#     left_knn_file = LaunchConfiguration("left_knn_file")

#     return LaunchDescription([
#         DeclareLaunchArgument(
#             "camera_topic",
#             default_value="/mechelangelo/camera/image_raw",
#             description="Gazebo robot camera image topic used for human approach tracking.",
#         ),
#         DeclareLaunchArgument(
#             "scan_topic",
#             default_value="/scan",
#             description="LaserScan topic used by the interaction gate.",
#         ),
#         DeclareLaunchArgument(
#             "usb_device",
#             default_value="0",
#             description="Laptop/USB camera index used for arm mimicry.",
#         ),
#         DeclareLaunchArgument(
#             "model_name",
#             default_value="mechelangelo",
#             description="Gazebo model name for the robot.",
#         ),
#         DeclareLaunchArgument(
#             "right_knn_file",
#             default_value="/home/pi/pose_knn_right.json",
#             description="KNN training file for right arm pose names.",
#         ),
#         DeclareLaunchArgument(
#             "left_knn_file",
#             default_value="/home/pi/pose_knn_left.json",
#             description="KNN training file for left arm pose names.",
#         ),

#         # Node 1: Gazebo camera -> physical-style tracking state.
#         Node(
#             package="mechelangelo_perception",
#             executable="sim_pi4_state_camera",
#             name="sim_pi4_tracking_camera",
#             output="screen",
#             arguments=[
#                 "--role", "tracking",
#                 "--camera", "sim",
#                 "--image-topic", camera_topic,
#                 "--state-topic", "/sim_pi4/tracking_state",
#                 "--activation-topic", "/interaction_active",
#                 "--flask-port", "5010",
#                 "--width", "640",
#                 "--height", "480",
#             ],
#         ),

#         # Node 2: Combine physical-style tracking/mimicry state into the same
#         # ROS topics used by the physical behaviour pipeline.
#         Node(
#             package="mechelangelo_perception",
#             executable="sim_pi4_state_bridge",
#             name="sim_pi4_state_bridge",
#             output="screen",
#             parameters=[{
#                 "tracking_state_topic": "/sim_pi4/tracking_state",
#                 "mimicry_state_topic": "/sim_pi4/mimicry_state",
#                 "state_timeout_sec": 1.0,
#             }],
#         ),

#         # Node 3: Detect when the robot has reached the human using LiDAR and
#         # unlock laptop-camera mimicry.
#         Node(
#             package="mechelangelo_perception",
#             executable="sim_interaction_gate",
#             name="sim_interaction_gate",
#             output="screen",
#             parameters=[{
#                 "human_tracking_topic": "/human_tracking",
#                 "scan_topic": scan_topic,
#                 "interaction_topic": "/interaction_active",
#                 "target_distance_m": 1.65,
#                 "target_tolerance_m": 0.25,
#                 "centred_offset_threshold": 0.20,
#                 "good_cycles_required": 5,
#             }],
#         ),

#         # Node 4: Laptop camera -> physical-style confirmed named arm poses.
#         # This node can run from startup; it will not publish poses until
#         # /interaction_active is true.
#         Node(
#             package="mechelangelo_perception",
#             executable="sim_pi4_state_camera",
#             name="sim_pi4_mimicry_camera",
#             output="screen",
#             arguments=[
#                 "--role", "mimicry",
#                 "--camera", "usb",
#                 "--device", usb_device,
#                 "--state-topic", "/sim_pi4/mimicry_state",
#                 "--activation-topic", "/interaction_active",
#                 "--flask-port", "5011",
#                 "--width", "640",
#                 "--height", "480",
#                 "--right-knn-file", right_knn_file,
#                 "--left-knn-file", left_knn_file,
#             ],
#         ),

#         # Node 5: Named poses -> Gazebo arm joints.
#         # Delayed to give Gazebo time to finish spawning the robot.
#         TimerAction(
#             period=3.0,
#             actions=[
#                 Node(
#                     package="mechelangelo_perception",
#                     executable="sim_named_arm_pose_bridge",
#                     name="sim_named_arm_pose_bridge",
#                     output="screen",
#                     parameters=[{
#                         "model_name": model_name,
#                     }],
#                 ),
#             ],
#         ),
#     ])


#!/usr/bin/env python3
"""
sim_physical_camera_pipeline.launch.py

Simulation pipeline that mirrors the current physical camera architecture.

Start Gazebo + robot + behaviour first, then launch this file.

Pipeline
--------
Gazebo robot camera
  -> sim_pi4_state_camera.py --role tracking
  -> /sim_pi4/tracking_state
  -> sim_pi4_state_bridge.py
  -> /human_tracking + /human_detected
  -> behaviour.cpp drives robot toward human using LiDAR distance

/scan + /human_tracking
  -> sim_interaction_gate.py
  -> /interaction_active

Laptop USB camera
  -> sim_pi4_state_camera.py --role mimicry
  -> waits until /interaction_active == true
  -> publishes confirmed named poses in /sim_pi4/mimicry_state
  -> sim_pi4_state_bridge.py
  -> /arm/right_pose + /arm/left_pose
  -> sim_named_arm_pose_bridge.py
  -> /set_joint_trajectory
  -> Gazebo arm joints

Debug streams
-------------
Tracking/Gazebo camera: http://localhost:5010
Laptop mimicry camera:  http://localhost:5011
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_topic = LaunchConfiguration("camera_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    usb_device = LaunchConfiguration("usb_device")
    model_name = LaunchConfiguration("model_name")
    right_knn_file = LaunchConfiguration("right_knn_file")
    left_knn_file = LaunchConfiguration("left_knn_file")
    trajectory_topic = LaunchConfiguration("trajectory_topic")
    arm_move_time_sec = LaunchConfiguration("arm_move_time_sec")
    arm_bridge_delay_sec = LaunchConfiguration("arm_bridge_delay_sec")

    return LaunchDescription([
        # ---------------------------------------------------------------------
        # Launch arguments
        # ---------------------------------------------------------------------
        DeclareLaunchArgument(
            "camera_topic",
            default_value="/mechelangelo/camera/image_raw",
            description="Gazebo robot camera image topic used for human approach tracking.",
        ),
        DeclareLaunchArgument(
            "scan_topic",
            default_value="/scan",
            description="LaserScan topic used by the interaction gate.",
        ),
        DeclareLaunchArgument(
            "usb_device",
            default_value="0",
            description="Laptop/USB camera index used for arm mimicry.",
        ),
        DeclareLaunchArgument(
            "model_name",
            default_value="mechelangelo",
            description="Gazebo model name for the robot.",
        ),
        DeclareLaunchArgument(
            "right_knn_file",
            default_value="/home/andy/pose_knn_right.json",
            description="KNN training file for right arm pose names.",
        ),
        DeclareLaunchArgument(
            "left_knn_file",
            default_value="/home/andy/pose_knn_left.json",
            description="KNN training file for left arm pose names.",
        ),
        DeclareLaunchArgument(
            "trajectory_topic",
            default_value="/set_joint_trajectory",
            description="JointTrajectory topic used by the Gazebo arm controller/plugin.",
        ),
        DeclareLaunchArgument(
            "arm_move_time_sec",
            default_value="2.0",
            description="Time allowed for each named arm pose movement.",
        ),
        DeclareLaunchArgument(
            "arm_bridge_delay_sec",
            default_value="3.0",
            description="Delay before starting the named-pose-to-arm-trajectory bridge.",
        ),

        # ---------------------------------------------------------------------
        # Node 1: Gazebo camera -> physical-style tracking state
        # ---------------------------------------------------------------------
        Node(
            package="mechelangelo_perception",
            executable="sim_pi4_state_camera",
            name="sim_pi4_tracking_camera",
            output="screen",
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

        # ---------------------------------------------------------------------
        # Node 2: Physical-style state bridge
        #
        # tracking state -> /human_tracking + /human_detected
        # mimicry state  -> /arm/right_pose + /arm/left_pose
        # ---------------------------------------------------------------------
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
        ),

        # ---------------------------------------------------------------------
        # Node 3: Interaction gate
        #
        # Uses LiDAR + human tracking to decide when mimicry is allowed.
        # Publishes /interaction_active.
        # ---------------------------------------------------------------------
        Node(
            package="mechelangelo_perception",
            executable="sim_interaction_gate",
            name="sim_interaction_gate",
            output="screen",
            parameters=[{
                "human_tracking_topic": "/human_tracking",
                "scan_topic": scan_topic,
                "interaction_topic": "/interaction_active",
                "target_distance_m": 1.65,
                "target_tolerance_m": 0.25,
                "centred_offset_threshold": 0.20,
                "good_cycles_required": 5,
            }],
        ),

        # ---------------------------------------------------------------------
        # Node 4: Laptop camera -> physical-style mimicry state
        #
        # This can run from startup, but it should only publish useful mimicry
        # output once /interaction_active is true.
        # ---------------------------------------------------------------------
        Node(
            package="mechelangelo_perception",
            executable="sim_pi4_state_camera",
            name="sim_pi4_mimicry_camera",
            output="screen",
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

        # ---------------------------------------------------------------------
        # Node 5: Named poses -> JointTrajectory arm command
        #
        # This bridge must publish trajectory_msgs/msg/JointTrajectory to the
        # same topic that you successfully tested manually:
        #
        #   /set_joint_trajectory
        #
        # It should NOT wait for the old Gazebo service path.
        # ---------------------------------------------------------------------
        TimerAction(
            period=arm_bridge_delay_sec,
            actions=[
                Node(
                    package="mechelangelo_perception",
                    executable="sim_named_arm_pose_bridge",
                    name="sim_named_arm_pose_bridge",
                    output="screen",
                    parameters=[{
                        "model_name": model_name,
                        "right_pose_topic": "/arm/right_pose",
                        "left_pose_topic": "/arm/left_pose",
                        "trajectory_topic": trajectory_topic,
                        "move_time_sec": arm_move_time_sec,
                    }],
                ),
            ],
        ),
    ])