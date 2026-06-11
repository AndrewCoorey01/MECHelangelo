# import os
# import random

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import (
#     DeclareLaunchArgument,
#     ExecuteProcess,
#     IncludeLaunchDescription,
#     SetEnvironmentVariable,
#     TimerAction,
# )
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node


# def generate_launch_description():
#     gazebo_pkg = get_package_share_directory('mechelangelo_gazebo')
#     description_pkg = get_package_share_directory('mechelangelo_description')
#     pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

#     world = os.path.join(gazebo_pkg, 'worlds', 'Gallery_Empty_Room.world')
#     sdf_file = os.path.join(gazebo_pkg, 'models', 'mechelangelo_final', 'model.sdf')
#     urdf_file = os.path.join(description_pkg, 'urdf', 'mechelangelo_final.urdf')

#     with open(urdf_file, 'r') as f:
#         robot_description = f.read()

#     x_pose = LaunchConfiguration('x_pose')
#     y_pose = LaunchConfiguration('y_pose')
#     z_pose = LaunchConfiguration('z_pose')
#     yaw = LaunchConfiguration('yaw')
#     roll = LaunchConfiguration('roll')
#     pitch = LaunchConfiguration('pitch')
#     use_sim_time = LaunchConfiguration('use_sim_time')

#     existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
#     new_model_path = os.path.join(gazebo_pkg, 'models')
#     gazebo_model_path = (new_model_path + ':' + existing_model_path
#                          if existing_model_path else new_model_path)

#     human_models = [
#         os.path.join(gazebo_pkg, 'models', 'human_male_1', 'model.sdf'),
#         os.path.join(gazebo_pkg, 'models', 'human_female_1', 'model.sdf'),
#         os.path.join(gazebo_pkg, 'models', 'human_female_1_1', 'model.sdf'),
#         os.path.join(gazebo_pkg, 'models', 'human_female_2', 'model.sdf'),
#         os.path.join(gazebo_pkg, 'models', 'human_female_3', 'model.sdf'),
#         os.path.join(gazebo_pkg, 'models', 'human_female_4', 'model.sdf'),
#         os.path.join(gazebo_pkg, 'models', 'human_male_1_1', 'model.sdf'),
#         os.path.join(gazebo_pkg, 'models', 'human_male_2', 'model.sdf'),
#         os.path.join(gazebo_pkg, 'models', 'human_male_3', 'model.sdf'),
#         os.path.join(gazebo_pkg, 'models', 'human_male_4', 'model.sdf'),
#     ]
#     random_human_sdf = random.choice(human_models)
#     human_x = '6.0'
#     human_y = '6.0'
#     human_z = '0.05'
#     human_yaw = str(random.uniform(-3.14159, 3.14159))

#     gzserver_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
#         launch_arguments={'world': world}.items()
#     )

#     gzclient_cmd = ExecuteProcess(cmd=['gzclient'], output='screen')

#     robot_state_publisher_cmd = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[{
#             'robot_description': robot_description,
#             'use_sim_time': use_sim_time,
#         }]
#     )

#     spawn_robot_cmd = TimerAction(
#         period=3.0,
#         actions=[
#             Node(
#                 package='gazebo_ros',
#                 executable='spawn_entity.py',
#                 arguments=[
#                     '-entity', 'mechelangelo',
#                     '-file', sdf_file,
#                     '-x', x_pose,
#                     '-y', y_pose,
#                     '-z', z_pose,
#                     '-R', roll,
#                     '-P', pitch,
#                     '-Y', yaw,
#                 ],
#                 output='screen',
#             )
#         ]
#     )

#     spawn_random_human_cmd = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=[
#             '-entity', 'random_human',
#             '-file', random_human_sdf,
#             '-x', human_x,
#             '-y', human_y,
#             '-z', human_z,
#             '-Y', human_yaw,
#         ],
#         output='screen'
#     )

#     ld = LaunchDescription()

#     ld.add_action(DeclareLaunchArgument('x_pose', default_value='2'))
#     ld.add_action(DeclareLaunchArgument('y_pose', default_value='2'))
#     ld.add_action(DeclareLaunchArgument('z_pose', default_value='0.0'))
#     ld.add_action(DeclareLaunchArgument('roll', default_value='0.0'))
#     ld.add_action(DeclareLaunchArgument('pitch', default_value='0'))
#     ld.add_action(DeclareLaunchArgument('yaw', default_value='0'))
#     ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))

#     ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''))
#     ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path))

#     ld.add_action(gzserver_cmd)
#     ld.add_action(gzclient_cmd)
#     ld.add_action(robot_state_publisher_cmd)
#     ld.add_action(spawn_robot_cmd)
#     ld.add_action(spawn_random_human_cmd)

#     return ld


#!/usr/bin/env python3

import os
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_pkg = get_package_share_directory("mechelangelo_gazebo")
    description_pkg = get_package_share_directory("mechelangelo_description")
    gazebo_ros_pkg = get_package_share_directory("gazebo_ros")

    # -------------------------------------------------------------------------
    # Model and world files
    # -------------------------------------------------------------------------
    world_file = os.path.join(
        gazebo_pkg,
        "worlds",
        "Gallery_Empty_Room.world",
    )

    robot_sdf_file = os.path.join(
        gazebo_pkg,
        "models",
        "mechelangelo_final",
        "model.sdf",
    )

    robot_urdf_file = os.path.join(
        description_pkg,
        "urdf",
        "mechelangelo_final.urdf",
    )

    with open(robot_urdf_file, "r", encoding="utf-8") as urdf:
        robot_description = urdf.read()

    # -------------------------------------------------------------------------
    # Launch arguments
    # -------------------------------------------------------------------------
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # -------------------------------------------------------------------------
    # Gazebo model path
    # -------------------------------------------------------------------------
    existing_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    package_model_path = os.path.join(gazebo_pkg, "models")

    if existing_model_path:
        gazebo_model_path = (
            package_model_path + ":" + existing_model_path
        )
    else:
        gazebo_model_path = package_model_path

    # -------------------------------------------------------------------------
    # Random human model
    # -------------------------------------------------------------------------
    human_models = [
        os.path.join(
            gazebo_pkg,
            "models",
            "human_male_1",
            "model.sdf",
        ),
        os.path.join(
            gazebo_pkg,
            "models",
            "human_female_1",
            "model.sdf",
        ),
        os.path.join(
            gazebo_pkg,
            "models",
            "human_female_1_1",
            "model.sdf",
        ),
        os.path.join(
            gazebo_pkg,
            "models",
            "human_female_2",
            "model.sdf",
        ),
        os.path.join(
            gazebo_pkg,
            "models",
            "human_female_3",
            "model.sdf",
        ),
        os.path.join(
            gazebo_pkg,
            "models",
            "human_female_4",
            "model.sdf",
        ),
        os.path.join(
            gazebo_pkg,
            "models",
            "human_male_1_1",
            "model.sdf",
        ),
        os.path.join(
            gazebo_pkg,
            "models",
            "human_male_2",
            "model.sdf",
        ),
        os.path.join(
            gazebo_pkg,
            "models",
            "human_male_3",
            "model.sdf",
        ),
        os.path.join(
            gazebo_pkg,
            "models",
            "human_male_4",
            "model.sdf",
        ),
    ]

    random_human_sdf = random.choice(human_models)

    human_x = "6.0"
    human_y = "6.0"
    human_z = "0.05"
    human_yaw = str(random.uniform(-3.14159, 3.14159))

    # -------------------------------------------------------------------------
    # Start Gazebo server
    # -------------------------------------------------------------------------
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_pkg,
                "launch",
                "gzserver.launch.py",
            )
        ),
        launch_arguments={
            "world": world_file,
        }.items(),
    )

    # -------------------------------------------------------------------------
    # Start Gazebo client
    # -------------------------------------------------------------------------
    gazebo_client = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
    )

    # -------------------------------------------------------------------------
    # Robot state publisher
    #
    # Gazebo uses the SDF for physics.
    # RViz and TF use the URDF loaded here.
    # -------------------------------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # -------------------------------------------------------------------------
    # Spawn random human
    # -------------------------------------------------------------------------
    spawn_random_human = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_random_human",
        arguments=[
            "-entity",
            "random_human",
            "-file",
            random_human_sdf,
            "-x",
            human_x,
            "-y",
            human_y,
            "-z",
            human_z,
            "-Y",
            human_yaw,
        ],
        output="screen",
    )

    # -------------------------------------------------------------------------
    # Spawn MECHelangelo
    #
    # Delayed to give Gazebo time to load the world and spawn service.
    # -------------------------------------------------------------------------
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_mechelangelo",
                arguments=[
                    "-entity",
                    "mechelangelo",
                    "-file",
                    robot_sdf_file,
                    "-x",
                    x_pose,
                    "-y",
                    y_pose,
                    "-z",
                    z_pose,
                    "-R",
                    roll,
                    "-P",
                    pitch,
                    "-Y",
                    yaw,
                ],
                output="screen",
            )
        ],
    )

    # -------------------------------------------------------------------------
    # Initial arm-down pose
    #
    # Joint order:
    #   left_joint1
    #   left_joint2
    #   left_joint3
    #   left_hand_joint
    #   right_joint1
    #   right_joint2
    #   right_joint3
    #   right_hand_joint
    #
    # Calibrated arm-down position:
    #   [0.0, -1.484, 0.0, 0.0] per arm
    #
    # The command is delayed until after the robot and its Gazebo plugins have
    # loaded. The zero trajectory header timestamp tells Gazebo to execute it
    # immediately.
    # -------------------------------------------------------------------------
    startup_arm_down = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "--once",
                    "/set_joint_trajectory",
                    "trajectory_msgs/msg/JointTrajectory",
                    (
                        "{"
                        "header: {frame_id: 'base_link'}, "
                        "joint_names: ["
                        "'left_joint1',"
                        "'left_joint2',"
                        "'left_joint3',"
                        "'left_hand_joint',"
                        "'right_joint1',"
                        "'right_joint2',"
                        "'right_joint3',"
                        "'right_hand_joint'"
                        "], "
                        "points: [{"
                        "positions: ["
                        "0.0, -1.484, 0.0, 0.0, "
                        "0.0, -1.484, 0.0, 0.0"
                        "], "
                        "time_from_start: {sec: 2, nanosec: 0}"
                        "}]"
                        "}"
                    ),
                ],
                output="screen",
                emulate_tty=True,
            )
        ],
    )

    # -------------------------------------------------------------------------
    # Launch description
    # -------------------------------------------------------------------------
    launch_description = LaunchDescription()

    launch_description.add_action(
        DeclareLaunchArgument(
            "x_pose",
            default_value="2.0",
            description="Initial robot X position in Gazebo.",
        )
    )

    launch_description.add_action(
        DeclareLaunchArgument(
            "y_pose",
            default_value="2.0",
            description="Initial robot Y position in Gazebo.",
        )
    )

    launch_description.add_action(
        DeclareLaunchArgument(
            "z_pose",
            default_value="0.0",
            description="Initial robot Z position in Gazebo.",
        )
    )

    launch_description.add_action(
        DeclareLaunchArgument(
            "roll",
            default_value="0.0",
            description="Initial robot roll.",
        )
    )

    launch_description.add_action(
        DeclareLaunchArgument(
            "pitch",
            default_value="0.0",
            description="Initial robot pitch.",
        )
    )

    launch_description.add_action(
        DeclareLaunchArgument(
            "yaw",
            default_value="0.0",
            description="Initial robot yaw.",
        )
    )

    launch_description.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use the Gazebo simulation clock.",
        )
    )

    launch_description.add_action(
        SetEnvironmentVariable(
            "GAZEBO_MODEL_DATABASE_URI",
            "",
        )
    )

    launch_description.add_action(
        SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH",
            gazebo_model_path,
        )
    )

    launch_description.add_action(gazebo_server)
    launch_description.add_action(gazebo_client)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(spawn_random_human)
    launch_description.add_action(spawn_robot)
    launch_description.add_action(startup_arm_down)

    return launch_description
