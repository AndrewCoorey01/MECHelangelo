import os
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('mechelangelo_gazebo')
    description_pkg = get_package_share_directory('mechelangelo_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world = os.path.join(gazebo_pkg, 'worlds', 'Gallery_Empty_Room.world')
    sdf_file = os.path.join(gazebo_pkg, 'models', 'mechelangelo_dual_arms', 'model.sdf')
    urdf_file = os.path.join(description_pkg, 'urdf', 'mechelangelo_dual_arms.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')

    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    new_model_path = os.path.join(gazebo_pkg, 'models')

    if existing_model_path:
        gazebo_model_path = new_model_path + ':' + existing_model_path
    else:
        gazebo_model_path = new_model_path

        # Random human model selection
    human_models = [
        os.path.join(gazebo_pkg, 'models', 'human_male_1', 'model.sdf'),
        os.path.join(gazebo_pkg, 'models', 'human_female_1', 'model.sdf'),
        os.path.join(gazebo_pkg, 'models', 'human_female_1_1', 'model.sdf'),
        os.path.join(gazebo_pkg, 'models', 'human_female_2', 'model.sdf'),
        os.path.join(gazebo_pkg, 'models', 'human_female_3', 'model.sdf'),
        os.path.join(gazebo_pkg, 'models', 'human_female_4', 'model.sdf'),
        os.path.join(gazebo_pkg, 'models', 'human_male_1_1', 'model.sdf'),
        os.path.join(gazebo_pkg, 'models', 'human_male_2', 'model.sdf'),
        os.path.join(gazebo_pkg, 'models', 'human_male_3', 'model.sdf'),
        os.path.join(gazebo_pkg, 'models', 'human_male_4', 'model.sdf'),
        # os.path.join(gazebo_pkg, 'models', 'mechelangelo', 'model.sdf'), #easter egg, add if you want a preliminary mechelangelo model to take the place of human model

    ]

    random_human_sdf = random.choice(human_models)

    # Random human spawn pose
    # human_x = str(random.uniform(2.0, 8.0))
    # human_y = str(random.uniform(2.0, 6.0))
    human_x = '6.0'
    human_y = '6.0'
    human_z = '0.05'
    human_yaw = str(random.uniform(-3.14159, 3.14159))

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # gzclient_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
    #     )
    # )

    gzclient_cmd = ExecuteProcess(
    cmd=['gzclient'],
    output='screen'
    )

    # robot_state_publisher_cmd = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'robot_description': robot_description,
    #         'use_sim_time': use_sim_time
    #     }]
    # )

    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mechelangelo',
            '-file', sdf_file,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ],
        output='screen'
    )

    spawn_random_human_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'random_human',
            '-file', random_human_sdf,
            '-x', human_x,
            '-y', human_y,
            '-z', human_z,
            '-Y', human_yaw
        ],
        output='screen'
    )

    ld = LaunchDescription()

    

    ld.add_action(DeclareLaunchArgument('x_pose', default_value='2'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='2'))
    ld.add_action(DeclareLaunchArgument('z_pose', default_value='0.5'))
    ld.add_action(DeclareLaunchArgument('roll', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('pitch', default_value='0'))
    ld.add_action(DeclareLaunchArgument('yaw', default_value='0'))
    # ld.add_action(DeclareLaunchArgument('pitch', default_value='1.5708'))
    # ld.add_action(DeclareLaunchArgument('yaw', default_value='1.5708'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))

    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''))
    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path))

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    # ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(spawn_random_human_cmd)

    return ld