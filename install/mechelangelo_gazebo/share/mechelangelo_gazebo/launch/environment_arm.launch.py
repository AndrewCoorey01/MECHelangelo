# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node


# def generate_launch_description():
#     gazebo_pkg = get_package_share_directory('mechelangelo_gazebo')
#     pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

#     world = os.path.join(gazebo_pkg, 'worlds', 'Gallery.world')
#     sdf_file = os.path.join(gazebo_pkg, 'models', 'mechelangelo', 'model.sdf')

#     x_pose = LaunchConfiguration('x_pose')
#     y_pose = LaunchConfiguration('y_pose')
#     z_pose = LaunchConfiguration('z_pose')
#     yaw = LaunchConfiguration('yaw')

#     existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
#     new_model_path = os.path.join(gazebo_pkg, 'models')

#     if existing_model_path:
#         gazebo_model_path = new_model_path + ':' + existing_model_path
#     else:
#         gazebo_model_path = new_model_path

#     gzserver_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
#         ),
#         launch_arguments={'world': world}.items()
#     )

#     gzclient_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
#         )
#     )

#     spawn_robot_cmd = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=[
#             '-entity', 'mechelangelo',
#             '-file', sdf_file,
#             '-x', x_pose,
#             '-y', y_pose,
#             '-z', z_pose,
#             '-Y', yaw
#         ],
#         output='screen'
#     )

#     ld = LaunchDescription()

#     ld.add_action(DeclareLaunchArgument('x_pose', default_value='1.5'))
#     ld.add_action(DeclareLaunchArgument('y_pose', default_value='1.5'))
#     ld.add_action(DeclareLaunchArgument('z_pose', default_value='0.2'))
#     ld.add_action(DeclareLaunchArgument('yaw', default_value='4.712'))

#     ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''))
#     ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path))

#     ld.add_action(gzserver_cmd)
#     ld.add_action(gzclient_cmd)
#     ld.add_action(spawn_robot_cmd)

#     return ld


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('mechelangelo_gazebo')
    description_pkg = get_package_share_directory('mechelangelo_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world = os.path.join(gazebo_pkg, 'worlds', 'Gallery.world')
    sdf_file = os.path.join(gazebo_pkg, 'models', 'mechelangelo_arm', 'arm.sdf')
    urdf_file = os.path.join(description_pkg, 'urdf', 'mechelangelo_arm.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')

    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    new_model_path = os.path.join(gazebo_pkg, 'models')

    if existing_model_path:
        gazebo_model_path = new_model_path + ':' + existing_model_path
    else:
        gazebo_model_path = new_model_path

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mechelangelo',
            '-file', sdf_file,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw
        ],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('x_pose', default_value='1.5'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='1.5'))
    ld.add_action(DeclareLaunchArgument('z_pose', default_value='0.2'))
    ld.add_action(DeclareLaunchArgument('yaw', default_value='4.712'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))

    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''))
    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path))

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_robot_cmd)

    return ld