import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('mechelangelo_bringup')
    description_dir = get_package_share_directory('mechelangelo_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')

    world = os.path.join(bringup_dir, 'worlds', 'Gallery_Test2.world')
    urdf_file = os.path.join(description_dir, 'urdf', 'mechelangelo_base.urdf')

    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    new_model_path = os.path.join(bringup_dir, 'Models')
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
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }]
    )

    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mechelangelo',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw
        ],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='3.125'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='2.85'))
    ld.add_action(DeclareLaunchArgument('z_pose', default_value='0.2'))
    ld.add_action(DeclareLaunchArgument('yaw', default_value='4.712'))

    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''))
    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path))

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_robot_cmd)

    return ld

#before adding gazebo model path safeguard

# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node


# def generate_launch_description():
#     bringup_dir = get_package_share_directory('mechelangelo_bringup')
#     description_dir = get_package_share_directory('mechelangelo_description')
#     pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

#     use_sim_time = LaunchConfiguration('use_sim_time', default='true')
#     x_pose = LaunchConfiguration('x_pose', default='3.125')
#     y_pose = LaunchConfiguration('y_pose', default='2.85')
#     z_pose = LaunchConfiguration('z_pose', default='0.2')
#     yaw = LaunchConfiguration('yaw', default='4.712')

#     world = os.path.join(bringup_dir, 'worlds', 'Gallery_Test2.world')
#     urdf_file = os.path.join(description_dir, 'urdf', 'mechelangelo_base.urdf')

#     with open(urdf_file, 'r') as infp:
#         robot_description_content = infp.read()

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

#     robot_state_publisher_cmd = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[{
#             'use_sim_time': use_sim_time,
#             'robot_description': robot_description_content
#         }]
#     )

#     spawn_robot_cmd = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=[
#             '-entity', 'mechelangelo',
#             '-topic', 'robot_description',
#             '-x', x_pose,
#             '-y', y_pose,
#             '-z', z_pose,
#             '-Y', yaw
#         ],
#         output='screen'
#     )

#     ld = LaunchDescription()

#     ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''))
#     ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(bringup_dir, 'Models')))

#     ld.add_action(gzserver_cmd)
#     ld.add_action(gzclient_cmd)
#     ld.add_action(robot_state_publisher_cmd)
#     ld.add_action(spawn_robot_cmd)

#     return ld


###original file


# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration


# def generate_launch_description():
#     # Paths
#     launch_file_dir = os.path.join(get_package_share_directory('mechelangelo_bringup'))
#     pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

#     # Launch configurations
#     use_sim_time = LaunchConfiguration('use_sim_time', default='true')
#     x_pose = LaunchConfiguration('x_pose', default='3.125')
#     y_pose = LaunchConfiguration('y_pose', default='2.85')
#     z_pose = LaunchConfiguration('z_pose', default='0.2')
#     yaw = LaunchConfiguration('yaw', default='4.712')  # 270 degrees in radians




#     # World file path
#     world = os.path.join(
#         get_package_share_directory('mechelangelo_bringup'),
#         'worlds',
#         'Gallery_Test2.world'
#     )

#     # Launch Gazebo server with the world
#     gzserver_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
#         ),
#         launch_arguments={'world': world}.items()
#     )

#     # Launch Gazebo client
#     gzclient_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
#         )
#     )


#     ###CHANGE THIS
#     # Robot state publisher
#     robot_state_publisher_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
#         ),
#         launch_arguments={'use_sim_time': use_sim_time}.items()
#     )

#     # Spawn TurtleBot3
#     spawn_turtlebot_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
#         ),
#         launch_arguments={
#             'x_pose': x_pose,
#             'y_pose': y_pose,
#             'z_pose': z_pose,
#             'yaw': yaw
#         }.items()
#     )

#     #########

#     # Launch description
#     ld = LaunchDescription()

#     # Disable Gazebo network access
#     ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''))

#     # Add actions
#     ld.add_action(gzserver_cmd)
#     ld.add_action(gzclient_cmd)
#     ld.add_action(robot_state_publisher_cmd)
#     ld.add_action(spawn_turtlebot_cmd)

#     return ld



