import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg = get_package_share_directory('mechelangelo_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(pkg, 'worlds', 'Gallery_Test2.world')

    # Expose model path so model:// URIs resolve
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    new_model_path = os.path.join(pkg, 'models')
    if existing:
        gazebo_model_path = new_model_path + ':' + existing
    else:
        gazebo_model_path = new_model_path

    ld = LaunchDescription()

    # Optional: allow overriding world file
    ld.add_action(DeclareLaunchArgument('world', default_value=world_file))

    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''))
    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path))

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    ld.add_action(gzserver)
    ld.add_action(gzclient)

    return ld
