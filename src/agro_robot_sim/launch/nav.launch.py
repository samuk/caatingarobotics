import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('agro_robot_sim')
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pkg_share, 'launch', 'sim.launch.py')))
    slam = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('slam_toolbox'),
                     'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': 'True'}.items())
    return LaunchDescription([sim, slam])
