import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_caatinga_nav = get_package_share_directory('caatinga_nav')

    # Default map and params
    map_file = LaunchConfiguration('map', default=os.path.join(pkg_caatinga_nav, 'maps', 'meu_mapa.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_nav2_bringup, 'params', 'nav2_params.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(pkg_caatinga_nav, 'maps', 'meu_mapa.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg_nav2_bringup, 'params', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': params_file,
                'autostart': 'true',
            }.items()
        )
    ])
