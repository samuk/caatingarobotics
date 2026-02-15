import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('agro_robot_sim')
    slam_params = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    rviz_cfg = os.path.join(pkg_share, 'config', 'mapping.rviz')

    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pkg_share, 'launch', 'sim.launch.py')))

    slam = TimerAction(period=5.0, actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')),
            launch_arguments={'use_sim_time': 'true',
                               'slam_params_file': slam_params}.items())
    ])

    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', rviz_cfg], output='screen')

    return LaunchDescription([sim, slam, rviz])
