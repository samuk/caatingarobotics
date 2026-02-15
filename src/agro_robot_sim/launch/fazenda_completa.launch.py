import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             LogInfo, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_agro = get_package_share_directory('agro_robot_sim')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_slam = get_package_share_directory('slam_toolbox')

    slam_arg = DeclareLaunchArgument(
        'slam', default_value='False',
        description='True to build map (SLAM), False to navigate with GPS')
    slam = LaunchConfiguration('slam')

    map_file = os.path.join(pkg_agro, 'maps', 'novo_mapa_fazenda.yaml')
    params_file = os.path.join(pkg_agro, 'config', 'nav2_speed.yaml')
    gps_params = os.path.join(pkg_agro, 'config', 'gps_ekf.yaml')

    sim_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pkg_agro, 'launch', 'sim.launch.py')))

    navsat = Node(condition=UnlessCondition(slam),
                  package='robot_localization', executable='navsat_transform_node',
                  name='navsat_transform', output='screen',
                  arguments=['--ros-args', '--log-level', 'warn'],
                  parameters=[gps_params, {'use_sim_time': True}],
                  remappings=[('imu', '/imu'), ('gps/fix', '/gps/fix'),
                               ('gps/filtered', '/gps/filtered'),
                               ('odometry/gps', '/odometry/gps')])

    ekf = Node(condition=UnlessCondition(slam),
               package='robot_localization', executable='ekf_node',
               name='ekf_filter_node_map', output='screen',
               arguments=['--ros-args', '--log-level', 'warn'],
               parameters=[gps_params, {'use_sim_time': True}],
               remappings=[('odometry/filtered', '/odometry/global')])

    nav = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
        condition=UnlessCondition(slam),
        launch_arguments={'use_sim_time': 'true', 'params_file': params_file,
                           'autostart': 'true', 'use_composition': 'False'}.items())

    map_server = Node(condition=UnlessCondition(slam),
                      package='nav2_map_server', executable='map_server',
                      name='map_server', output='screen',
                      parameters=[params_file, {'yaml_filename': map_file,
                                                 'use_sim_time': True}])

    lm_loc = Node(condition=UnlessCondition(slam),
                  package='nav2_lifecycle_manager', executable='lifecycle_manager',
                  name='lifecycle_manager_localization', output='screen',
                  parameters=[{'use_sim_time': True}, {'autostart': True},
                               {'node_names': ['map_server']}])

    nav_slam = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
        condition=IfCondition(slam),
        launch_arguments={'slam': 'True', 'map': map_file, 'use_sim_time': 'true',
                           'params_file': params_file, 'autostart': 'true'}.items())

    collision = Node(condition=UnlessCondition(slam),
                     package='nav2_collision_monitor', executable='collision_monitor',
                     name='collision_monitor', output='screen', parameters=[params_file],
                     remappings=[('cmd_vel_in', 'cmd_vel'),
                                  ('cmd_vel_out', 'cmd_vel_smoothed')])

    rviz_nav = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pkg_nav2, 'launch', 'rviz_launch.py')),
        condition=UnlessCondition(slam),
        launch_arguments={'use_sim_time': 'true',
                           'rviz_config': os.path.join(pkg_nav2, 'rviz',
                                                        'nav2_default_view.rviz')}.items())

    rviz_slam = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pkg_nav2, 'launch', 'rviz_launch.py')),
        condition=IfCondition(slam),
        launch_arguments={'use_sim_time': 'true',
                           'rviz_config': os.path.join(pkg_slam, 'config',
                                                        'slam_toolbox_default.rviz')}.items())

    return LaunchDescription([
        slam_arg,
        LogInfo(condition=UnlessCondition(slam),
                msg='GPS mode active: AMCL disabled, map_server active.'),
        sim_launch,
        TimerAction(period=3.5, actions=[
            navsat, ekf, nav, map_server, lm_loc, nav_slam, collision, rviz_nav, rviz_slam
        ]),
    ])
