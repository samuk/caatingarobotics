import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Package directories
    pkg_agro = get_package_share_directory('agro_robot_sim')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_slam = get_package_share_directory('slam_toolbox')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 2. Launch arguments
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='True to build map with SLAM, False to navigate with GPS',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='minha_fazenda.sdf',
        description='World SDF filename (resolved from GZ_SIM_RESOURCE_PATH)',
    )

    slam = LaunchConfiguration('slam')
    world_file_name = LaunchConfiguration('world')

    # 3. File paths
    map_file = os.path.join(pkg_agro, 'maps', 'novo_mapa_fazenda.yaml')
    params_file = os.path.join(pkg_agro, 'config', 'nav2_speed.yaml')
    gps_params_file = os.path.join(pkg_agro, 'config', 'gps_ekf.yaml')
    rviz_nav2 = os.path.join(pkg_nav2, 'rviz', 'nav2_default_view.rviz')
    rviz_slam = os.path.join(pkg_slam, 'config', 'slam_toolbox_default.rviz')

    # 4. Resource path for Gazebo Harmonic world lookup
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_agro, 'worlds'),
    )

    # 5. sim.launch.py (Gazebo Harmonic + robot spawn + gz_bridge)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_agro, 'launch', 'sim.launch.py')
        ),
        launch_arguments={'world': world_file_name}.items(),
    )

    # 6. GPS localisation nodes (robot_localization) — GPS mode only
    navsat_transform_node = Node(
        condition=UnlessCondition(slam),
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[gps_params_file, {'use_sim_time': True}],
        remappings=[
            ('imu', '/imu'),
            ('gps/fix', '/gps/fix'),
            ('gps/filtered', '/gps/filtered'),
            ('odometry/gps', '/odometry/gps'),
        ],
    )

    ekf_filter_node = Node(
        condition=UnlessCondition(slam),
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[gps_params_file, {'use_sim_time': True}],
        remappings=[('odometry/filtered', '/odometry/global')],
    )

    # 7. Nav2 (GPS navigation mode)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        condition=UnlessCondition(slam),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )

    map_server_node = Node(
        condition=UnlessCondition(slam),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_file, 'use_sim_time': True}],
    )

    lifecycle_manager_localization = Node(
        condition=UnlessCondition(slam),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server']},
        ],
    )

    # 8. Nav2 (SLAM mode)
    nav_launch_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        condition=IfCondition(slam),
        launch_arguments={
            'slam': 'True',
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': params_file,
            'autostart': 'true',
        }.items(),
    )

    # 9. Collision monitor
    node_collision_monitor = Node(
        condition=UnlessCondition(slam),
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('cmd_vel_in', 'cmd_vel'),
            ('cmd_vel_out', 'cmd_vel_smoothed'),
        ],
    )

    # 10. RViz
    rviz_cmd_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'rviz_launch.py')
        ),
        condition=UnlessCondition(slam),
        launch_arguments={'use_sim_time': 'true', 'rviz_config': rviz_nav2}.items(),
    )
    rviz_cmd_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'rviz_launch.py')
        ),
        condition=IfCondition(slam),
        launch_arguments={'use_sim_time': 'true', 'rviz_config': rviz_slam}.items(),
    )

    return LaunchDescription([
        slam_arg,
        world_arg,
        set_gz_resource_path,
        LogInfo(
            condition=UnlessCondition(slam),
            msg='GPS mode active: AMCL disabled, map_server active.',
        ),
        sim_launch,
        # Delay Nav2 stack until robot is spawned and /odom TF is publishing.
        TimerAction(
            period=3.5,
            actions=[
                navsat_transform_node,
                ekf_filter_node,
                nav_launch,
                map_server_node,
                lifecycle_manager_localization,
                nav_launch_slam,
                node_collision_monitor,
                rviz_cmd_nav,
                rviz_cmd_slam,
            ],
        ),
    ])
