import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_agro = get_package_share_directory('agro_robot_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # ── Launch arguments ──────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Full path to the SDF world file (empty = Gazebo default empty world)',
    )

    # ── Resource path so Gazebo can find our worlds / models ──────────────────
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_agro, 'worlds'),
    )

    # ── Gazebo server (headless physics) ─────────────────────────────────────
    # -r  : start running immediately
    # -s  : server only (no GUI)
    # -v1 : minimal console verbosity
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v1 ', LaunchConfiguration('world')],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── Gazebo GUI client ─────────────────────────────────────────────────────
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v1'}.items(),
    )

    # ── Process URDF/xacro ────────────────────────────────────────────────────
    xacro_file = os.path.join(pkg_agro, 'urdf', 'robo_caatinga.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # ── robot_state_publisher ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
    )

    # ── Spawn robot into Gazebo Harmonic ──────────────────────────────────────
    # ros_gz_sim create replaces gazebo_ros spawn_entity.py
    # -name replaces -entity, -string replaces -topic for inline URDF
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'agro_robot',
            '-string', robot_desc,
            '-x', '0', '-y', '0', '-z', '0.1',
        ],
        output='screen',
    )

    # ── ros_gz_bridge: gz-transport <-> ROS 2 topic bridges ──────────────────
    # Each entry: gz_topic_name@ros_msg_type[gz_msg_type
    #   [ = gz->ros only  ] = ros->gz only  @ = bidirectional
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            # Drive command (ROS -> Gazebo)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Odometry (Gazebo -> ROS)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # TF from diff-drive (Gazebo -> ROS)
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Joint states (Gazebo -> ROS)
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # LiDAR scan (Gazebo -> ROS)
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # IMU (Gazebo -> ROS)
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # GPS fix (Gazebo -> ROS)
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            # Sim clock (Gazebo -> ROS) – enables use_sim_time
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
    )

    return LaunchDescription([
        world_arg,
        set_gz_resource_path,
        gz_sim_server,
        gz_sim_gui,
        robot_state_publisher,
        spawn_robot,
        gz_bridge,
    ])
