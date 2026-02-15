import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_agro = get_package_share_directory('agro_robot_sim')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_agro, 'worlds', 'minha_fazenda.world'),
        description='Full path to world file',
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': LaunchConfiguration('world')}.items(),
    )

    xacro_file = os.path.join(pkg_agro, 'urdf', 'robo_caatinga.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'agro_robot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.3',
        ],
        output='screen',
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen',
    )

    return LaunchDescription([world_arg, gz_sim, robot_state_publisher, spawn_robot, gz_bridge])
