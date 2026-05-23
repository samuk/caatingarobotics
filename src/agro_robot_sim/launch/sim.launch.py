import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    pkg_name     = "agro_robot_sim"
    pkg_share    = get_package_share_directory(pkg_name)
    gz_pkg_share = get_package_share_directory("ros_gz_sim")

    # 0. launch arguments
    x_arg = DeclareLaunchArgument("x", default_value="0.0")
    y_arg = DeclareLaunchArgument("y", default_value="0.0")
    z_arg = DeclareLaunchArgument("z", default_value="0.3")

    urdf_arg = DeclareLaunchArgument(
        "urdf",
        default_value="agro_robot.urdf.xacro",
        description="URDF/xacro filename inside agro_robot_sim/urdf/",
    )

    # 1. open gazebo harmonic
    world = LaunchConfiguration("world")
    world_file = PathJoinSubstitution([pkg_share, "worlds", world])
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [TextSubstitution(text="-r "), world_file],
            "on_exit_shutdown": "True",
        }.items(),
    )

    # 2. robot_state_publisher — xacro file resolved from urdf arg
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", LaunchConfiguration("urdf")])
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", xacro_file]), value_type=str
                ),
                "use_sim_time": True,
            }
        ],
    )

    # 3. spawn robot
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_robot",
                output="screen",
                arguments=[
                    "-name",  "agro_robot",
                    "-topic", "/robot_description",
                    "-x", LaunchConfiguration("x"),
                    "-y", LaunchConfiguration("y"),
                    "-z", LaunchConfiguration("z"),
                ],
            )
        ],
    )

    # 4. topic bridge
    ros_gz_bridge = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_bridge",
                output="screen",
                arguments=[
                    "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                    "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                    "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    "/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                ],
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        urdf_arg,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        ros_gz_bridge,
    ])
