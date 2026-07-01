from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("sowbot_row_follow"), "config", "crop_row_params.yaml"]
    )

    return LaunchDescription([
        Node(
            package="sowbot_row_follow",
            executable="limbic_row_follow",
            name="limbic_row_follow",
            output="screen",
            parameters=[
                params_file,
            ],
        ),
    ])
