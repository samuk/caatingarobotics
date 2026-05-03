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
                {
                    "row_follow_handover_distance": 1.0,
                    "heartbeat_timeout_s": 3.0,
                    "enable_service_timeout_s": 2.0,
                    "monitor_rate_hz": 10.0,
                },
            ],
        ),
    ])
EOF
