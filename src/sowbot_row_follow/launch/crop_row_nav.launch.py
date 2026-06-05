#!/usr/bin/env python3

# Copyright 2022 Agricultural-Robotics-Bonn
# Copyright 2026 Agroecology Lab Ltd
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# crop_row_nav.launch.py
# ======================
# Derived from:
#   visual-multi-crop-row-navigation
#   https://github.com/Agricultural-Robotics-Bonn/visual-multi-crop-row-navigation
#   Authors: Alireza Ahmadi, Michael Halstead, Chris McCool (Agricultural-Robotics-Bonn)
#   ROS2 port: Agroecology Lab CIC
#   Reference: Ahmadi et al., "Towards Autonomous Crop-Agnostic Visual Navigation
#              in Arable Fields", arXiv:2109.11936, 2021.
#              Ahmadi et al., ICRA 2020.
#
# Adapted for caatinga_vision / Sowbot by Agroecology Lab Ltd
#
# Launches the USB camera (usb_cam_node_exe), crop_row_node, and
# web_video_server together in one launch session / DDS graph:
#   - usb_cam publishes /devkit/camera/image_raw
#   - crop_row_node subscribes to it, publishes /caatinga_vision/row_nav/debug_image
#   - web_video_server serves ALL image topics over HTTP for browser viewing
#
# Why usb_cam is launched here:
#   On loopback-only CycloneDDS a separately-started process can land in a
#   different discovery context, so crop_row_node saw zero publishers and the
#   feed never appeared. Launching everything from one launch file guarantees
#   they share the same session.
#
# Why web_video_server:
#   Serves the stream over HTTP (default port 8080), so the feed is viewable in
#   any browser with no X-forwarding and no DDS config on the viewer side.
#   Browse all topics at  http://localhost:8080
#   Direct stream:        http://localhost:8080/stream?topic=/caatinga_vision/row_nav/debug_image
#
# QoS note:
#   crop_row_node subscribes to the image topic RELIABLE. usb_cam defaults to
#   BEST_EFFORT, which will NOT connect to a RELIABLE subscriber, so we force
#   usb_cam to publish RELIABLE here via qos_overrides.
#
# Topics published:
#   /devkit/camera/image_raw            Raw camera frames (from usb_cam)
#   /caatinga_vision/row_nav/debug_image  Annotated frames
#   /aoc/conditions/row_offset          Normalised lateral offset [-1,1]
#   /aoc/conditions/row_heading_error   Heading error in radians
#   /aoc/heartbeat/neo_vision           Perception heartbeat for M10
#   /cmd_vel                            Always published; gated by /row_follow/enable service
#
# Usage:
#   ros2 launch sowbot_row_follow crop_row_nav.launch.py
#   ros2 launch sowbot_row_follow crop_row_nav.launch.py video_device:=/dev/video2
#   ros2 launch sowbot_row_follow crop_row_nav.launch.py use_camera:=false        # subscribe only
#   ros2 launch sowbot_row_follow crop_row_nav.launch.py use_web_video:=false     # no HTTP stream
#   ros2 launch sowbot_row_follow crop_row_nav.launch.py web_video_port:=8081

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Base params from file — hardware-specific values live here, not in this launch file.
    # Edit caatinga_vision/config/crop_row_params.yaml to set camera_height_m, camera_tilt_deg etc.
    params_file = PathJoinSubstitution(
        [FindPackageShare("sowbot_row_follow"), "config", "crop_row_params.yaml"]
    )

    image_topic = LaunchConfiguration("image_topic")
    web_video_port = LaunchConfiguration("web_video_port")

    return LaunchDescription([

        # ------------------------------------------------------------------
        # Launch arguments
        # ------------------------------------------------------------------
        DeclareLaunchArgument("camera_index", default_value="0"),
        DeclareLaunchArgument(
            "image_topic", default_value="/devkit/camera/image_raw",
            description="Image topic crop_row_node subscribes to and usb_cam publishes.",
        ),
        DeclareLaunchArgument(
            "video_device", default_value="/dev/video0",
            description="V4L2 device node for the USB camera.",
        ),
        DeclareLaunchArgument(
            "use_camera", default_value="true",
            description="If true, launch usb_cam here. Set false for sim / external / remote camera.",
        ),
        DeclareLaunchArgument(
            "use_web_video", default_value="true",
            description="If true, launch web_video_server to serve image topics over HTTP.",
        ),
        DeclareLaunchArgument(
            "web_video_port", default_value="8080",
            description="HTTP port for web_video_server.",
        ),

        # ------------------------------------------------------------------
        # USB camera — publishes the image topic crop_row_node consumes.
        # Remapped so usb_cam's default /image_raw becomes image_topic.
        # QoS forced RELIABLE to match crop_row_node's subscription.
        # ------------------------------------------------------------------
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="usb_cam",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_camera")),
            parameters=[{
                "video_device": LaunchConfiguration("video_device"),
                "image_width": 640,
                "image_height": 480,
                "pixel_format": "yuyv",
                "framerate": 30.0,
                "camera_name": "default_cam",
                "qos_overrides./devkit/camera/image_raw.publisher.reliability": "reliable",
            }],
            remappings=[
                ("/image_raw", image_topic),
            ],
        ),

        # ------------------------------------------------------------------
        # Crop-row perception / control node
        # ------------------------------------------------------------------
        Node(
            package="sowbot_row_follow",
            executable="crop_row_node",
            name="crop_row_node",
            output="screen",
            parameters=[
                params_file,
                {
                    "camera_index": LaunchConfiguration("camera_index"),
                    "image_topic":  image_topic,
                },
            ],
        ),

        # ------------------------------------------------------------------
        # web_video_server — HTTP stream of all image topics for browser viewing
        # ------------------------------------------------------------------
        Node(
            package="web_video_server",
            executable="web_video_server",
            name="web_video_server",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_web_video")),
            parameters=[{
                "port": web_video_port,
                "address": "0.0.0.0",
            }],
        ),

        # Print the viewing URLs prominently in the launch terminal.
        LogInfo(condition=IfCondition(LaunchConfiguration("use_web_video")),
                msg=["\n",
                     "============================================================\n",
                     "  Camera stream live in your browser:\n",
                     "    All topics : http://localhost:", web_video_port, "\n",
                     "    Debug view : http://localhost:", web_video_port,
                     "/stream?topic=/caatinga_vision/row_nav/debug_image\n",
                     "    Raw camera : http://localhost:", web_video_port,
                     "/stream?topic=/devkit/camera/image_raw\n",
                     "============================================================"]),
    ])
